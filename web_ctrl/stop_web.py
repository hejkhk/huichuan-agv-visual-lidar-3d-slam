import os
import signal
import subprocess
import time
from pathlib import Path


PORT = 5173
PID_FILE = Path(__file__).with_name("web_server.pid")


def run_command(args):
    try:
        return subprocess.run(args, text=True, capture_output=True, check=False)
    except OSError:
        return None


def pid_exists(pid):
    if pid <= 0:
        return False
    if os.name == "nt":
        result = run_command(["tasklist", "/FI", f"PID eq {pid}"])
        return bool(result and str(pid) in result.stdout)
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def kill_pid(pid):
    if not pid_exists(pid):
        return False
    if os.name == "nt":
        run_command(["taskkill", "/PID", str(pid), "/T", "/F"])
    else:
        os.kill(pid, signal.SIGTERM)
    return True


def pids_listening_on_port(port):
    pids = set()
    if os.name == "nt":
        result = run_command([
            "powershell",
            "-NoProfile",
            "-Command",
            (
                f"Get-NetTCPConnection -LocalPort {port} -State Listen "
                f"-ErrorAction SilentlyContinue | Select-Object -ExpandProperty OwningProcess"
            ),
        ])
        if result:
            for line in result.stdout.splitlines():
                line = line.strip()
                if line.isdigit():
                    pids.add(int(line))
    else:
        result = run_command(["sh", "-lc", f"lsof -ti tcp:{port}"])
        if result:
            for line in result.stdout.splitlines():
                line = line.strip()
                if line.isdigit():
                    pids.add(int(line))
    return sorted(pids)


def main():
    stopped = []

    if PID_FILE.exists():
        try:
            pid = int(PID_FILE.read_text(encoding="utf-8").strip())
            if kill_pid(pid):
                stopped.append(pid)
        except ValueError:
            pass

    time.sleep(0.3)
    for pid in pids_listening_on_port(PORT):
        if pid not in stopped and kill_pid(pid):
            stopped.append(pid)

    if PID_FILE.exists():
        PID_FILE.unlink()

    if stopped:
        print(f"Web server stopped. PID: {', '.join(map(str, stopped))}")
    else:
        print(f"No web server found on port {PORT}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
