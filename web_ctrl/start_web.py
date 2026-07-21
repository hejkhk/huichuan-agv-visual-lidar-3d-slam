import os
import socket
import subprocess
import time
from pathlib import Path


PORT = 5173
ROOT_DIR = Path(__file__).resolve().parents[1]
PID_FILE = Path(__file__).with_name("web_server.pid")
LOG_FILE = Path(__file__).with_name("web_server.log")


def is_port_open(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.4)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def local_ip_candidates():
    ips = []
    try:
        hostname = socket.gethostname()
        for ip in socket.gethostbyname_ex(hostname)[2]:
            if ip and not ip.startswith("127.") and ip not in ips:
                ips.append(ip)
    except OSError:
        pass

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            ip = sock.getsockname()[0]
            if ip and not ip.startswith("127.") and ip not in ips:
                ips.insert(0, ip)
    except OSError:
        pass
    return ips


def find_web_dir():
    direct_candidates = [ROOT_DIR / "web", ROOT_DIR / "Web"]
    for candidate in direct_candidates:
        if (candidate / "package.json").exists():
            return candidate

    for child in ROOT_DIR.iterdir():
        if not child.is_dir():
            continue
        if (child / "package.json").exists() and (child / "src" / "main.js").exists():
            return child
    return None


def find_node_exe():
    if os.name != "nt":
        return "node"

    candidates = [
        Path(os.environ.get("LOCALAPPDATA", "")) / "hermes" / "node" / "node.exe",
        Path(os.environ.get("ProgramFiles", "")) / "nodejs" / "node.exe",
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return "node.exe"


def print_urls():
    print(f"Local: http://127.0.0.1:{PORT}")
    for ip in local_ip_candidates():
        print(f"LAN:   http://{ip}:{PORT}")


def windows_creationflags():
    if os.name != "nt":
        return 0

    flags = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.DETACHED_PROCESS
    flags |= getattr(subprocess, "CREATE_NO_WINDOW", 0)
    flags |= getattr(subprocess, "CREATE_BREAKAWAY_FROM_JOB", 0)
    return flags


def main():
    web_dir = find_web_dir()
    if web_dir is None:
        print(f"Cannot find web project under: {ROOT_DIR}")
        return 1

    vite_js = web_dir / "node_modules" / "vite" / "bin" / "vite.js"
    if not vite_js.exists():
        print(f"Cannot find Vite: {vite_js}")
        print("Please run npm install in the web project first.")
        return 1

    if is_port_open(PORT):
        print(f"Web server is already running on port {PORT}.")
        print_urls()
        return 0

    node_exe = find_node_exe()
    with open(LOG_FILE, "a", encoding="utf-8") as log:
        log.write("\n\n===== start web server =====\n")
        log.write(f"web_dir={web_dir}\n")
        log.write(f"node={node_exe}\n")
        log.flush()

        process = subprocess.Popen(
            [node_exe, str(vite_js), "--host", "0.0.0.0"],
            cwd=str(web_dir),
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            creationflags=windows_creationflags(),
            close_fds=True,
        )

    PID_FILE.write_text(str(process.pid), encoding="utf-8")
    print(f"Starting web server, PID={process.pid}")

    for _ in range(30):
        if is_port_open(PORT):
            print("Web server started.")
            print_urls()
            print(f"Log: {LOG_FILE}")
            return 0
        time.sleep(0.3)

    print("Web server may have failed to start. Check log:")
    print(LOG_FILE)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
