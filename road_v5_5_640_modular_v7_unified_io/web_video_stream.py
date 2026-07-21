import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2 as cv
import numpy as np


_streamer = None


def _local_ip_candidates():
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


class _WebVideoStreamer:
    def __init__(self, host="0.0.0.0", port=8080, fps=12, quality=75):
        self.host = host
        self.port = int(port)
        self.fps = max(1, int(fps))
        self.quality = int(max(20, min(95, quality)))
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.latest_shape = None
        self.frame_count = 0
        self.started_at = time.time()
        self.running = False
        self.server = None
        self.thread = None

    def publish(self, frame):
        if frame is None:
            return
        if frame.ndim == 2:
            frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)

        ok, encoded = cv.imencode(
            ".jpg",
            frame,
            [int(cv.IMWRITE_JPEG_QUALITY), self.quality],
        )
        if not ok:
            return

        with self.lock:
            self.latest_jpeg = encoded.tobytes()
            self.latest_shape = tuple(frame.shape[:2])
            self.frame_count += 1

    def snapshot(self):
        with self.lock:
            return self.latest_jpeg, self.latest_shape, self.frame_count

    def health(self):
        with self.lock:
            has_frame = self.latest_jpeg is not None
            shape = self.latest_shape
            count = self.frame_count
        return {
            "status": "ok",
            "camera": "Gemini2",
            "stream": "/video_feed",
            "has_frame": has_frame,
            "frame_count": count,
            "shape": shape,
            "fps_limit": self.fps,
            "uptime_sec": round(time.time() - self.started_at, 2),
        }

    def placeholder_jpeg(self):
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        cv.putText(
            image,
            "Waiting for camera frame...",
            (58, 185),
            cv.FONT_HERSHEY_SIMPLEX,
            0.8,
            (220, 220, 220),
            2,
            cv.LINE_AA,
        )
        ok, encoded = cv.imencode(".jpg", image, [int(cv.IMWRITE_JPEG_QUALITY), self.quality])
        return encoded.tobytes() if ok else b""

    def start(self):
        if self.running:
            return

        streamer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def _cors(self):
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Headers", "*")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")

            def do_OPTIONS(self):
                self.send_response(204)
                self._cors()
                self.end_headers()

            def do_GET(self):
                if self.path.startswith("/health"):
                    body = json.dumps(streamer.health()).encode("utf-8")
                    self.send_response(200)
                    self._cors()
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path.startswith("/tx_status"):
                    try:
                        from serial_comm import get_latest_tx_status

                        status = get_latest_tx_status()
                    except Exception as exc:
                        status = {
                            "has_tx": False,
                            "error": str(exc),
                            "command": "",
                            "frame": "",
                        }
                    body = json.dumps(status, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self._cors()
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path.startswith("/snapshot.jpg"):
                    jpeg, _, _ = streamer.snapshot()
                    body = jpeg or streamer.placeholder_jpeg()
                    self.send_response(200)
                    self._cors()
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path.startswith("/video_feed"):
                    self.send_response(200)
                    self._cors()
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    delay = 1.0 / streamer.fps
                    last_count = -1

                    while streamer.running:
                        jpeg, _, count = streamer.snapshot()
                        if jpeg is None:
                            jpeg = streamer.placeholder_jpeg()
                        elif count == last_count:
                            time.sleep(min(delay, 0.05))
                            continue

                        last_count = count
                        try:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                            self.wfile.flush()
                        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                            break
                        time.sleep(delay)
                    return

                self.send_response(404)
                self._cors()
                self.end_headers()

        self.server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.server.daemon_threads = True
        self.running = True
        self.thread = threading.Thread(target=self.server.serve_forever, name="web_video_stream", daemon=True)
        self.thread.start()
        print(f"WEB_VIDEO stream started on 0.0.0.0:{self.port}")
        print(f"WEB_VIDEO local:  http://127.0.0.1:{self.port}/video_feed")
        for ip in _local_ip_candidates():
            print(f"WEB_VIDEO LAN:    http://{ip}:{self.port}/video_feed")
        print(f"WEB_VIDEO health: http://127.0.0.1:{self.port}/health")

    def stop(self):
        if not self.running:
            return
        self.running = False
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        if self.thread is not None:
            self.thread.join(timeout=1.5)
        print("WEB_VIDEO stream stopped")


def start_web_video_server(host="0.0.0.0", port=8080, fps=12, quality=75):
    global _streamer
    if _streamer is None:
        _streamer = _WebVideoStreamer(host=host, port=port, fps=fps, quality=quality)
    _streamer.start()
    return _streamer


def stop_web_video_server():
    global _streamer
    if _streamer is not None:
        _streamer.stop()
        _streamer = None


def publish_web_frame(frame):
    if _streamer is not None:
        _streamer.publish(frame)
