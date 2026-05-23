#!/usr/bin/env python3
# encoding: utf-8

"""
Robot Camera MJPEG Stream Server

Run on Raspberry Pi:
    python3 test_robot_camera_stream.py --host 127.0.0.1 --port 8090

Then from laptop:
    ssh -L 8090:localhost:8090 pi@ROBOT_IP

Laptop can test:
    http://127.0.0.1:8090/video
"""

import argparse
import http.server
import os
import threading
import time

import cv2


_frame_lock = threading.Lock()
_jpeg_frame = b""
_running = True


def open_robot_camera(width=640, height=480):
    """
    Try common robot camera devices.
    This matches the style of your test_vision.py camera setup.
    """
    camera_options = [
        "/dev/video0",
        "/dev/video1",
        "/dev/usb_cam",
        0,
        1,
        2,
    ]

    for device in camera_options:
        if isinstance(device, str) and not os.path.exists(device):
            continue

        print(f"[CAM] trying camera: {device}")
        cap = cv2.VideoCapture(device)

        if cap.isOpened():
            ret, frame = cap.read()

            if ret and frame is not None:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

                actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

                print(f"[CAM] opened {device}: {actual_w}x{actual_h}")
                return cap, device

            cap.release()

    return None, None


def capture_loop(width=640, height=480, quality=75, fps=30):
    global _jpeg_frame

    cap, device = open_robot_camera(width=width, height=height)

    if cap is None:
        print("[CAM] ERROR: could not open robot camera")
        return

    delay = 1.0 / max(1, fps)

    print("[CAM] capture loop running")

    while _running:
        ret, frame = cap.read()

        if not ret or frame is None:
            time.sleep(0.05)
            continue

        ok, jpeg = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, int(quality)],
        )

        if ok:
            with _frame_lock:
                _jpeg_frame = jpeg.tobytes()

        time.sleep(delay)

    cap.release()
    print("[CAM] stopped")


class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/video":
            self.stream_mjpeg()
        elif self.path == "/health":
            self.health()
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"not found\n")

    def health(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.end_headers()
        self.wfile.write(b"ok\n")

    def stream_mjpeg(self):
        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        try:
            while True:
                with _frame_lock:
                    frame = _jpeg_frame

                if frame:
                    self.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(frame)).encode("ascii") + b"\r\n"
                        b"\r\n" + frame + b"\r\n"
                    )
                    self.wfile.flush()

                time.sleep(1 / 30)

        except BrokenPipeError:
            pass
        except ConnectionResetError:
            pass
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--quality", type=int, default=75)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    threading.Thread(
        target=capture_loop,
        args=(args.width, args.height, args.quality, args.fps),
        daemon=True,
    ).start()

    with http.server.ThreadingHTTPServer((args.host, args.port), Handler) as httpd:
        print(f"[HTTP] robot camera stream: http://{args.host}:{args.port}/video")
        print(f"[HTTP] health check:        http://{args.host}:{args.port}/health")
        httpd.serve_forever()


if __name__ == "__main__":
    main()