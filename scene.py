import asyncio, functools, http.server, json, os, threading, time, websockets
import cv2

HTTP_PORT = 8080
WS_PORT   = 5005

# ── Webcam capture ───────────────────────────────────────────────────────────
_frame_lock = threading.Lock()
_jpeg_frame = b''

def _capture_loop():
    global _jpeg_frame
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[CAM] could not open webcam")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("[CAM] webcam open")
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with _frame_lock:
            _jpeg_frame = jpeg.tobytes()

threading.Thread(target=_capture_loop, daemon=True).start()

# ── HTTP server ──────────────────────────────────────────────────────────────
class _Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        if self.path == '/video':
            self._stream_mjpeg()
        else:
            super().do_GET()

    def _stream_mjpeg(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    frame = _jpeg_frame
                if frame:
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                    )
                time.sleep(1 / 30)
        except Exception:
            pass

def _run_http():
    here    = os.path.dirname(os.path.abspath(__file__))
    handler = functools.partial(_Handler, directory=here)
    with http.server.ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), handler) as httpd:
        print(f"[HTTP] http://0.0.0.0:{HTTP_PORT}")
        print(f"[HTTP] open on Quest: http://localhost:{HTTP_PORT}/scene.html")
        httpd.serve_forever()

async def _ws_handler(websocket):
    async for msg in websocket:
        try:
            d = json.loads(msg)
            parts = []
            for hand in ("left", "right"):
                if hand not in d:
                    continue
                h    = d[hand]
                pos  = h.get("pos") or [0, 0, 0]
                ori  = h.get("ori") or [0, 0, 0, 1]
                trig = h.get("trigger", 0)
                grip = h.get("grip", 0)
                parts.append(
                    f"{hand[0].upper()}: pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})"
                    f" ori=({ori[0]:+.3f},{ori[1]:+.3f},{ori[2]:+.3f},{ori[3]:+.3f})"
                    f" trig={trig:.2f} grip={grip:.2f}"
                )
            if parts:
                print("  |  ".join(parts))
        except Exception as e:
            print(f"[WS] bad packet: {e}")

async def _main():
    async with websockets.serve(_ws_handler, "0.0.0.0", WS_PORT):
        print(f"[WS]  ws://0.0.0.0:{WS_PORT}")
        await asyncio.Future()

if __name__ == "__main__":
    threading.Thread(target=_run_http, daemon=True).start()
    asyncio.run(_main())
