import asyncio, functools, http.server, json, os, threading, websockets

HTTP_PORT = 8080
WS_PORT   = 5005

class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a): pass

def _run_http():
    here    = os.path.dirname(os.path.abspath(__file__))
    handler = functools.partial(_QuietHandler, directory=here)
    with http.server.HTTPServer(("0.0.0.0", HTTP_PORT), handler) as httpd:
        print(f"[HTTP] serving at http://0.0.0.0:{HTTP_PORT}")
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
