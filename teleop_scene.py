#!/usr/bin/env python3
# encoding: utf-8

"""
VR Robot Teleop Scene Server

This runs on the laptop.

It:
  1. Serves teleop_scene.html over HTTP.
  2. Proxies the Raspberry Pi robot camera MJPEG stream into /robot_video.
  3. Receives Quest/WebXR controller packets over WebSocket.
  4. Sends A/B/X/Y commands to the robot's test_teleop.py TCP server.

Expected setup:

On Raspberry Pi, terminal 1:
    python3 test_teleop.py --host 127.0.0.1 --port 9999

On Raspberry Pi, terminal 2:
    python3 robot_camera_stream.py --host 127.0.0.1 --port 8090

On laptop:
    ssh -L 9999:localhost:9999 -L 8090:localhost:8090 pi@ROBOT_IP

Then run on laptop:
    python3 teleop_scene.py

Open on Quest:
    http://LAPTOP_IP:8080/teleop_scene.html

Default controls:
    A -> walk forward
    B -> walk backward
    X -> turn left
    Y -> turn right
"""

import argparse
import asyncio
import functools
import http.server
import json
import os
import socket
import threading
import time
import urllib.request

import websockets


# =============================================================================
# CONFIG
# =============================================================================

HTTP_PORT = 8080
WS_PORT = 5005

# Robot teleop TCP server.
# Usually localhost because of:
#   ssh -L 9999:localhost:9999 pi@ROBOT_IP
ROBOT_HOST = "127.0.0.1"
ROBOT_PORT = 9999

# Robot camera stream.
# Usually localhost because of:
#   ssh -L 8090:localhost:8090 pi@ROBOT_IP
ROBOT_CAMERA_URL = "http://127.0.0.1:8090/video"

BUTTON_COMMANDS = {
    "A": "walk",
    "B": "walk_backward",
    "X": "walk_turn_left",
    "Y": "walk_turn_right",
}


# =============================================================================
# HTTP SERVER
# =============================================================================

class RobotHTTPHandler(http.server.SimpleHTTPRequestHandler):
    """
    Serves:
      - static files from the current folder
      - /robot_video as proxied MJPEG from the Raspberry Pi
      - /health as a simple check
    """

    robot_camera_url = ROBOT_CAMERA_URL

    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/robot_video":
            self.proxy_robot_video()
        elif self.path == "/health":
            self.health()
        else:
            super().do_GET()

    def health(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.end_headers()
        self.wfile.write(b"ok\n")

    def proxy_robot_video(self):
        """
        Quest sees:
            http://LAPTOP_IP:8080/robot_video

        Laptop internally pulls:
            http://127.0.0.1:8090/video

        That URL should be forwarded to the Pi with:
            ssh -L 8090:localhost:8090 pi@ROBOT_IP
        """
        upstream = None

        try:
            print(f"[ROBOT_CAM] connecting to {self.robot_camera_url}")
            upstream = urllib.request.urlopen(self.robot_camera_url, timeout=5)

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Pragma", "no-cache")
            self.end_headers()

            print("[ROBOT_CAM] proxy connected")

            while True:
                chunk = upstream.read(4096)

                if not chunk:
                    print("[ROBOT_CAM] upstream ended")
                    break

                self.wfile.write(chunk)
                self.wfile.flush()

        except BrokenPipeError:
            pass
        except ConnectionResetError:
            pass
        except Exception as e:
            print(f"[ROBOT_CAM] proxy error: {e}")

            try:
                self.send_response(502)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(
                    f"Robot camera proxy error: {e}\n".encode("utf-8")
                )
            except Exception:
                pass

        finally:
            if upstream is not None:
                try:
                    upstream.close()
                except Exception:
                    pass


def run_http(http_host, http_port, directory, robot_camera_url):
    handler_cls = type(
        "ConfiguredRobotHTTPHandler",
        (RobotHTTPHandler,),
        {"robot_camera_url": robot_camera_url},
    )

    handler = functools.partial(handler_cls, directory=directory)

    with http.server.ThreadingHTTPServer((http_host, http_port), handler) as httpd:
        print(f"[HTTP] serving directory: {directory}")
        print(f"[HTTP] page server:        http://{http_host}:{http_port}")
        print(f"[HTTP] robot camera proxy: /robot_video -> {robot_camera_url}")
        print(f"[HTTP] Quest URL:          http://LAPTOP_IP:{http_port}/teleop_scene.html")
        httpd.serve_forever()


# =============================================================================
# ROBOT TCP COMMAND CLIENT
# =============================================================================

def send_robot_command(cmd, robot_host=ROBOT_HOST, robot_port=ROBOT_PORT, token=None):
    """
    Send one command to test_teleop.py.

    If using SSH tunnel:
        ssh -L 9999:localhost:9999 pi@ROBOT_IP

    This connects to:
        127.0.0.1:9999
    """
    if not cmd:
        return False, "empty command"

    outgoing = cmd

    if token:
        outgoing = f"{token} {cmd}"

    try:
        with socket.create_connection((robot_host, robot_port), timeout=1.0) as s:
            # Try to read greeting.
            s.settimeout(0.2)

            try:
                _ = s.recv(4096)
            except Exception:
                pass

            s.sendall((outgoing + "\n").encode("utf-8"))

            # Try to read response.
            s.settimeout(3.0)

            try:
                resp = s.recv(4096).decode("utf-8", errors="replace")
                resp = resp.strip()
            except Exception:
                resp = ""

            print(f"[ROBOT] {cmd} -> {resp or 'sent'}")
            return True, resp or "sent"

    except Exception as e:
        msg = f"failed to send '{cmd}' to {robot_host}:{robot_port}: {e}"
        print(f"[ROBOT] {msg}")
        return False, msg


# =============================================================================
# WEBSOCKET BRIDGE
# =============================================================================

class VRRobotBridge:
    """
    Receives VR controller packets and sends robot commands on button rising edges.
    """

    def __init__(
        self,
        robot_host=ROBOT_HOST,
        robot_port=ROBOT_PORT,
        robot_token=None,
        cooldown_s=0.35,
        verbose_pose=False,
        verbose_buttons=True,
    ):
        self.robot_host = robot_host
        self.robot_port = robot_port
        self.robot_token = robot_token
        self.cooldown_s = cooldown_s
        self.verbose_pose = verbose_pose
        self.verbose_buttons = verbose_buttons

        self.last_buttons = {
            "A": False,
            "B": False,
            "X": False,
            "Y": False,
        }

        self.last_command_time = 0.0

    async def handle_packet(self, packet):
        self.print_debug(packet)

        buttons = packet.get("buttons", {})
        now = time.time()

        for button_name, robot_cmd in BUTTON_COMMANDS.items():
            pressed = bool(buttons.get(button_name, False))
            was_pressed = self.last_buttons.get(button_name, False)

            rising_edge = pressed and not was_pressed
            cooldown_ok = (now - self.last_command_time) >= self.cooldown_s

            if rising_edge and cooldown_ok:
                print(f"[VR] {button_name} pressed -> {robot_cmd}")

                ok, resp = await asyncio.to_thread(
                    send_robot_command,
                    robot_cmd,
                    self.robot_host,
                    self.robot_port,
                    self.robot_token,
                )

                self.last_command_time = time.time()

                if not ok:
                    print(f"[VR] robot command failed: {resp}")

            self.last_buttons[button_name] = pressed

    def print_debug(self, packet):
        if self.verbose_pose:
            parts = []

            for hand in ("left", "right"):
                if hand not in packet:
                    continue

                h = packet[hand]
                pos = h.get("pos") or [0, 0, 0]
                ori = h.get("ori") or [0, 0, 0, 1]
                trig = h.get("trigger", 0)
                grip = h.get("grip", 0)

                parts.append(
                    f"{hand[0].upper()}: "
                    f"pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
                    f"ori=({ori[0]:+.3f},{ori[1]:+.3f},{ori[2]:+.3f},{ori[3]:+.3f}) "
                    f"trig={trig:.2f} grip={grip:.2f}"
                )

            if parts:
                print("  |  ".join(parts))

        if self.verbose_buttons:
            buttons = packet.get("buttons", {})

            if buttons:
                pressed = [k for k, v in buttons.items() if v]

                if pressed:
                    print(f"[BUTTONS] pressed: {pressed}")


async def ws_handler(websocket, bridge):
    peer = getattr(websocket, "remote_address", None)
    print(f"[WS] client connected: {peer}")

    try:
        async for msg in websocket:
            try:
                packet = json.loads(msg)
                await bridge.handle_packet(packet)

            except json.JSONDecodeError:
                print("[WS] bad JSON packet")

            except Exception as e:
                print(f"[WS] error handling packet: {e}")

    except websockets.ConnectionClosed:
        pass

    finally:
        print(f"[WS] client disconnected: {peer}")


async def run_ws(ws_host, ws_port, bridge):
    async def handler(websocket):
        await ws_handler(websocket, bridge)

    async with websockets.serve(handler, ws_host, ws_port):
        print(f"[WS] ws://{ws_host}:{ws_port}")
        await asyncio.Future()


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="VR WebXR Robot Teleop Scene")

    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=HTTP_PORT)

    parser.add_argument("--ws-host", default="0.0.0.0")
    parser.add_argument("--ws-port", type=int, default=WS_PORT)

    parser.add_argument("--robot-host", default=ROBOT_HOST)
    parser.add_argument("--robot-port", type=int, default=ROBOT_PORT)
    parser.add_argument("--robot-token", default=None)

    parser.add_argument("--robot-camera-url", default=ROBOT_CAMERA_URL)

    parser.add_argument("--cooldown", type=float, default=0.35)
    parser.add_argument("--verbose-pose", action="store_true")
    parser.add_argument("--quiet-buttons", action="store_true")

    args = parser.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))

    threading.Thread(
        target=run_http,
        args=(args.http_host, args.http_port, here, args.robot_camera_url),
        daemon=True,
    ).start()

    bridge = VRRobotBridge(
        robot_host=args.robot_host,
        robot_port=args.robot_port,
        robot_token=args.robot_token,
        cooldown_s=args.cooldown,
        verbose_pose=args.verbose_pose,
        verbose_buttons=not args.quiet_buttons,
    )

    print("[MAP] A -> walk")
    print("[MAP] B -> walk_backward")
    print("[MAP] X -> walk_turn_left")
    print("[MAP] Y -> walk_turn_right")
    print(f"[ROBOT] command target tcp://{args.robot_host}:{args.robot_port}")
    print(f"[ROBOT_CAM] source {args.robot_camera_url}")
    print("[HTML] teleop_scene.html should use: camImg.src = '/robot_video';")

    asyncio.run(run_ws(args.ws_host, args.ws_port, bridge))


if __name__ == "__main__":
    main()