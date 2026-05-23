#!/usr/bin/env python3
# encoding: utf-8

"""
AINEX Robot TCP Teleop Server

Run on Raspberry Pi:
    python3 test_teleop.py --host 127.0.0.1 --port 9999

From laptop through SSH tunnel:
    ssh -L 9999:localhost:9999 pi@ROBOT_IP
    nc localhost 9999

Then type commands:
    help
    list
    w
    a
    d
    s
    wave
    reset
    battery
    action walk_ready
    quit
"""

import argparse
import socketserver
import threading
import traceback
import time

import test_skills as skills
from ainex_sdk import Board


# Short teleop aliases.
# Change these to match the action names that work best on your robot.
ALIASES = {
    "w": "walk",
    "s": "walk_backward",
    "a": "walk_turn_left",
    "d": "walk_turn_right",
    "x": "walk_ready",
    "r": "reset",
    "b": "battery",
    "q": "quit",

    "1": "wave",
    "2": "look_around",
    "3": "nod",
    "4": "shake",
    "5": "arms_up",
    "6": "dance",
    "7": "grab_right",
    "8": "drop_right",
}


class RobotState:
    def __init__(self, board, token=None):
        self.board = board
        self.token = token
        self.lock = threading.Lock()
        self.shutdown_requested = False


def run_skill(robot, skill_name):
    """
    Run one registered skill safely.
    Only one skill runs at a time.
    """
    skill_name = ALIASES.get(skill_name, skill_name)

    if skill_name not in skills.SKILLS:
        return f"ERROR: Unknown skill '{skill_name}'. Type 'list'.\n"

    name, func = skills.SKILLS[skill_name]

    if not robot.lock.acquire(blocking=False):
        return "BUSY: Robot is already running a skill. Wait for it to finish.\n"

    try:
        print(f"[TELEOP] Running skill: {skill_name} - {name}")
        func(robot.board)
        return f"OK: finished {skill_name} - {name}\n"
    except Exception as e:
        traceback.print_exc()
        return f"ERROR while running {skill_name}: {e}\n"
    finally:
        robot.lock.release()


def run_action(robot, action_name):
    """
    Play a raw .d6a action file by name.
    Example command:
        action walk_ready
        action left_shot
    """
    if not action_name:
        return "ERROR: usage: action <action_name>\n"

    if not robot.lock.acquire(blocking=False):
        return "BUSY: Robot is already running a skill/action.\n"

    try:
        print(f"[TELEOP] Running action: {action_name}")
        if skills.action_player is None:
            skills.action_player = skills.ActionPlayer(robot.board)

        if not skills.action_player.action_exists(action_name):
            available = skills.action_player.list_actions()
            return (
                f"ERROR: action '{action_name}' not found.\n"
                f"Available actions: {available}\n"
            )

        skills.action_player.play_action(action_name)
        return f"OK: finished action {action_name}\n"
    except Exception as e:
        traceback.print_exc()
        return f"ERROR while running action {action_name}: {e}\n"
    finally:
        robot.lock.release()


def help_text():
    return """
AINEX TCP Teleop Commands

Movement aliases:
  w                 walk forward
  s                 walk backward
  a                 turn left
  d                 turn right
  x                 walk_ready pose

Quick skills:
  1                 wave
  2                 look_around
  3                 nod
  4                 shake
  5                 arms_up
  6                 dance
  7                 grab_right
  8                 drop_right

Named commands:
  list              list available skills
  actions           list available .d6a action files
  <skill_name>      run skill from test_skills.SKILLS
  action <name>     play raw .d6a action file
  reset             reset pose
  battery           read battery
  stop              request current action_player stop
  quit              close this connection
  shutdown          stop teleop server

Examples:
  wave
  walk_ready
  action left_shot
  action walk_ready
""".lstrip()


def handle_command(robot, raw_command):
    command = raw_command.strip()

    if not command:
        return ""

    # Optional simple token check:
    # If started with --token abc123, every command must be:
    #   abc123 wave
    if robot.token is not None:
        parts = command.split(maxsplit=1)
        if len(parts) == 0 or parts[0] != robot.token:
            return "ERROR: bad or missing token.\n"
        command = parts[1] if len(parts) > 1 else ""

    command = command.strip()
    lower = command.lower()

    if lower in ["help", "h", "?"]:
        return help_text()

    if lower in ["quit", "exit", "q"]:
        return "QUIT\n"

    if lower == "shutdown":
        robot.shutdown_requested = True
        return "SHUTDOWN\n"

    if lower == "list":
        lines = ["Available skills:"]
        for key, (name, _) in skills.SKILLS.items():
            lines.append(f"  {key:<18} {name}")
        return "\n".join(lines) + "\n"

    if lower == "actions":
        if skills.action_player is None:
            skills.action_player = skills.ActionPlayer(robot.board)
        actions = skills.action_player.list_actions()
        return "Available actions:\n" + "\n".join(f"  {a}" for a in actions) + "\n"

    if lower == "stop":
        if skills.action_player is not None:
            skills.action_player.stop()
            return "OK: stop requested for current action_player action.\n"
        return "OK: no action_player exists yet.\n"

    if lower.startswith("action "):
        action_name = command.split(maxsplit=1)[1].strip()
        return run_action(robot, action_name)

    return run_skill(robot, lower)


class TeleopHandler(socketserver.StreamRequestHandler):
    def handle(self):
        robot = self.server.robot

        peer = self.client_address
        print(f"[TELEOP] Client connected: {peer}")

        self.wfile.write(b"AINEX teleop connected. Type 'help'.\n> ")
        self.wfile.flush()

        while True:
            line = self.rfile.readline()
            if not line:
                break

            raw = line.decode("utf-8", errors="replace").strip()
            response = handle_command(robot, raw)

            if response == "QUIT\n":
                self.wfile.write(b"Goodbye.\n")
                self.wfile.flush()
                break

            if response == "SHUTDOWN\n":
                self.wfile.write(b"Server shutting down.\n")
                self.wfile.flush()
                threading.Thread(target=self.server.shutdown, daemon=True).start()
                break

            self.wfile.write(response.encode("utf-8"))
            self.wfile.write(b"> ")
            self.wfile.flush()

            if robot.shutdown_requested:
                break

        print(f"[TELEOP] Client disconnected: {peer}")


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True


def main():
    parser = argparse.ArgumentParser(description="AINEX TCP Teleop Server")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind address. Use 127.0.0.1 for SSH tunnel safety. Use 0.0.0.0 for direct LAN access.")
    parser.add_argument("--port", type=int, default=9999)
    parser.add_argument("--token", default=None,
                        help="Optional command token. If set, commands must be: TOKEN command")
    parser.add_argument("--no-walk-ready-on-exit", action="store_true")
    args = parser.parse_args()

    print("AINEX TCP Teleop Server")
    print("Initializing board...")

    try:
        board = Board()
        print("Board connected successfully.")
    except Exception as e:
        print(f"ERROR: could not connect to board: {e}")
        return

    # Initialize the shared action player used by many skills.
    skills.action_player = skills.ActionPlayer(board)

    robot = RobotState(board=board, token=args.token)

    server = ThreadedTCPServer((args.host, args.port), TeleopHandler)
    server.robot = robot

    print(f"Listening on {args.host}:{args.port}")
    if args.host == "127.0.0.1":
        print("Use SSH tunnel from laptop:")
        print(f"  ssh -L {args.port}:localhost:{args.port} pi@ROBOT_IP")
        print(f"  nc localhost {args.port}")
    else:
        print("Direct LAN mode. Consider using --token for safety.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Shutting down teleop server...")
        server.server_close()

        if not args.no_walk_ready_on_exit:
            print("Returning to walk_ready position...")
            try:
                skills.skill_walk_ready(board)
            except Exception:
                pass

        print("Done.")


if __name__ == "__main__":
    main()