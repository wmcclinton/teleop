#!/usr/bin/env python3
# encoding: utf-8

"""
AINEX Advanced TCP Teleop Server

This does:
  - w/a/s/d movement commands
  - skill/action playback from test_skills.py
  - VR position-based arm/end-effector control from Meta Quest
  - right Meta Quest joystick head control

Run on Raspberry Pi:
    python3 advanced_test_teleop.py --host 127.0.0.1 --port 9999

From laptop:
    ssh -L 9999:localhost:9999 pi@ROBOT_IP

VR usage:
  - Hold left grip to control left arm.
  - Hold right grip to control right arm.
  - Move controller relative to first gripped pose.
  - Trigger closes corresponding gripper.
  - Releasing trigger opens corresponding gripper.
  - Right joystick controls head pan/tilt.
"""

import argparse
import json
import socketserver
import threading
import time
import traceback

import test_skills as skills
from ainex_sdk import Board


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


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def safe_float(x, default=0.0):
    try:
        return float(x)
    except Exception:
        return default


class VRArmController:
    """
    Approximate position-based end-effector control.

    This is a safe relative controller, not full IK:
      - First grip frame becomes an anchor.
      - Controller motion relative to that anchor maps to shoulder/elbow servos.
      - Trigger closes gripper.
      - Trigger release opens gripper.
      - Right joystick maps to head pan/tilt.
    """

    def __init__(
        self,
        board,
        hz=20.0,
        grip_deadman=0.25,
        max_step=45,
        move_time=0.08,
        verbose=False,
    ):
        self.board = board
        self.hz = hz
        self.dt = 1.0 / hz
        self.grip_deadman = grip_deadman
        self.max_step = max_step
        self.move_time = move_time
        self.verbose = verbose

        self.enabled = False
        self.running = False

        self.packet_lock = threading.Lock()
        self.latest_packet = None
        self.latest_packet_time = 0.0

        self.anchor_pos = {
            "left": None,
            "right": None,
        }

        # OBSERVED gripper behavior from your robot:
        #   Left open  = 200
        #   Left close = 800
        #   Right open = 800
        #   Right close = 200
        self.current = {
            "left": {
                "sho_pitch": skills.L_SHO_PITCH,
                "sho_roll": skills.L_SHO_ROLL,
                "el_pitch": skills.L_EL_PITCH,
                "gripper": skills.L_GRIPPER,

                "sho_pitch_pos": 875,
                "sho_roll_pos": 500,
                "el_pitch_pos": 500,
                "gripper_pos": 200,
            },
            "right": {
                "sho_pitch": skills.R_SHO_PITCH,
                "sho_roll": skills.R_SHO_ROLL,
                "el_pitch": skills.R_EL_PITCH,
                "gripper": skills.R_GRIPPER,

                "sho_pitch_pos": 125,
                "sho_roll_pos": 500,
                "el_pitch_pos": 500,
                "gripper_pos": 800,
            },
        }

        self.ranges = {
            "left": {
                "sho_pitch": (450, 875),
                "sho_roll": (250, 850),
                "el_pitch": (150, 850),
                "gripper": (200, 800),
            },
            "right": {
                "sho_pitch": (125, 575),
                "sho_roll": (150, 750),
                "el_pitch": (200, 850),
                "gripper": (200, 800),
            },
        }

        self.neutral = {
            "left": {
                "sho_pitch": 875,
                "sho_roll": 500,
                "el_pitch": 500,
                "gripper": 200,
            },
            "right": {
                "sho_pitch": 125,
                "sho_roll": 500,
                "el_pitch": 500,
                "gripper": 800,
            },
        }

        self.gains = {
            "pitch_from_forward": 900.0,
            "pitch_from_up": 250.0,
            "roll_from_side": 850.0,
            "elbow_from_forward": 700.0,
        }

        # Right joystick head control.
        # From test_skills.py:
        #   HEAD_PAN 700 = left
        #   HEAD_PAN 300 = right
        #   HEAD_TILT 350 = up
        #   HEAD_TILT 650 = down
        self.head_enabled = True
        self.head_pan = 500
        self.head_tilt = 500
        self.head_deadzone = 0.18
        self.head_center_pan = 500
        self.head_center_tilt = 500

        self.head_pan_range = (300, 700)
        self.head_tilt_range = (300, 700)

        self.head_max_step = 12
        self.head_pan_gain = 180
        self.head_tilt_gain = 160

    def start(self, robot):
        self.running = True
        t = threading.Thread(target=self._loop, args=(robot,), daemon=True)
        t.start()

    def stop(self):
        self.running = False

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        if not self.enabled:
            self.anchor_pos = {"left": None, "right": None}

    def zero(self):
        self.anchor_pos = {
            "left": None,
            "right": None,
        }

    def update_packet(self, packet):
        with self.packet_lock:
            self.latest_packet = packet
            self.latest_packet_time = time.time()

    def status(self):
        age = time.time() - self.latest_packet_time if self.latest_packet_time else None
        return {
            "enabled": self.enabled,
            "head_enabled": self.head_enabled,
            "has_packet": self.latest_packet is not None,
            "packet_age_s": age,
            "anchors": self.anchor_pos,
            "current": self.current,
            "head": {
                "pan": self.head_pan,
                "tilt": self.head_tilt,
            },
        }

    def center_head(self):
        self.head_pan = 500
        self.head_tilt = 500
        self.board.bus_servo_set_position(
            0.25,
            [[skills.HEAD_PAN, 500], [skills.HEAD_TILT, 500]],
        )

    def open_grippers(self):
        """
        Open both grippers based on observed hardware behavior.

        Observed:
          left open  = 200
          right open = 800
        """
        self.current["left"]["gripper_pos"] = 200
        self.current["right"]["gripper_pos"] = 800

        self.board.bus_servo_set_position(
            0.25,
            [
                [skills.L_GRIPPER, 200],
                [skills.R_GRIPPER, 800],
            ],
        )

    def _loop(self, robot):
        print("[VR_ARM] control loop started")

        while self.running:
            start = time.time()

            try:
                if self.enabled:
                    self._update_once(robot)
            except Exception as e:
                print(f"[VR_ARM] loop error: {e}")
                traceback.print_exc()

            elapsed = time.time() - start
            time.sleep(max(0.0, self.dt - elapsed))

        print("[VR_ARM] control loop stopped")

    def _update_once(self, robot):
        with self.packet_lock:
            packet = self.latest_packet
            packet_time = self.latest_packet_time

        if packet is None:
            return

        if time.time() - packet_time > 0.5:
            return

        commands = []

        head_cmd = self._right_joystick_to_head(packet)
        if head_cmd:
            commands.extend(head_cmd)

        for hand in ("left", "right"):
            h = packet.get(hand)
            if not h:
                self.anchor_pos[hand] = None
                continue

            grip = safe_float(h.get("grip", 0.0))
            trigger = safe_float(h.get("trigger", 0.0))
            pos = h.get("pos")

            # Even if grip is not held, trigger should still control gripper.
            if grip < self.grip_deadman or not pos or len(pos) < 3:
                self.anchor_pos[hand] = None
                commands.append(self._trigger_to_gripper(hand, trigger))
                continue

            pos = [safe_float(v) for v in pos[:3]]

            if self.anchor_pos[hand] is None:
                self.anchor_pos[hand] = pos
                if self.verbose:
                    print(f"[VR_ARM] anchor {hand}: {pos}")

                # Still process gripper on anchor frame.
                commands.append(self._trigger_to_gripper(hand, trigger))
                continue

            anchor = self.anchor_pos[hand]

            dx = pos[0] - anchor[0]
            dy = pos[1] - anchor[1]
            dz = pos[2] - anchor[2]

            target = self._pose_delta_to_servo_targets(hand, dx, dy, dz, trigger)

            for joint_name, target_pos in target.items():
                servo_id = self.current[hand][joint_name]
                current_key = joint_name + "_pos"
                old_pos = self.current[hand][current_key]

                limited = self._rate_limit(old_pos, target_pos)
                self.current[hand][current_key] = limited
                commands.append([servo_id, int(limited)])

        if not commands:
            return

        if not robot.lock.acquire(blocking=False):
            return

        try:
            self.board.bus_servo_set_position(self.move_time, commands)
        finally:
            robot.lock.release()

    def _trigger_to_gripper(self, hand, trigger):
        """
        Gripper behavior based on observed hardware behavior:

          trigger released -> open
          trigger held     -> closed

        Observed:
          Left:
            open   = 200
            closed = 800

          Right:
            open   = 800
            closed = 200
        """
        trigger_pressed = trigger > 0.35

        if hand == "left":
            target = 800 if trigger_pressed else 200
        else:
            target = 200 if trigger_pressed else 800

        servo_id = self.current[hand]["gripper"]
        old_pos = self.current[hand]["gripper_pos"]
        limited = self._rate_limit(old_pos, target)
        self.current[hand]["gripper_pos"] = limited

        return [servo_id, int(limited)]

    def _pose_delta_to_servo_targets(self, hand, dx, dy, dz, trigger):
        n = self.neutral[hand]
        r = self.ranges[hand]
        g = self.gains

        forward = -dz

        if hand == "right":
            sho_pitch = (
                n["sho_pitch"]
                + g["pitch_from_forward"] * forward
                + g["pitch_from_up"] * dy
            )

            sho_roll = n["sho_roll"] - g["roll_from_side"] * dx
            el_pitch = n["el_pitch"] - g["elbow_from_forward"] * forward

            # Right observed:
            #   open   = 800
            #   closed = 200
            gripper = 200 if trigger > 0.35 else 800

        else:
            sho_pitch = (
                n["sho_pitch"]
                - g["pitch_from_forward"] * forward
                - g["pitch_from_up"] * dy
            )

            sho_roll = n["sho_roll"] - g["roll_from_side"] * dx
            el_pitch = n["el_pitch"] + g["elbow_from_forward"] * forward

            # Left observed:
            #   open   = 200
            #   closed = 800
            gripper = 800 if trigger > 0.35 else 200

        return {
            "sho_pitch": clamp(sho_pitch, *r["sho_pitch"]),
            "sho_roll": clamp(sho_roll, *r["sho_roll"]),
            "el_pitch": clamp(el_pitch, *r["el_pitch"]),
            "gripper": clamp(gripper, *r["gripper"]),
        }

    def _right_joystick_to_head(self, packet):
        if not self.head_enabled:
            return []

        right = packet.get("right")
        if not right:
            return []

        axes = right.get("axes") or []
        if not axes:
            return []

        if len(axes) >= 4:
            joy_x = safe_float(axes[2])
            joy_y = safe_float(axes[3])
        elif len(axes) >= 2:
            joy_x = safe_float(axes[0])
            joy_y = safe_float(axes[1])
        else:
            return []

        if abs(joy_x) < self.head_deadzone:
            joy_x = 0.0
        if abs(joy_y) < self.head_deadzone:
            joy_y = 0.0

        if joy_x == 0.0 and joy_y == 0.0:
            return []

        # Correct direction according to test_skills.py:
        #   HEAD_PAN 700 = left, 300 = right
        #   HEAD_TILT 350 = up, 650 = down
        #
        # Browser joystick convention is usually:
        #   joy_x > 0 = stick right
        #   joy_y < 0 = stick up
        target_pan = self.head_center_pan - joy_x * self.head_pan_gain
        target_tilt = self.head_center_tilt + joy_y * self.head_tilt_gain

        target_pan = clamp(target_pan, *self.head_pan_range)
        target_tilt = clamp(target_tilt, *self.head_tilt_range)

        self.head_pan = self._rate_limit_head(self.head_pan, target_pan)
        self.head_tilt = self._rate_limit_head(self.head_tilt, target_tilt)

        return [
            [skills.HEAD_PAN, int(self.head_pan)],
            [skills.HEAD_TILT, int(self.head_tilt)],
        ]

    def _rate_limit(self, old_pos, target_pos):
        delta = target_pos - old_pos

        if abs(delta) <= self.max_step:
            return target_pos

        return old_pos + self.max_step * (1 if delta > 0 else -1)

    def _rate_limit_head(self, old_pos, target_pos):
        delta = target_pos - old_pos

        if abs(delta) <= self.head_max_step:
            return target_pos

        return old_pos + self.head_max_step * (1 if delta > 0 else -1)


class RobotState:
    def __init__(self, board, token=None, vr_verbose=False):
        self.board = board
        self.token = token
        self.lock = threading.Lock()
        self.shutdown_requested = False

        self.vr = VRArmController(
            board=board,
            hz=20.0,
            grip_deadman=0.25,
            max_step=45,
            move_time=0.08,
            verbose=vr_verbose,
        )


def run_skill(robot, skill_name):
    skill_name = ALIASES.get(skill_name, skill_name)

    if skill_name not in skills.SKILLS:
        return f"ERROR: Unknown skill '{skill_name}'. Type 'list'.\n"

    name, func = skills.SKILLS[skill_name]

    was_vr_enabled = robot.vr.enabled
    robot.vr.set_enabled(False)

    if not robot.lock.acquire(blocking=False):
        return "BUSY: Robot is already running a skill/action/VR update.\n"

    try:
        print(f"[TELEOP] Running skill: {skill_name} - {name}")
        func(robot.board)
        return f"OK: finished {skill_name} - {name}\n"

    except Exception as e:
        traceback.print_exc()
        return f"ERROR while running {skill_name}: {e}\n"

    finally:
        robot.lock.release()
        robot.vr.set_enabled(was_vr_enabled)


def run_action(robot, action_name):
    if not action_name:
        return "ERROR: usage: action <action_name>\n"

    was_vr_enabled = robot.vr.enabled
    robot.vr.set_enabled(False)

    if not robot.lock.acquire(blocking=False):
        return "BUSY: Robot is already running a skill/action/VR update.\n"

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
        robot.vr.set_enabled(was_vr_enabled)


def help_text():
    return """
AINEX Advanced TCP Teleop Commands

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
  stop              disable VR and request current action stop
  quit              close this connection
  shutdown          stop teleop server

VR arm/head control:
  vr_on             enable VR arm/head control and open grippers
  vr_off            disable VR arm/head control and open grippers
  vr_zero           reset VR controller anchors
  vr_status         show VR control status
  vr {json}         update latest VR packet from Quest

Head:
  head_center       center head
  head_on           enable right joystick head control
  head_off          disable right joystick head control

Grippers:
  grippers_open     open both grippers

VR usage:
  Hold LEFT grip      -> control left arm
  Hold RIGHT grip     -> control right arm
  Move hand           -> move approximate end-effector
  Trigger held        -> close corresponding gripper
  Trigger released    -> open corresponding gripper
  Right joystick      -> head up/down/left/right
""".lstrip()


def handle_vr_command(robot, command):
    lower = command.lower().strip()

    if lower == "vr_on":
        robot.vr.set_enabled(True)
        robot.vr.open_grippers()
        return "OK: VR arm/head control enabled. Grippers opened.\n"

    if lower == "vr_off":
        robot.vr.set_enabled(False)
        robot.vr.open_grippers()
        return "OK: VR arm/head control disabled. Grippers opened.\n"

    if lower == "vr_zero":
        robot.vr.zero()
        return "OK: VR anchors reset. Hold grip again to re-anchor.\n"

    if lower == "vr_status":
        return json.dumps(robot.vr.status(), indent=2, default=str) + "\n"

    if lower == "head_center":
        robot.vr.center_head()
        return "OK: head centered.\n"

    if lower == "head_off":
        robot.vr.head_enabled = False
        return "OK: right joystick head control disabled.\n"

    if lower == "head_on":
        robot.vr.head_enabled = True
        return "OK: right joystick head control enabled.\n"

    if lower == "grippers_open":
        robot.vr.open_grippers()
        return "OK: both grippers opened.\n"

    if command.startswith("vr "):
        payload_text = command[3:].strip()

        try:
            packet = json.loads(payload_text)
        except Exception as e:
            return f"ERROR: bad VR JSON: {e}\n"

        robot.vr.update_packet(packet)
        return "OK: vr packet\n"

    return None


def handle_command(robot, raw_command):
    command = raw_command.strip()

    if not command:
        return ""

    if robot.token is not None:
        parts = command.split(maxsplit=1)
        if len(parts) == 0 or parts[0] != robot.token:
            return "ERROR: bad or missing token.\n"
        command = parts[1] if len(parts) > 1 else ""

    command = command.strip()
    lower = command.lower()

    vr_response = handle_vr_command(robot, command)
    if vr_response is not None:
        return vr_response

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
        robot.vr.set_enabled(False)
        robot.vr.open_grippers()
        if skills.action_player is not None:
            skills.action_player.stop()
            return "OK: VR disabled, grippers opened, and action stop requested.\n"
        return "OK: VR disabled and grippers opened. No action_player exists yet.\n"

    if lower.startswith("action "):
        action_name = command.split(maxsplit=1)[1].strip()
        return run_action(robot, action_name)

    return run_skill(robot, lower)


class TeleopHandler(socketserver.StreamRequestHandler):
    def handle(self):
        robot = self.server.robot

        peer = self.client_address
        print(f"[TELEOP] Client connected: {peer}")

        self.wfile.write(b"AINEX advanced teleop connected. Type 'help'.\n> ")
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
    parser = argparse.ArgumentParser(description="AINEX Advanced TCP Teleop Server")
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Use 127.0.0.1 with SSH tunnel; use 0.0.0.0 for direct LAN.",
    )
    parser.add_argument("--port", type=int, default=9999)
    parser.add_argument("--token", default=None)
    parser.add_argument("--vr-verbose", action="store_true")
    parser.add_argument("--no-walk-ready-on-exit", action="store_true")
    args = parser.parse_args()

    print("AINEX Advanced TCP Teleop Server")
    print("Initializing board...")

    try:
        board = Board()
        print("Board connected successfully.")
    except Exception as e:
        print(f"ERROR: could not connect to board: {e}")
        return

    skills.action_player = skills.ActionPlayer(board)

    robot = RobotState(
        board=board,
        token=args.token,
        vr_verbose=args.vr_verbose,
    )

    robot.vr.start(robot)

    server = ThreadedTCPServer((args.host, args.port), TeleopHandler)
    server.robot = robot

    print(f"Listening on {args.host}:{args.port}")
    print("VR commands available: vr_on, vr_off, vr_zero, vr_status")
    print("Head commands available: head_center, head_on, head_off")
    print("Gripper command available: grippers_open")
    print("Use SSH tunnel:")
    print(f"  ssh -L {args.port}:localhost:{args.port} pi@ROBOT_IP")

    try:
        server.serve_forever()

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        print("Shutting down...")
        robot.vr.set_enabled(False)
        robot.vr.stop()
        server.server_close()

        try:
            robot.vr.open_grippers()
        except Exception:
            pass

        if not args.no_walk_ready_on_exit:
            print("Returning to walk_ready position...")
            try:
                skills.skill_walk_ready(board)
            except Exception:
                pass

        print("Done.")


if __name__ == "__main__":
    main()