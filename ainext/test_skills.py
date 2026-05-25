#!/usr/bin/env python3
# encoding: utf-8
"""
AINEX Robot Skills Test Script
Tests various robot capabilities without using ROS.

Usage:
    python3 test_skills.py           # Run interactive menu
    python3 test_skills.py --all     # Run all tests sequentially
    python3 test_skills.py --skill wave  # Run specific skill
"""

import time
import sys
import os
import argparse
import sqlite3

# Set headless mode for OpenCV before importing cv2
# This prevents Qt display errors when running via SSH
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import cv2
import numpy as np
from ainex_sdk import Board

# =============================================================================
# SERVO IDs (from servo_controller.yaml)
# =============================================================================
# Left leg
L_ANK_ROLL = 1      # init: 500
L_ANK_PITCH = 3     # init: 500
L_KNEE = 5          # init: 240
L_HIP_PITCH = 7     # init: 500
L_HIP_ROLL = 9      # init: 500
L_HIP_YAW = 11      # init: 500

# Right leg
R_ANK_ROLL = 2      # init: 500
R_ANK_PITCH = 4     # init: 500
R_KNEE = 6          # init: 760
R_HIP_PITCH = 8     # init: 500
R_HIP_ROLL = 10     # init: 500
R_HIP_YAW = 12      # init: 500

# Left arm
L_SHO_PITCH = 13    # init: 875 (shoulder pitch)
L_SHO_ROLL = 15     # init: 500 (shoulder roll)
L_EL_PITCH = 17     # init: 500 (elbow pitch)
L_EL_YAW = 19       # init: 500 (elbow yaw)
L_GRIPPER = 21      # init: 500

# Right arm
R_SHO_PITCH = 14    # init: 125 (shoulder pitch)
R_SHO_ROLL = 16     # init: 500 (shoulder roll)
R_EL_PITCH = 18     # init: 500 (elbow pitch)
R_EL_YAW = 20       # init: 500 (elbow yaw)
R_GRIPPER = 22      # init: 500

# Head
HEAD_PAN = 23       # init: 500 (left/right)
HEAD_TILT = 24      # init: 500 (up/down)

# Initial positions dictionary
INIT_POSITIONS = {
    L_ANK_ROLL: 500, L_ANK_PITCH: 500, L_KNEE: 240, L_HIP_PITCH: 500, L_HIP_ROLL: 500, L_HIP_YAW: 500,
    R_ANK_ROLL: 500, R_ANK_PITCH: 500, R_KNEE: 760, R_HIP_PITCH: 500, R_HIP_ROLL: 500, R_HIP_YAW: 500,
    L_SHO_PITCH: 875, L_SHO_ROLL: 500, L_EL_PITCH: 500, L_EL_YAW: 500, L_GRIPPER: 500,
    R_SHO_PITCH: 125, R_SHO_ROLL: 500, R_EL_PITCH: 500, R_EL_YAW: 500, R_GRIPPER: 500,
    HEAD_PAN: 500, HEAD_TILT: 500
}

# =============================================================================
# ACTION PLAYER (for .d6a action files)
# =============================================================================

# Default path where action files are stored on the robot
ACTION_PATH = 'ainex_controller/ActionGroups'

class ActionPlayer:
    """
    Plays pre-recorded action sequences from .d6a files (SQLite databases).
    This is a non-ROS version of MotionManager.
    """
    
    def __init__(self, board, action_path=ACTION_PATH):
        self.board = board
        self.action_path = action_path
        self.running = False
        self.stop_requested = False
    
    def action_exists(self, action_name):
        """Check if an action file exists"""
        action_file = os.path.join(self.action_path, action_name + ".d6a")
        return os.path.exists(action_file)
    
    def list_actions(self):
        """List all available action files"""
        if not os.path.exists(self.action_path):
            return []
        actions = []
        for f in os.listdir(self.action_path):
            if f.endswith('.d6a'):
                actions.append(f[:-4])  # Remove .d6a extension
        return sorted(actions)
    
    def play_action(self, action_name):
        """
        Play an action sequence from a .d6a file.
        
        .d6a files are SQLite databases with an ActionGroup table.
        Each row contains: [index, duration_ms, servo1_pos, servo2_pos, ... servo24_pos]
        """
        action_file = os.path.join(self.action_path, action_name + ".d6a")
        
        if not os.path.exists(action_file):
            print(f"  Action file not found: {action_file}")
            print(f"  Make sure you're running this on the robot.")
            return False
        
        self.running = True
        self.stop_requested = False
        
        try:
            # Open SQLite database
            conn = sqlite3.connect(action_file)
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM ActionGroup")
            
            frame_count = 0
            while True:
                if self.stop_requested:
                    print("  Action stopped by user")
                    break
                
                row = cursor.fetchone()
                if row is None:
                    break
                
                frame_count += 1
                
                # row format: [index, duration_ms, servo1, servo2, ..., servo24]
                duration_ms = row[1]
                
                # Build servo position list: [[servo_id, position], ...]
                positions = []
                for servo_id in range(1, len(row) - 1):  # servo IDs 1-24
                    position = row[1 + servo_id]  # Skip index and duration
                    if position is not None:
                        positions.append([servo_id, position])
                
                # Send command to servos
                if positions:
                    self.board.bus_servo_set_position(duration_ms / 1000.0, positions)
                    time.sleep(duration_ms / 1000.0)
            
            cursor.close()
            conn.close()
            
            print(f"  Played {frame_count} frames")
            return True
            
        except Exception as e:
            print(f"  Error playing action: {e}")
            return False
        finally:
            self.running = False
    
    def stop(self):
        """Request to stop the current action"""
        self.stop_requested = True


# Global action player instance (initialized in main)
action_player = None


# =============================================================================
# WALKING HELPER (uses action files like kick does)
# =============================================================================

def walk_using_actions(board, action_name, count=1):
    """
    Use pre-recorded action files for walking.
    This is more reliable than manual servo control.
    """
    global action_player
    if action_player is None:
        print("  Action player not initialized")
        return False
    
    if not action_player.action_exists(action_name):
        print(f"  Action file '{action_name}.d6a' not found.")
        print(f"  Available actions: {action_player.list_actions()}")
        return False
    
    for i in range(count):
        print(f"  Playing '{action_name}' ({i+1}/{count})...")
        action_player.play_action(action_name)
    
    return True


# =============================================================================
# SKILL FUNCTIONS
# =============================================================================

def skill_wave(board, arm='right'):
    """Make the robot wave with specified arm"""
    print(f"Skill: Wave ({arm} arm)")
    
    duration = 0.5
    
    if arm == 'right':
        shoulder = R_SHO_PITCH
        elbow = R_EL_PITCH
        shoulder_init = 125
        shoulder_up = 400
    else:
        shoulder = L_SHO_PITCH
        elbow = L_EL_PITCH
        shoulder_init = 875
        shoulder_up = 600  # Lower value = up for left arm
    
    # Move to initial
    board.bus_servo_set_position(duration, [[shoulder, shoulder_init], [elbow, 500]])
    time.sleep(duration + 0.1)
    
    # Raise arm
    board.bus_servo_set_position(duration, [[shoulder, shoulder_up], [elbow, 700]])
    time.sleep(duration + 0.1)
    
    # Wave 3 times
    for i in range(3):
        board.bus_servo_set_position(0.25, [[elbow, 600]])
        time.sleep(0.3)
        board.bus_servo_set_position(0.25, [[elbow, 800]])
        time.sleep(0.3)
    
    # Return to init
    board.bus_servo_set_position(duration, [[shoulder, shoulder_init], [elbow, 500]])
    time.sleep(duration + 0.1)
    
    print("  Wave complete!")


def skill_head_look_around(board):
    """Make the robot look around (head movements)"""
    print("Skill: Head Look Around")
    
    duration = 0.4
    
    # Look center
    board.bus_servo_set_position(duration, [[HEAD_PAN, 500], [HEAD_TILT, 500]])
    time.sleep(duration + 0.1)
    
    # Look left
    print("  Looking left...")
    board.bus_servo_set_position(duration, [[HEAD_PAN, 700]])
    time.sleep(duration + 0.2)
    
    # Look right
    print("  Looking right...")
    board.bus_servo_set_position(duration, [[HEAD_PAN, 300]])
    time.sleep(duration + 0.2)
    
    # Look up
    print("  Looking up...")
    board.bus_servo_set_position(duration, [[HEAD_PAN, 500], [HEAD_TILT, 350]])
    time.sleep(duration + 0.2)
    
    # Look down
    print("  Looking down...")
    board.bus_servo_set_position(duration, [[HEAD_TILT, 650]])
    time.sleep(duration + 0.2)
    
    # Return to center
    board.bus_servo_set_position(duration, [[HEAD_PAN, 500], [HEAD_TILT, 500]])
    time.sleep(duration + 0.1)
    
    print("  Head movements complete!")


def skill_nod_yes(board):
    """Make the robot nod (say yes)"""
    print("Skill: Nod Yes")
    
    duration = 0.2
    
    for _ in range(3):
        board.bus_servo_set_position(duration, [[HEAD_TILT, 600]])
        time.sleep(duration + 0.05)
        board.bus_servo_set_position(duration, [[HEAD_TILT, 400]])
        time.sleep(duration + 0.05)
    
    board.bus_servo_set_position(duration, [[HEAD_TILT, 500]])
    time.sleep(duration)
    
    print("  Nod complete!")


def skill_shake_no(board):
    """Make the robot shake head (say no)"""
    print("Skill: Shake No")
    
    duration = 0.2
    
    for _ in range(3):
        board.bus_servo_set_position(duration, [[HEAD_PAN, 600]])
        time.sleep(duration + 0.05)
        board.bus_servo_set_position(duration, [[HEAD_PAN, 400]])
        time.sleep(duration + 0.05)
    
    board.bus_servo_set_position(duration, [[HEAD_PAN, 500]])
    time.sleep(duration)
    
    print("  Shake complete!")


def skill_bow(board):
    """Make the robot bow politely"""
    print("Skill: Bow")
    
    duration = 0.6
    
    # Bow down
    board.bus_servo_set_position(duration, [[HEAD_TILT, 750]])
    time.sleep(duration + 0.3)
    
    # Rise up
    board.bus_servo_set_position(duration, [[HEAD_TILT, 500]])
    time.sleep(duration + 0.1)
    
    print("  Bow complete!")


def skill_arms_up(board):
    """Raise both arms up (celebration pose)"""
    print("Skill: Arms Up (Celebration)")
    
    duration = 0.6
    
    # Raise both arms
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 500],  # Right arm up
        [L_SHO_PITCH, 500],  # Left arm up
        [R_EL_PITCH, 300],
        [L_EL_PITCH, 700]
    ])
    time.sleep(duration + 0.5)
    
    # Return to init
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 125],
        [L_SHO_PITCH, 875],
        [R_EL_PITCH, 500],
        [L_EL_PITCH, 500]
    ])
    time.sleep(duration + 0.1)
    
    print("  Arms up complete!")


def skill_arms_cross(board):
    """Cross arms in front (thinking pose)"""
    print("Skill: Arms Cross")
    
    duration = 0.5
    
    # Move arms to crossed position
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 400],
        [L_SHO_PITCH, 600],
        [R_SHO_ROLL, 300],
        [L_SHO_ROLL, 700],
        [R_EL_PITCH, 800],
        [L_EL_PITCH, 200]
    ])
    time.sleep(duration + 0.5)
    
    # Return to init
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 125],
        [L_SHO_PITCH, 875],
        [R_SHO_ROLL, 500],
        [L_SHO_ROLL, 500],
        [R_EL_PITCH, 500],
        [L_EL_PITCH, 500]
    ])
    time.sleep(duration + 0.1)
    
    print("  Arms cross complete!")


def skill_grippers(board):
    """Test both grippers (open and close)"""
    print("Skill: Gripper Test")
    
    duration = 0.4
    
    # Close both grippers
    print("  Closing grippers...")
    board.bus_servo_set_position(duration, [
        [R_GRIPPER, 800],
        [L_GRIPPER, 200]
    ])
    time.sleep(duration + 0.3)
    
    # Open both grippers
    print("  Opening grippers...")
    board.bus_servo_set_position(duration, [
        [R_GRIPPER, 200],
        [L_GRIPPER, 800]
    ])
    time.sleep(duration + 0.3)
    
    # Return to init
    board.bus_servo_set_position(duration, [
        [R_GRIPPER, 500],
        [L_GRIPPER, 500]
    ])
    time.sleep(duration + 0.1)
    
    print("  Gripper test complete!")


def skill_dance_simple(board):
    """Simple dance routine with arm and head movements"""
    print("Skill: Simple Dance")
    
    duration = 0.3
    
    for _ in range(2):
        # Move right arm up, head left
        board.bus_servo_set_position(duration, [
            [R_SHO_PITCH, 400],
            [HEAD_PAN, 650]
        ])
        time.sleep(duration + 0.1)
        
        # Move left arm up, head right
        board.bus_servo_set_position(duration, [
            [R_SHO_PITCH, 125],
            [L_SHO_PITCH, 600],
            [HEAD_PAN, 350]
        ])
        time.sleep(duration + 0.1)
        
        # Both arms up
        board.bus_servo_set_position(duration, [
            [L_SHO_PITCH, 875],
            [R_SHO_PITCH, 400],
            [L_SHO_PITCH, 600],
            [HEAD_PAN, 500]
        ])
        time.sleep(duration + 0.1)
    
    # Return to init
    board.bus_servo_set_position(0.5, [
        [R_SHO_PITCH, 125],
        [L_SHO_PITCH, 875],
        [HEAD_PAN, 500]
    ])
    time.sleep(0.6)
    
    print("  Dance complete!")


def skill_stretch(board):
    """Stretching routine"""
    print("Skill: Stretch")
    
    duration = 0.6
    
    # Stretch right arm out
    print("  Stretching right arm...")
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 500],
        [R_SHO_ROLL, 200],
        [R_EL_PITCH, 200]
    ])
    time.sleep(duration + 0.3)
    
    # Return and stretch left
    print("  Stretching left arm...")
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 125],
        [R_SHO_ROLL, 500],
        [R_EL_PITCH, 500],
        [L_SHO_PITCH, 500],
        [L_SHO_ROLL, 800],
        [L_EL_PITCH, 800]
    ])
    time.sleep(duration + 0.3)
    
    # Return to init
    board.bus_servo_set_position(duration, [
        [L_SHO_PITCH, 875],
        [L_SHO_ROLL, 500],
        [L_EL_PITCH, 500]
    ])
    time.sleep(duration + 0.1)
    
    print("  Stretch complete!")


def skill_rgb_rainbow(board):
    """Cycle through rainbow colors on RGB LED"""
    print("Skill: RGB Rainbow")
    
    colors = [
        (255, 0, 0),    # Red
        (255, 127, 0),  # Orange
        (255, 255, 0),  # Yellow
        (0, 255, 0),    # Green
        (0, 0, 255),    # Blue
        (75, 0, 130),   # Indigo
        (148, 0, 211),  # Violet
    ]
    
    for name, (r, g, b) in zip(['Red', 'Orange', 'Yellow', 'Green', 'Blue', 'Indigo', 'Violet'], colors):
        print(f"  {name}...")
        board.set_rgb([[1, r, g, b]])
        time.sleep(0.4)
    
    # Turn off
    board.set_rgb([[1, 0, 0, 0]])
    
    print("  RGB rainbow complete!")


def skill_rgb_blink(board, color=(0, 255, 0), count=5):
    """Blink RGB LED"""
    print(f"Skill: RGB Blink ({count}x)")
    
    r, g, b = color
    
    for i in range(count):
        board.set_rgb([[1, r, g, b]])
        time.sleep(0.2)
        board.set_rgb([[1, 0, 0, 0]])
        time.sleep(0.2)
    
    print("  RGB blink complete!")


def skill_buzzer_beep(board, count=3):
    """Make buzzer beeps"""
    print(f"Skill: Buzzer Beep ({count}x)")
    
    for i in range(count):
        board.set_buzzer(1500 + i * 200, 0.1, 0.1, 1)
        time.sleep(0.3)
    
    print("  Buzzer beep complete!")


def skill_buzzer_melody(board):
    """Play a simple melody"""
    print("Skill: Buzzer Melody")
    
    # Simple melody notes (approximate frequencies)
    notes = [
        (523, 0.2),  # C5
        (587, 0.2),  # D5
        (659, 0.2),  # E5
        (523, 0.2),  # C5
        (659, 0.3),  # E5
        (659, 0.3),  # E5
        (587, 0.2),  # D5
        (587, 0.2),  # D5
        (587, 0.3),  # D5
        (523, 0.2),  # C5
        (784, 0.2),  # G5
        (659, 0.4),  # E5
    ]
    
    for freq, dur in notes:
        board.set_buzzer(freq, dur, 0.05, 1)
        time.sleep(dur + 0.05)
    
    print("  Melody complete!")


def skill_read_imu(board):
    """Read and display IMU sensor data"""
    print("Skill: Read IMU")
    
    board.enable_reception()
    time.sleep(0.1)
    
    readings = 0
    print("  Reading IMU data for 2 seconds...")
    print("  Format: ax, ay, az, gx, gy, gz [, mx, my, mz]")
    
    start_time = time.time()
    while time.time() - start_time < 2.0:
        data = board.get_imu()
        if data is not None:
            readings += 1
            if readings % 20 == 0:  # Print every 20th reading
                formatted = ", ".join([f"{v:7.3f}" for v in data])
                print(f"  [{formatted}]")
        time.sleep(0.01)
    
    print(f"  IMU test complete! ({readings} readings)")


def skill_read_battery(board):
    """Read battery voltage"""
    print("Skill: Read Battery")
    
    board.enable_reception()
    time.sleep(0.2)
    
    voltage = board.get_battery()
    if voltage is not None:
        print(f"  Battery voltage: {voltage}mV ({voltage/1000:.2f}V)")
    else:
        print("  Could not read battery voltage")
    
    print("  Battery check complete!")


def skill_reset_pose(board):
    """Reset all servos to initial positions"""
    print("Skill: Reset to Initial Pose")
    
    duration = 1.0
    
    # Build list of positions
    positions = [[servo_id, pos] for servo_id, pos in INIT_POSITIONS.items()]
    
    board.bus_servo_set_position(duration, positions)
    time.sleep(duration + 0.2)
    
    print("  Reset complete!")


def skill_robot_standup(board):
    """Return robot to standing position (upper body only - safe)"""
    print("Skill: Standing Pose (Upper Body)")
    
    duration = 0.8
    
    # Set upper body to standing pose
    upper_body = [
        [L_SHO_PITCH, 875], [L_SHO_ROLL, 500], [L_EL_PITCH, 500], [L_EL_YAW, 500], [L_GRIPPER, 500],
        [R_SHO_PITCH, 125], [R_SHO_ROLL, 500], [R_EL_PITCH, 500], [R_EL_YAW, 500], [R_GRIPPER, 500],
        [HEAD_PAN, 500], [HEAD_TILT, 500]
    ]
    
    board.bus_servo_set_position(duration, upper_body)
    time.sleep(duration + 0.2)
    
    print("  Standing pose complete!")


def skill_flex_arms(board):
    """Flex arms like showing muscles"""
    print("Skill: Flex Arms")
    
    duration = 0.5
    
    # Flex pose - arms bent
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 400],
        [L_SHO_PITCH, 600],
        [R_SHO_ROLL, 300],
        [L_SHO_ROLL, 700],
        [R_EL_PITCH, 900],
        [L_EL_PITCH, 100]
    ])
    time.sleep(duration + 0.5)
    
    # Pump once
    for _ in range(2):
        board.bus_servo_set_position(0.2, [
            [R_EL_PITCH, 800],
            [L_EL_PITCH, 200]
        ])
        time.sleep(0.25)
        board.bus_servo_set_position(0.2, [
            [R_EL_PITCH, 900],
            [L_EL_PITCH, 100]
        ])
        time.sleep(0.25)
    
    # Return to init
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 125],
        [L_SHO_PITCH, 875],
        [R_SHO_ROLL, 500],
        [L_SHO_ROLL, 500],
        [R_EL_PITCH, 500],
        [L_EL_PITCH, 500]
    ])
    time.sleep(duration + 0.1)
    
    print("  Flex complete!")


def skill_point(board, direction='forward'):
    """Point in a direction"""
    print(f"Skill: Point {direction}")
    
    duration = 0.5
    
    if direction == 'forward':
        board.bus_servo_set_position(duration, [
            [R_SHO_PITCH, 500],
            [R_SHO_ROLL, 500],
            [R_EL_PITCH, 200]
        ])
    elif direction == 'left':
        board.bus_servo_set_position(duration, [
            [L_SHO_PITCH, 500],
            [L_SHO_ROLL, 850],
            [L_EL_PITCH, 800],
            [HEAD_PAN, 700]
        ])
    elif direction == 'right':
        board.bus_servo_set_position(duration, [
            [R_SHO_PITCH, 500],
            [R_SHO_ROLL, 150],
            [R_EL_PITCH, 200],
            [HEAD_PAN, 300]
        ])
    
    time.sleep(duration + 0.5)
    
    # Return
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 125],
        [L_SHO_PITCH, 875],
        [R_SHO_ROLL, 500],
        [L_SHO_ROLL, 500],
        [R_EL_PITCH, 500],
        [L_EL_PITCH, 500],
        [HEAD_PAN, 500]
    ])
    time.sleep(duration + 0.1)
    
    print(f"  Point {direction} complete!")


# =============================================================================
# SKILL REGISTRY
# =============================================================================

# =============================================================================
# ACTION-BASED SKILLS (require .d6a files on robot)
# =============================================================================

def skill_kick_left(board):
    """Kick ball with left foot (uses pre-recorded action)"""
    print("Skill: Kick Left")
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    if not action_player.action_exists('left_shot'):
        print("  Action file 'left_shot.d6a' not found.")
        print("  This skill requires running on the robot with action files.")
        return
    
    action_player.play_action('left_shot')
    print("  Kick left complete!")


def skill_kick_right(board):
    """Kick ball with right foot (uses pre-recorded action)"""
    print("Skill: Kick Right")
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    if not action_player.action_exists('right_shot'):
        print("  Action file 'right_shot.d6a' not found.")
        print("  This skill requires running on the robot with action files.")
        return
    
    action_player.play_action('right_shot')
    print("  Kick right complete!")


def skill_getup_front(board):
    """Get up from lying face-down (uses pre-recorded action)"""
    print("Skill: Get Up (from front/face-down)")
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    if not action_player.action_exists('lie_to_stand'):
        print("  Action file 'lie_to_stand.d6a' not found.")
        print("  This skill requires running on the robot with action files.")
        return
    
    action_player.play_action('lie_to_stand')
    print("  Get up (front) complete!")


def skill_getup_back(board):
    """Get up from lying face-up (uses pre-recorded action)"""
    print("Skill: Get Up (from back/face-up)")
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    if not action_player.action_exists('recline_to_stand'):
        print("  Action file 'recline_to_stand.d6a' not found.")
        print("  This skill requires running on the robot with action files.")
        return
    
    action_player.play_action('recline_to_stand')
    print("  Get up (back) complete!")


def skill_stand_action(board):
    """Go to standing pose (uses pre-recorded action)"""
    print("Skill: Stand (action file)")
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    if not action_player.action_exists('stand'):
        print("  Action file 'stand.d6a' not found.")
        print("  This skill requires running on the robot with action files.")
        return
    
    action_player.play_action('stand')
    print("  Stand action complete!")


def skill_walk_ready(board):
    """Go to walk-ready pose (uses pre-recorded action)"""
    print("Skill: Walk Ready Pose")
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    if not action_player.action_exists('walk_ready'):
        print("  Action file 'walk_ready.d6a' not found.")
        print("  This skill requires running on the robot with action files.")
        return
    
    action_player.play_action('walk_ready')
    print("  Walk ready pose complete!")


def skill_list_actions(board):
    """List all available action files"""
    print("Skill: List Available Actions")
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    actions = action_player.list_actions()
    if actions:
        print(f"  Found {len(actions)} action files:")
        for i, action in enumerate(actions, 1):
            print(f"    {i:2}. {action}")
    else:
        print(f"  No action files found in {ACTION_PATH}")
        print("  This skill requires running on the robot.")


def skill_play_action(board, action_name=None):
    """Play a specific action by name"""
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    if action_name is None:
        # Interactive mode - ask for action name
        actions = action_player.list_actions()
        if not actions:
            print("  No action files available.")
            return
        
        print("  Available actions:")
        for i, action in enumerate(actions, 1):
            print(f"    {i:2}. {action}")
        
        try:
            choice = input("  Enter action number or name: ").strip()
            if choice.isdigit():
                idx = int(choice) - 1
                if 0 <= idx < len(actions):
                    action_name = actions[idx]
                else:
                    print("  Invalid number!")
                    return
            else:
                action_name = choice
        except KeyboardInterrupt:
            print("\n  Cancelled.")
            return
    
    print(f"Skill: Play Action '{action_name}'")
    if not action_player.action_exists(action_name):
        print(f"  Action '{action_name}' not found.")
        return
    
    action_player.play_action(action_name)
    print(f"  Action '{action_name}' complete!")


def skill_pushup(board):
    """Do a pushup motion (manual servo sequence)"""
    print("Skill: Pushup")
    
    duration = 0.8
    
    # This is a simplified pushup using upper body servos
    # Real pushup would need the robot to be on the ground
    
    print("  Going to pushup position...")
    # Arms forward, like preparing for pushup
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 500],
        [L_SHO_PITCH, 500],
        [R_EL_PITCH, 200],
        [L_EL_PITCH, 800],
        [HEAD_TILT, 650]  # Look down
    ])
    time.sleep(duration + 0.2)
    
    # Do 3 pushups
    for i in range(3):
        print(f"  Pushup {i+1}/3...")
        # Down
        board.bus_servo_set_position(0.5, [
            [R_EL_PITCH, 400],
            [L_EL_PITCH, 600]
        ])
        time.sleep(0.6)
        # Up
        board.bus_servo_set_position(0.5, [
            [R_EL_PITCH, 200],
            [L_EL_PITCH, 800]
        ])
        time.sleep(0.6)
    
    # Return to standing
    print("  Returning to stand...")
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 125],
        [L_SHO_PITCH, 875],
        [R_EL_PITCH, 500],
        [L_EL_PITCH, 500],
        [HEAD_TILT, 500]
    ])
    time.sleep(duration + 0.1)
    
    print("  Pushup complete!")


def skill_balance_kick(board):
    """Balance on one leg and kick (simplified, upper body + head motion)"""
    print("Skill: Balance and Kick")
    
    # NOTE: Actually balancing on one leg requires careful IMU feedback
    # This is a simplified demonstration using arm/upper body movements
    # to simulate the motion. A real kick uses the action files.
    
    duration = 0.6
    
    print("  Preparing to kick...")
    # Arms out for balance
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 350],
        [L_SHO_PITCH, 650],
        [R_SHO_ROLL, 200],
        [L_SHO_ROLL, 800]
    ])
    time.sleep(duration + 0.2)
    
    print("  Winding up...")
    # Look at target
    board.bus_servo_set_position(0.3, [
        [HEAD_TILT, 600],  # Look down
        [HEAD_PAN, 400]    # Slight turn
    ])
    time.sleep(0.4)
    
    print("  KICK!")
    # Quick arm motion to simulate kick momentum
    board.bus_servo_set_position(0.2, [
        [R_SHO_PITCH, 500],
        [HEAD_PAN, 500]
    ])
    time.sleep(0.3)
    
    print("  Recovery...")
    # Return to balanced pose
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 125],
        [L_SHO_PITCH, 875],
        [R_SHO_ROLL, 500],
        [L_SHO_ROLL, 500],
        [HEAD_TILT, 500],
        [HEAD_PAN, 500]
    ])
    time.sleep(duration + 0.1)
    
    print("  Balance kick demo complete!")
    print("  (For real kicking, use 'kick_left' or 'kick_right' with action files)")


def skill_lie_down_front(board):
    """Lie down face-down (simplified - upper body goes forward)"""
    print("Skill: Lie Down (Front)")
    print("  WARNING: Robot may fall. Ensure it's on a soft surface!")
    
    duration = 0.8
    
    # Slowly go to lying position - arms forward
    print("  Going down...")
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 500],  # Arms forward
        [L_SHO_PITCH, 500],
        [R_EL_PITCH, 200],   # Arms straight out
        [L_EL_PITCH, 800],
        [HEAD_TILT, 700]     # Head tucked
    ])
    time.sleep(duration + 0.3)
    
    print("  Lying face-down pose (upper body demo)")
    time.sleep(1.0)
    
    print("  Use 'getup_front' action to get up from this position")


def skill_lie_down_back(board):
    """Lie down face-up (simplified - upper body goes backward)"""
    print("Skill: Lie Down (Back)")
    print("  WARNING: Robot may fall. Ensure it's on a soft surface!")
    
    duration = 0.8
    
    # Slowly go to lying position - arms back
    print("  Going down...")
    board.bus_servo_set_position(duration, [
        [R_SHO_PITCH, 200],  # Arms back
        [L_SHO_PITCH, 800],
        [R_EL_PITCH, 500],
        [L_EL_PITCH, 500],
        [HEAD_TILT, 300]     # Head back
    ])
    time.sleep(duration + 0.3)
    
    print("  Lying face-up pose (upper body demo)")
    time.sleep(1.0)
    
    print("  Use 'getup_back' action to get up from this position")


def skill_walk_forward(board, steps=4):
    """
    Walk forward using pre-recorded action files.
    This uses the same reliable action system as kicks.
    """
    print("Skill: Walk Forward")
    print("  Looking for walking action files...")
    
    # Try various possible action names for walking forward
    possible_actions = ['go_forward', 'forward', 'walk_forward', 'walk', 'step_forward']
    
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    # Start in walk_ready position
    if action_player.action_exists('walk_ready'):
        print("  Going to walk_ready position...")
        action_player.play_action('walk_ready')
    
    # List available actions to help user
    available = action_player.list_actions()
    print(f"  Available actions: {available}")
    
    # Try to find a walking action
    walk_action = None
    for action in possible_actions:
        if action_player.action_exists(action):
            walk_action = action
            break
    
    if walk_action:
        print(f"  Using action: {walk_action}")
        for i in range(steps):
            print(f"  Step {i+1}/{steps}...")
            action_player.play_action(walk_action)
        # Return to walk_ready position
        if action_player.action_exists('walk_ready'):
            print("  Returning to walk_ready position...")
            action_player.play_action('walk_ready')
        print("  Walk forward complete!")
    else:
        print("  No walking action found.")
        print("  Try 'list_actions' to see what's available, or use 'play_action' to test one.")


def skill_walk_turn_left(board):
    """Turn left using pre-recorded action files"""
    print("Skill: Turn Left")
    
    possible_actions = ['turn_left', 'left_turn', 'rotate_left']
    
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    # Start in walk_ready position
    if action_player.action_exists('walk_ready'):
        print("  Going to walk_ready position...")
        action_player.play_action('walk_ready')
    
    for action in possible_actions:
        if action_player.action_exists(action):
            print(f"  Using action: {action}")
            for i in range(8):
                print(f"  Turn {i+1}/8...")
                action_player.play_action(action)
            # Return to walk_ready position
            if action_player.action_exists('walk_ready'):
                print("  Returning to walk_ready position...")
                action_player.play_action('walk_ready')
            print("  Turn left complete!")
            return
    
    print("  No turn left action found. Available:", action_player.list_actions())


def skill_walk_turn_right(board):
    """Turn right using pre-recorded action files"""
    print("Skill: Turn Right")
    
    possible_actions = ['turn_right', 'right_turn', 'rotate_right']
    
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    # Start in walk_ready position
    if action_player.action_exists('walk_ready'):
        print("  Going to walk_ready position...")
        action_player.play_action('walk_ready')
    
    for action in possible_actions:
        if action_player.action_exists(action):
            print(f"  Using action: {action}")
            for i in range(8):
                print(f"  Turn {i+1}/8...")
                action_player.play_action(action)
            # Return to walk_ready position
            if action_player.action_exists('walk_ready'):
                print("  Returning to walk_ready position...")
                action_player.play_action('walk_ready')
            print("  Turn right complete!")
            return
    
    print("  No turn right action found. Available:", action_player.list_actions())


def skill_walk_backward(board, steps=4):
    """Walk backward using pre-recorded action files"""
    print("Skill: Walk Backward")
    
    possible_actions = ['go_backward', 'backward', 'walk_backward', 'back', 'step_backward']
    
    global action_player
    if action_player is None:
        action_player = ActionPlayer(board)
    
    # Start in walk_ready position
    if action_player.action_exists('walk_ready'):
        print("  Going to walk_ready position...")
        action_player.play_action('walk_ready')
    
    for action in possible_actions:
        if action_player.action_exists(action):
            print(f"  Using action: {action}")
            for i in range(steps):
                print(f"  Step {i+1}/{steps}...")
                action_player.play_action(action)
            # Return to walk_ready position
            if action_player.action_exists('walk_ready'):
                print("  Returning to walk_ready position...")
                action_player.play_action('walk_ready')
            print("  Walk backward complete!")
            return
    
    print("  No backward action found. Available:", action_player.list_actions())


# =============================================================================
# VISION + MANIPULATION SKILLS
# =============================================================================

def detect_red_object(frame):
    """
    Detect red objects in the frame using HSV color space.
    Returns (found, center_x, center_y, area) of the largest red object.
    """
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Red has two ranges in HSV (wraps around 0/180)
    # Lower red range: 0-10
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    # Upper red range: 160-180
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # Create masks for both red ranges
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Clean up mask with morphology
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return False, 0, 0, 0
    
    # Find largest contour
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    
    # Minimum area threshold
    if area < 500:
        return False, 0, 0, 0
    
    # Get center using moments
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return False, 0, 0, 0
    
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    
    return True, cx, cy, area


def skill_grab_bin(board):
    """
    Look for a red bin/bucket and grab it using both hands.
    This combines vision (camera) with the 'move_up' action.
    Best for larger objects like bins (like autonomous_transport does).
    """
    print("Skill: Grab Bin (two-handed)")
    
    global action_player
    
    # Create debug directory
    debug_dir = "./debug"
    os.makedirs(debug_dir, exist_ok=True)
    
    # Camera options - prioritize common USB camera paths
    camera_options = [
        "/dev/video0",      # Most common USB camera location
        "/dev/video1",
        "/dev/usb_cam",
        0, 1, 2
    ]
    
    cap = None
    for cam in camera_options:
        if isinstance(cam, str) and not os.path.exists(cam):
            continue
        print(f"  Trying camera: {cam}...")
        test_cap = cv2.VideoCapture(cam)
        if test_cap.isOpened():
            ret, frame = test_cap.read()
            if ret and frame is not None:
                cap = test_cap
                print(f"  Camera {cam} opened successfully!")
                break
            test_cap.release()
    
    if cap is None:
        print("  ERROR: Could not open any camera!")
        print("  Available video devices:")
        for f in os.listdir("/dev"):
            if f.startswith("video"):
                print(f"    /dev/{f}")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("  Camera opened")
    
    try:
        # Step 1: Look DOWN to see the floor
        # HEAD_TILT ~280-330 = looking down (per autonomous_transport code)
        # NOT 750 which would make it look UP!
        print("  Step 1: Looking down at floor...")
        board.bus_servo_set_position(0.5, [[HEAD_TILT, 300], [HEAD_PAN, 500]])
        time.sleep(0.6)
        
        # Step 2: Prepare arms - move them back so they don't block view
        print("  Step 2: Moving arms back...")
        board.bus_servo_set_position(0.4, [
            [R_SHO_PITCH, 125],  # Right arm back (init position)
            [L_SHO_PITCH, 875],  # Left arm back (init position)
            [R_GRIPPER, 200],    # Open right gripper
        ])
        time.sleep(0.5)
        
        # Step 3: Search for red object
        print("  Step 3: Searching for red cube...")
        found = False
        center_x = 0
        center_y = 0
        frame_width = 640
        frame_height = 480
        last_frame = None
        
        for attempt in range(30):  # Try for ~3 seconds
            ret, frame = cap.read()
            if not ret:
                continue
            
            last_frame = frame
            found, cx, cy, area = detect_red_object(frame)
            
            if found:
                center_x = cx
                center_y = cy
                frame_width = frame.shape[1]
                frame_height = frame.shape[0]
                print(f"  Found red object at ({cx}, {cy}), area={area}")
                # Save debug image with detection marked
                debug_frame = frame.copy()
                cv2.circle(debug_frame, (cx, cy), 10, (0, 255, 0), 2)
                cv2.putText(debug_frame, f"RED ({area})", (cx-40, cy-20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # Draw crosshairs at center
                cv2.line(debug_frame, (frame_width//2 - 20, frame_height//2), 
                        (frame_width//2 + 20, frame_height//2), (255, 0, 0), 1)
                cv2.line(debug_frame, (frame_width//2, frame_height//2 - 20), 
                        (frame_width//2, frame_height//2 + 20), (255, 0, 0), 1)
                debug_path = os.path.join(debug_dir, "grab_detected.jpg")
                cv2.imwrite(debug_path, debug_frame)
                print(f"  Debug image saved to {debug_path}")
                break
            
            time.sleep(0.1)
        
        if not found:
            print("  No red cube detected!")
            print("  Make sure there's a red cube visible in front of the robot.")
            # Save debug image so user can see what camera saw
            if last_frame is not None:
                debug_path = os.path.join(debug_dir, "grab_notfound.jpg")
                cv2.imwrite(debug_path, last_frame)
                print(f"  Debug image saved to {debug_path}")
            # Return to normal pose
            board.bus_servo_set_position(0.5, [[HEAD_TILT, 500], [R_GRIPPER, 500]])
            time.sleep(0.6)
            cap.release()
            return
        
        # Step 4: Adjust head to center on object
        # Use head pan to center horizontally, tilt to center vertically
        print("  Step 4: Centering on object...")
        frame_center_x = frame_width // 2
        frame_center_y = frame_height // 2
        
        # Pan: if object is to the right (positive offset), decrease pan
        offset_x = center_x - frame_center_x
        pan_adjustment = int(-offset_x * 0.3)
        new_pan = max(300, min(700, 500 + pan_adjustment))
        
        # Tilt: if object is lower (positive offset from center), look more down
        offset_y = center_y - frame_center_y
        tilt_adjustment = int(-offset_y * 0.2)  # Smaller factor for tilt
        new_tilt = max(250, min(400, 300 + tilt_adjustment))  # Stay in looking-down range
        
        print(f"  Adjusting head: pan={new_pan}, tilt={new_tilt}")
        board.bus_servo_set_position(0.3, [[HEAD_PAN, new_pan], [HEAD_TILT, new_tilt]])
        time.sleep(0.4)
        
        # Save another debug frame after centering
        ret, frame = cap.read()
        if ret:
            debug_path = os.path.join(debug_dir, "grab_centered.jpg")
            cv2.imwrite(debug_path, frame)
        
        # Step 5-8: Use the pre-recorded 'move_up' action if available
        # This is how autonomous_transport does it - much more reliable!
        if action_player is None:
            action_player = ActionPlayer(board)
        
        if action_player.action_exists('move_up'):
            print("  Step 5-8: Using 'move_up' action to grab...")
            # Return head to forward position before action
            board.bus_servo_set_position(0.3, [[HEAD_PAN, 500], [HEAD_TILT, 500]])
            time.sleep(0.4)
            
            # Play the pick-up action
            action_player.play_action('move_up')
            
            # Confirm grab with beep
            board.set_buzzer(1500, 0.1, 0.05, 1)
            time.sleep(0.2)
        else:
            # Fallback: Manual arm movements (less reliable)
            print("  'move_up' action not found, using manual arm control...")
            print("  Step 5: Moving to grab position...")
            
            # Return head to neutral first
            board.bus_servo_set_position(0.3, [[HEAD_PAN, 500], [HEAD_TILT, 500]])
            time.sleep(0.4)
            
            # The manual approach - reach down and grab
            # First extend arm forward at chest level
            board.bus_servo_set_position(0.6, [
                [R_SHO_PITCH, 500],   # Shoulder forward (500 = horizontal)
                [R_SHO_ROLL, 500],    # Centered
                [R_EL_PITCH, 300],    # Elbow slightly bent
                [R_GRIPPER, 200],     # Keep gripper open
            ])
            time.sleep(0.7)
            
            print("  Step 6: Lowering to floor level...")
            # This is tricky without proper IK - robot may need to squat
            # For now, try to reach lower by adjusting shoulder and elbow
            board.bus_servo_set_position(0.6, [
                [R_SHO_PITCH, 650],   # Shoulder more forward/down
                [R_EL_PITCH, 200],    # Straighten elbow more
            ])
            time.sleep(0.7)
            
            print("  Step 7: Closing gripper...")
            board.bus_servo_set_position(0.4, [[R_GRIPPER, 750]])  # Close gripper
            time.sleep(0.3)
            
            # Tighten gripper for secure hold
            print("  Tightening gripper...")
            board.bus_servo_set_position(0.3, [[R_GRIPPER, 100]])  # Tighten further
            time.sleep(0.4)
            
            # Beep to confirm
            board.set_buzzer(1500, 0.1, 0.05, 1)
            time.sleep(0.2)
            
            print("  Step 8: Lifting...")
            board.bus_servo_set_position(0.6, [
                [R_SHO_PITCH, 400],   # Lift arm
                [R_EL_PITCH, 600],    # Bend elbow
            ])
            time.sleep(0.7)
        
        # Final pose - hold object in front
        print("  Holding object...")
        board.bus_servo_set_position(0.5, [
            [HEAD_TILT, 500],
            [HEAD_PAN, 500],
        ])
        time.sleep(0.4)
        
        # Green LED to indicate success
        board.set_rgb([[1, 0, 255, 0]])
        time.sleep(1.0)
        board.set_rgb([[1, 0, 0, 0]])
        
        print("  Grab complete! Robot should be holding the red bin.")
        print("  Use 'drop' skill to release.")
        print(f"  Note: Uses 'move_up' action for two-handed bin grabbing.")
        
    except Exception as e:
        print(f"  Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cap.release()


def skill_grab_red_cube(board, arm='right'):
    """
    Look for a red cube on the floor and grab it with the specified hand.
    Uses the 'crawl_left' or 'crawl_right' action for reliable one-handed grabbing.
    
    Args:
        arm: 'left' or 'right' - which arm to use for grabbing
    """
    print(f"Skill: Grab Red Cube ({arm} hand)")
    
    global action_player
    
    # Create debug directory
    debug_dir = "./debug"
    os.makedirs(debug_dir, exist_ok=True)
    
    # Camera options
    camera_options = [
        "/dev/video0",
        "/dev/video1",
        "/dev/usb_cam",
        0, 1, 2
    ]
    
    cap = None
    for cam in camera_options:
        if isinstance(cam, str) and not os.path.exists(cam):
            continue
        print(f"  Trying camera: {cam}...")
        test_cap = cv2.VideoCapture(cam)
        if test_cap.isOpened():
            ret, frame = test_cap.read()
            if ret and frame is not None:
                cap = test_cap
                print(f"  Camera {cam} opened successfully!")
                break
            test_cap.release()
    
    if cap is None:
        print("  ERROR: Could not open any camera!")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("  Camera opened")
    
    try:
        # Step 1: Look DOWN to see the floor
        print("  Step 1: Looking down at floor...")
        board.bus_servo_set_position(0.5, [[HEAD_TILT, 300], [HEAD_PAN, 500]])
        time.sleep(0.6)
        
        # Step 2: Search for red object
        print("  Step 2: Searching for red cube...")
        found = False
        center_x = 0
        center_y = 0
        frame_width = 640
        frame_height = 480
        last_frame = None
        
        for attempt in range(30):
            ret, frame = cap.read()
            if not ret:
                continue
            
            last_frame = frame
            found, cx, cy, area = detect_red_object(frame)
            
            if found and area > 500:  # Need reasonable size for a cube
                center_x = cx
                center_y = cy
                frame_width = frame.shape[1]
                frame_height = frame.shape[0]
                print(f"  Found red cube at ({cx}, {cy}), area={area}")
                # Save debug image
                debug_frame = frame.copy()
                cv2.circle(debug_frame, (cx, cy), 10, (0, 255, 0), 2)
                cv2.putText(debug_frame, f"CUBE ({area})", (cx-40, cy-20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                debug_path = os.path.join(debug_dir, "cube_detected.jpg")
                cv2.imwrite(debug_path, debug_frame)
                print(f"  Debug image saved to {debug_path}")
                break
            
            time.sleep(0.1)
        
        if not found:
            print("  No red cube detected!")
            if last_frame is not None:
                debug_path = os.path.join(debug_dir, "cube_notfound.jpg")
                cv2.imwrite(debug_path, last_frame)
                print(f"  Debug image saved to {debug_path}")
            board.bus_servo_set_position(0.5, [[HEAD_TILT, 500]])
            time.sleep(0.6)
            cap.release()
            return
        
        # Initialize action player
        if action_player is None:
            action_player = ActionPlayer(board)
        
        # Step 3: Use the specified arm's action
        grab_action = 'crawl_right' if arm == 'right' else 'crawl_left'
        print(f"  Step 3: Using {arm} arm to grab...")
        
        if action_player.action_exists(grab_action):
            print(f"  Using '{grab_action}' action for one-handed grab...")
            # Move hand back first (like visual_patrol_pick_up does)
            if action_player.action_exists('hand_back'):
                action_player.play_action('hand_back')
            
            # Play the grab action
            action_player.play_action(grab_action)
            
            # Tighten gripper for secure hold
            print("  Tightening gripper...")
            if arm == 'right':
                board.bus_servo_set_position(0.3, [[R_GRIPPER, 100]])  # Tighten right
            else:
                board.bus_servo_set_position(0.3, [[L_GRIPPER, 900]])  # Tighten left
            time.sleep(0.4)
            
            # Confirm grab with beep
            board.set_buzzer(1500, 0.1, 0.05, 1)
            time.sleep(0.2)

            # Green LED to indicate success
            board.set_rgb([[1, 0, 255, 0]])
            time.sleep(1.0)
            board.set_rgb([[1, 0, 0, 0]])
        else:
            # Fallback: Manual one-handed grab sequence
            print(f"  '{grab_action}' action not found")
        
        print(f"  Grab complete! Robot should be holding the cube in {arm} hand.")
        print(f"  Use 'drop_{arm}' skill to release.")
        
    except Exception as e:
        print(f"  Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cap.release()


def skill_drop(board, arm='right'):
    """Drop whatever is in the specified gripper"""
    print(f"Skill: Drop Object ({arm} hand)")
    
    if arm == 'right':
        sho_pitch = R_SHO_PITCH
        el_pitch = R_EL_PITCH
        gripper = R_GRIPPER
        sho_init = 125
        gripper_open = 800
    else:
        sho_pitch = L_SHO_PITCH
        el_pitch = L_EL_PITCH
        gripper = L_GRIPPER
        sho_init = 875
        gripper_open = 0
    
    # Extend arm forward a bit
    print("  Extending arm...")
    board.bus_servo_set_position(0.5, [
        [sho_pitch, 500],
        [el_pitch, 400 if arm == 'right' else 600],
    ])
    time.sleep(0.6)
    
    # Open gripper
    print("  Releasing...")
    board.bus_servo_set_position(0.3, [[gripper, gripper_open]])
    time.sleep(0.4)
    
    # Return arm to init
    board.bus_servo_set_position(0.5, [
        [sho_pitch, sho_init],
        [el_pitch, 500],
        [gripper, 500],
    ])
    time.sleep(0.6)
    
    print("  Drop complete!")


# =============================================================================
# SKILL REGISTRY
# =============================================================================

SKILLS = {
    'wave': ('Wave Right Arm', lambda b: skill_wave(b, 'right')),
    'wave_left': ('Wave Left Arm', lambda b: skill_wave(b, 'left')),
    'look_around': ('Head Look Around', skill_head_look_around),
    'nod': ('Nod Yes', skill_nod_yes),
    'shake': ('Shake No', skill_shake_no),
    'bow': ('Bow', skill_bow),
    'arms_up': ('Arms Up', skill_arms_up),
    'arms_cross': ('Arms Cross', skill_arms_cross),
    'grippers': ('Gripper Test', skill_grippers),
    'dance': ('Simple Dance', skill_dance_simple),
    'stretch': ('Stretch', skill_stretch),
    'flex': ('Flex Arms', skill_flex_arms),
    'point': ('Point Forward', lambda b: skill_point(b, 'forward')),
    'point_left': ('Point Left', lambda b: skill_point(b, 'left')),
    'point_right': ('Point Right', lambda b: skill_point(b, 'right')),
    'pushup': ('Pushup Motion', skill_pushup),
    'balance_kick': ('Balance & Kick Demo', skill_balance_kick),
    # Lie down skills
    'lie_down_front': ('Lie Down (Front)', skill_lie_down_front),
    'lie_down_back': ('Lie Down (Back)', skill_lie_down_back),
    # Walking (uses action files)
    'walk': ('Walk Forward (action)', skill_walk_forward),
    'walk_backward': ('Walk Backward (action)', skill_walk_backward),
    'walk_turn_left': ('Turn Left (action)', skill_walk_turn_left),
    'walk_turn_right': ('Turn Right (action)', skill_walk_turn_right),
    # Lights & Sound
    'rgb_rainbow': ('RGB Rainbow', skill_rgb_rainbow),
    'rgb_blink': ('RGB Blink', skill_rgb_blink),
    'buzzer': ('Buzzer Beep', skill_buzzer_beep),
    'melody': ('Buzzer Melody', skill_buzzer_melody),
    'imu': ('Read IMU', skill_read_imu),
    'battery': ('Read Battery', skill_read_battery),
    'reset': ('Reset Pose', skill_reset_pose),
    'standup': ('Standing Pose', skill_robot_standup),
    # Action file based skills (require running on robot)
    'kick_left': ('Kick Left (action)', skill_kick_left),
    'kick_right': ('Kick Right (action)', skill_kick_right),
    'getup_front': ('Get Up from Front (action)', skill_getup_front),
    'getup_back': ('Get Up from Back (action)', skill_getup_back),
    'stand_action': ('Stand (action)', skill_stand_action),
    'walk_ready': ('Walk Ready (action)', skill_walk_ready),
    'list_actions': ('List All Actions', skill_list_actions),
    'play_action': ('Play Custom Action', skill_play_action),
    # Vision + Manipulation
    'grab_right': ('Grab Cube (right hand)', lambda b: skill_grab_red_cube(b, 'right')),
    'grab_left': ('Grab Cube (left hand)', lambda b: skill_grab_red_cube(b, 'left')),
    'grab_bin': ('Grab Bin (two-hand)', skill_grab_bin),
    'drop_right': ('Drop (right hand)', lambda b: skill_drop(b, 'right')),
    'drop_left': ('Drop (left hand)', lambda b: skill_drop(b, 'left')),
}


# =============================================================================
# MAIN PROGRAM
# =============================================================================

def print_menu():
    """Print available skills menu"""
    print("\n" + "=" * 50)
    print("AINEX Robot Skills Test Menu")
    print("=" * 50)
    
    categories = {
        'Arm Movements': ['wave', 'wave_left', 'arms_up', 'arms_cross', 'flex', 'point', 'point_left', 'point_right', 'stretch'],
        'Head Movements': ['look_around', 'nod', 'shake', 'bow'],
        'Hand/Gripper': ['grippers', 'grab_right', 'grab_left', 'grab_bin', 'drop_right', 'drop_left'],
        'Performance': ['dance', 'pushup', 'balance_kick'],
        'Locomotion (demos)': ['walk', 'walk_backward', 'walk_turn_left', 'walk_turn_right', 'lie_down_front', 'lie_down_back'],
        'Lights & Sound': ['rgb_rainbow', 'rgb_blink', 'buzzer', 'melody'],
        'Sensors': ['imu', 'battery'],
        'Utility': ['reset', 'standup'],
        'Action Files (robot only)': ['kick_left', 'kick_right', 'getup_front', 'getup_back', 'stand_action', 'walk_ready', 'list_actions', 'play_action'],
    }
    
    num = 1
    skill_map = {}
    
    for category, skill_keys in categories.items():
        print(f"\n{category}:")
        for key in skill_keys:
            if key in SKILLS:
                name, _ = SKILLS[key]
                print(f"  {num:2}. {name:<20} [{key}]")
                skill_map[num] = key
                num += 1
    
    print(f"\n  0. Exit")
    print(f"  a. Run ALL skills")
    print("=" * 50)
    
    return skill_map


def run_interactive(board):
    """Run interactive menu mode"""
    while True:
        skill_map = print_menu()
        
        try:
            choice = input("\nEnter choice (number, skill name, or 'a' for all): ").strip().lower()
            
            if choice == '0' or choice == 'exit' or choice == 'q':
                print("Goodbye!")
                break
            
            if choice == 'a' or choice == 'all':
                print("\nRunning ALL skills...")
                for key in SKILLS:
                    name, func = SKILLS[key]
                    print(f"\n--- {name} ---")
                    try:
                        func(board)
                    except Exception as e:
                        print(f"  Error: {e}")
                    time.sleep(0.5)
                print("\nAll skills completed!")
                continue
            
            # Try as number
            if choice.isdigit():
                num = int(choice)
                if num in skill_map:
                    key = skill_map[num]
                    name, func = SKILLS[key]
                    print(f"\n--- {name} ---")
                    func(board)
                else:
                    print("Invalid number!")
            # Try as skill name
            elif choice in SKILLS:
                name, func = SKILLS[choice]
                print(f"\n--- {name} ---")
                func(board)
            else:
                print("Unknown skill!")
                
        except KeyboardInterrupt:
            print("\n\nInterrupted! Resetting pose...")
            skill_reset_pose(board)
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    parser = argparse.ArgumentParser(description='AINEX Robot Skills Test')
    parser.add_argument('--all', action='store_true', help='Run all skills sequentially')
    parser.add_argument('--skill', type=str, help='Run a specific skill by name')
    parser.add_argument('--list', action='store_true', help='List all available skills')
    args = parser.parse_args()
    
    if args.list:
        print("Available skills:")
        for key, (name, _) in SKILLS.items():
            print(f"  {key:<15} - {name}")
        return
    
    print("AINEX Robot Skills Test")
    print("Initializing board...")
    
    try:
        board = Board()
        print("Board connected successfully!")
    except Exception as e:
        print(f"Error connecting to board: {e}")
        print("Make sure the robot is powered on and connected.")
        return
    
    try:
        if args.all:
            print("\nRunning ALL skills...")
            for key in SKILLS:
                name, func = SKILLS[key]
                print(f"\n--- {name} ---")
                try:
                    func(board)
                except Exception as e:
                    print(f"  Error: {e}")
                time.sleep(0.5)
            print("\nAll skills completed!")
            
        elif args.skill:
            if args.skill in SKILLS:
                name, func = SKILLS[args.skill]
                print(f"\n--- {name} ---")
                func(board)
            else:
                print(f"Unknown skill: {args.skill}")
                print("Use --list to see available skills")
                
        else:
            run_interactive(board)
            
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        print("Returning to walk_ready position...")
        try:
            skill_walk_ready(board)
        except:
            pass
        print("Done!")


if __name__ == '__main__':
    main()
