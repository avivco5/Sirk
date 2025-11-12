#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASCII-only, english comments

Hybrid Telemetry GUI (no RC sliders) + PS4 joystick + YOLOv8 + DeepSORT
- Clean telemetry-only UI (no RC sliders)
- PS4 controller bars for live sticks (ROLL/PITCH/YAW/THR)
- Triangle short press = cycle flight mode (STABILIZE -> ALT_HOLD -> GUIDED -> LOITER)
  Triangle long press  = Emergency Stop
- Yaw PD from bbox center + Pitch forward-only from bbox width with smooth blending
  so both yaw and pitch act simultaneously
- Optional throttle compensation while moving forward
- Start/Stop detection buttons
- Simple MAVLink connect (TCP/UDP/Serial auto)
- Minimal health/battery/attitude panels
- Compact logging

Dependencies (install if missing):
  pip install opencv-python ultralytics deep-sort-realtime pygame pymavlink
"""

import os, sys, time, math, json, glob, socket, threading, queue, argparse, pathlib, logging, logging.handlers
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import font as tkfont

# Optional imports
try:
    import numpy as np

    if not hasattr(np, "float"): np.float = float
    if not hasattr(np, "int"):   np.int = int
    if not hasattr(np, "bool"):  np.bool = bool
except Exception:
    np = None

try:
    import cv2
    from ultralytics import YOLO
    from deep_sort_realtime.deepsort_tracker import DeepSort

    # Silence DeepSORT debug logs
    logging.getLogger('deep_sort_realtime').setLevel(logging.ERROR)
except Exception:
    cv2, YOLO, DeepSort = None, None, None

try:
    import pygame
    from pygame.locals import *
except Exception:
    pygame = None

try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

# Global Configs
MAV_BAUD = 57600
MAV_MASTER = None  # Default is auto-detect or user-defined

YOLO_CAM_INDEX = 0
YOLO_CONF = 0.5
YOLO_IOU = 0.5
DET_DOWNSCALE = 1.0  # Detection image downscale factor (e.g., 0.5 for half size)
YOLO_SHOW_WINDOW = True  # Show CV window with detection

# PID/Controller settings
YAW_P = 0.05  # Proportional gain for yaw (x center of bbox)
YAW_D = 0.001  # Derivative gain for yaw
PITCH_P = 0.0005  # Proportional gain for pitch (width of bbox)
PITCH_OFFSET = 0  # Offset to apply to Pitch calculation (e.g., +150 to keep a small distance)
PITCH_FWD_MAX = 500  # Max forward pitch (us value)
PITCH_REV_MAX = 0  # Max reverse pitch (us value)
YAW_MAX = 500  # Max yaw (us value)
YAW_CENTER_TOLERANCE = 0.05  # Center tolerance (percent of screen width)

THROTTLE_COMPENSATION = 0.0  # Multiplier for throttle compensation when moving forward (0.0 to 1.0)
THROTTLE_COMP_PITCH_THRESHOLD = 1550  # Pitch PWM threshold above which compensation is applied

# MAVLink constants
DEFAULT_RC_VALUES = {'roll': 1500, 'pitch': 1500, 'yaw': 1500, 'throttle': 1000, 'mode': 'STABILIZE'}
MAX_RC_US = 2000
MIN_RC_US = 1000
RC_CENTER = 1500
RC_DEFAULT_THROTTLE = 1000  # Use 1000 as default throttle to prevent unexpected movement

# PS4 joystick constants
PS4_AXIS_L_LEFT_RIGHT = 0
PS4_AXIS_L_UP_DOWN = 1
PS4_AXIS_R_LEFT_RIGHT = 2
PS4_AXIS_R_UP_DOWN = 3
PS4_AXIS_L2 = 4
PS4_AXIS_R2 = 5
PS4_BUTTON_SQUARE = 0
PS4_BUTTON_X = 1
PS4_BUTTON_CIRCLE = 2
PS4_BUTTON_TRIANGLE = 3
PS4_BUTTON_L1 = 4
PS4_BUTTON_R1 = 5
PS4_BUTTON_L2 = 6
PS4_BUTTON_R2 = 7
PS4_BUTTON_SHARE = 8
PS4_BUTTON_OPTIONS = 9
PS4_BUTTON_PS = 10
PS4_BUTTON_L3 = 11
PS4_BUTTON_R3 = 12

# GUI Colors
COLOR_BG = "#2E2E2E"
COLOR_FG = "#FFFFFF"
COLOR_PRIMARY = "#4CAF50"  # Green
COLOR_SECONDARY = "#2196F3"  # Blue
COLOR_WARN = "#FF9800"  # Orange
COLOR_ERROR = "#F44336"  # Red
COLOR_INFO = "#9E9E9E"  # Grey

# Logging setup
LOG_FILE = "hybrid_telemetry.log"
LOG_FORMAT = '%(asctime)s - %(levelname)s - %(message)s'
MAX_LOG_SIZE = 10 * 1024 * 1024  # 10MB
BACKUP_COUNT = 5


def setup_logging():
    global logger
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    # Console handler
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(logging.Formatter(LOG_FORMAT))
    logger.addHandler(ch)

    # File handler (Rotating)
    if not os.path.exists('logs'):
        os.makedirs('logs')
    fh = logging.handlers.RotatingFileHandler(
        pathlib.Path('logs') / LOG_FILE,
        maxBytes=MAX_LOG_SIZE,
        backupCount=BACKUP_COUNT,
        encoding='utf-8'
    )
    fh.setFormatter(logging.Formatter(LOG_FORMAT))
    logger.addHandler(fh)


setup_logging()


# --- Utility functions ---

def normalize_ps4_axis(value):
    """Normalize PS4 axis (usually -1.0 to 1.0) to 1000-2000 PWM range."""
    # Reverse the axis for up/down (like Y axis in most controllers)
    if value in [PS4_AXIS_L_UP_DOWN, PS4_AXIS_R_UP_DOWN]:
        value = -value
    return int((value * 500) + 1500)


def normalize_ps4_trigger(value):
    """Normalize PS4 trigger (usually -1.0 to 1.0) to 0.0 to 1.0 range."""
    # Triggers start at -1.0 (untouched) and go to 1.0 (fully pressed)
    return (value + 1.0) / 2.0


def scale_to_pwm(value, min_val, max_val, min_pwm=1000, max_pwm=2000):
    """Scale a value from its range (min_val to max_val) to PWM range (min_pwm to max_pwm)."""
    return int((value - min_val) * (max_pwm - min_pwm) / (max_val - min_val) + min_pwm)


def clamp(value, min_val, max_val):
    """Clamp a value within a specified range."""
    return max(min_val, min(max_val, value))


def lerp(a, b, t):
    """Linear interpolation between a and b by factor t (0.0 to 1.0)."""
    return a + (b - a) * t


# --- MAVLink Communication ---

# C:\Users\avivc\PycharmProjects\81\Drone\test2.py

# ... (ייבוא וקבועים לפני קלאס MAVLinkManager) ...

# --- MAVLink Communication ---

class MAVLinkManager:
    def __init__(self, master_address, baud_rate):
        self.master_address = master_address
        self.baud_rate = baud_rate
        self.mav = None
        self.rc_channels = dict(DEFAULT_RC_VALUES)
        self.is_connected = False
        self.thread = None
        self.running = False
        self.telemetry = {
            'mode': 'UNKNOWN', 'armed': False, 'voltage': 0.0, 'current': 0.0, 'battery_remaining': 0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'alt': 0.0, 'lat': 0.0, 'lon': 0.0, 'hdg': 0,
            'fix_type': 0, 'satellites': 0, 'status': 'DISCONNECTED', 'messages': []
        }
        self.message_queue = queue.Queue(maxsize=100)
        self.heartbeat_queue = queue.Queue(maxsize=1)

        # --- תיקון שגיאת AttributeError: MAV_MODE_STABILIZE ---
        # הגדרת מספרי Custom Mode של ArduPilot (Multirotor):
        # 0: STABILIZE, 2: ALT_HOLD, 4: GUIDED, 5: LOITER
        AP_MODE_STABILIZE = 0
        AP_MODE_ALT_HOLD = 2
        AP_MODE_GUIDED = 4  # GUIDED for NOGPS/Velocity control
        AP_MODE_LOITER = 5  # LOITER (requires GPS)

        # המילון custom_mode_map משמש לשליחת פקודות SET_MODE
        self.custom_mode_map = {
            'STABILIZE': AP_MODE_STABILIZE,
            'ALT_HOLD': AP_MODE_ALT_HOLD,
            'GUIDED': AP_MODE_GUIDED,
            'LOITER': AP_MODE_LOITER
        }

        # רשימת המצבים שדרכם המערכת יכולה לעבור (Cycling)
        self.mode_list = ['STABILIZE', 'ALT_HOLD', 'GUIDED', 'LOITER']
        self.current_mode_index = 0
        # -------------------------------------------------------------------

    def connect(self):
        if self.is_connected:
            return True
        try:
            logger.info(f"Attempting connection to master: {self.master_address} with baud: {self.baud_rate}...")
            self.mav = mavutil.mavlink_connection(
                self.master_address,
                baud=self.baud_rate,
                source_system=255,  # Unique system ID for ground station
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
            )
            self.mav.wait_heartbeat()
            self.is_connected = True
            logger.info(
                f"MAVLink heartbeat received. Connected to system {self.mav.target_system}, component {self.mav.target_component}.")
            self.telemetry['status'] = 'CONNECTED'

            self.running = True
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

            # Request all data streams (MAV_DATA_STREAM_ALL) at 10Hz
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,  # Rate in Hz
                1  # Start sending
            )

            return True
        except Exception as e:
            logger.error(f"MAVLink connection failed: {e}")
            self.telemetry['status'] = 'DISCONNECTED (Error)'
            return False

    def close(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        if self.mav:
            self.mav.close()
        self.is_connected = False
        self.telemetry['status'] = 'DISCONNECTED'
        logger.info("MAVLink connection closed.")

    def _run(self):
        logger.info("MAVLink receive thread started.")
        while self.running:
            try:
                # 1. Read message
                msg = self.mav.recv_match(blocking=False, timeout=0.1)

                # 2. Process message
                if msg:
                    msg_type = msg.get_type()

                    if msg_type == 'HEARTBEAT':
                        self.heartbeat_queue.put(True)
                        self.telemetry['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                        # קבלת שם המצב (Mode) מ-custom_mode
                        mode_num = msg.custom_mode
                        # שימוש ב-mode_mapping() המובנה של mavutil
                        mode_str = self.mav.mode_mapping().get(mode_num, f'Mode {mode_num}')

                        self.telemetry['mode'] = mode_str
                        # Update index for mode cycling
                        if mode_str in self.mode_list:
                            self.current_mode_index = self.mode_list.index(mode_str)
                        else:
                            self.current_mode_index = 0

                    elif msg_type == 'ATTITUDE':
                        self.telemetry['roll'] = math.degrees(msg.roll)
                        self.telemetry['pitch'] = math.degrees(msg.pitch)
                        self.telemetry['yaw'] = math.degrees(msg.yaw)

                    elif msg_type == 'SYS_STATUS':
                        # Minimal health check
                        voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != -1 else 0.0
                        current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
                        remaining = msg.battery_remaining if msg.battery_remaining != -1 else 0
                        self.telemetry['voltage'] = voltage
                        self.telemetry['current'] = current
                        self.telemetry['battery_remaining'] = remaining

                    elif msg_type == 'GLOBAL_POSITION_INT':
                        self.telemetry['alt'] = msg.relative_alt / 1000.0  # m
                        self.telemetry['lat'] = msg.lat / 1.0e7  # deg
                        self.telemetry['lon'] = msg.lon / 1.0e7  # deg
                        self.telemetry['hdg'] = msg.hdg / 100.0  # deg

                    elif msg_type == 'GPS_RAW_INT':
                        self.telemetry['fix_type'] = msg.fix_type
                        self.telemetry['satellites'] = msg.satellites_visible

                    elif msg_type == 'STATUSTEXT':
                        try:
                            text = msg.text.decode('utf-8').strip()
                        except AttributeError:
                            text = str(msg.text).strip()
                        self.log_message(f"MSG: {text}", level='INFO')

                    elif msg_type == 'RC_CHANNELS':
                        # Optional: Use RC_CHANNELS to show what the FC is actually receiving/outputting
                        pass

                    # Add message to queue for GUI display
                    if not self.message_queue.full() and msg_type not in ['HEARTBEAT', 'ATTITUDE', 'SYS_STATUS',
                                                                          'GLOBAL_POSITION_INT', 'GPS_RAW_INT']:
                        self.message_queue.put(f"RAW: {msg_type}")

                # 3. Send RC channels and/or heartbeat (if needed)
                if self.rc_channels:
                    # Send RC_CHANNELS_OVERRIDE
                    self.mav.mav.rc_channels_override_send(
                        self.mav.target_system,
                        self.mav.target_component,
                        self.rc_channels.get('roll', RC_CENTER),  # Channel 1
                        self.rc_channels.get('pitch', RC_CENTER),  # Channel 2
                        self.rc_channels.get('yaw', RC_CENTER),  # Channel 3
                        self.rc_channels.get('throttle', RC_DEFAULT_THROTTLE),  # Channel 4
                        0,  # Channel 5
                        0,  # Channel 6
                        0,  # Channel 7
                        0,  # Channel 8
                    )

            except mavutil.MavlinkError:
                # This usually happens if connection is lost
                self.telemetry['status'] = 'DISCONNECTED (MavError)'
                self.running = False
                self.is_connected = False
                logger.error("MAVLink thread error (MavlinkError). Stopping thread.")
            except socket.error:
                # Happens on timeout/disconnection
                self.telemetry['status'] = 'DISCONNECTED (SocketError)'
                self.running = False
                self.is_connected = False
                logger.error("MAVLink thread error (SocketError). Stopping thread.")
            except Exception as e:
                logger.error(f"MAVLink thread exception: {e}")
                time.sleep(0.01)  # Avoid tight loop on general exception

        logger.info("MAVLink receive thread stopped.")
        self.is_connected = False
        self.telemetry['status'] = 'DISCONNECTED'

    def set_rc_channels(self, **kwargs):
        """Update the RC channels to be sent."""
        for key, value in kwargs.items():
            if key in self.rc_channels:
                self.rc_channels[key] = clamp(value, MIN_RC_US, MAX_RC_US)
            else:
                logger.warning(f"Attempted to set unknown RC channel: {key}")

    def arm_disarm(self, arm=True):
        """Arm or Disarm the vehicle."""
        if not self.is_connected:
            logger.warning("Cannot arm/disarm, not connected.")
            return

        command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        param1 = 1 if arm else 0  # 1 to arm, 0 to disarm
        param2 = 21196 if arm else 0  # 21196 to force arming (for safety)

        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            command,
            0,  # Confirmation
            param1, param2, 0, 0, 0, 0, 0  # Parameters
        )
        action = "ARM" if arm else "DISARM"
        logger.info(f"Sent {action} command.")
        self.log_message(f"COMMAND: {action}", level='WARNING')

    def set_mode(self, mode_name):
        """Set the flight mode."""
        if not self.is_connected:
            logger.warning("Cannot set mode, not connected.")
            return

        # שימוש במילון custom_mode_map
        if mode_name not in self.custom_mode_map:
            logger.error(f"Unknown mode: {mode_name}")
            return

        custom_mode_number = self.custom_mode_map[mode_name]

        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode_number
        )
        logger.info(f"Sent SET_MODE to {mode_name} ({custom_mode_number}).")
        self.log_message(f"COMMAND: SET_MODE {mode_name}", level='WARNING')

    def cycle_mode(self):
        """Cycle through the predefined flight modes."""
        if not self.telemetry['armed']:
            self.log_message("ERROR: Cannot cycle mode, vehicle is DISARMED.", level='ERROR')
            logger.warning("Cannot cycle mode, vehicle is DISARMED.")
            return

        self.current_mode_index = (self.current_mode_index + 1) % len(self.mode_list)
        new_mode = self.mode_list[self.current_mode_index]
        self.set_mode(new_mode)

    def emergency_stop(self):
        """Disarm the vehicle and set throttle to min."""
        logger.critical("EMERGENCY STOP TRIGGERED!")
        self.arm_disarm(arm=False)
        self.set_rc_channels(throttle=RC_DEFAULT_THROTTLE)
        self.log_message("EMERGENCY STOP!", level='CRITICAL')

    def get_telemetry(self):
        return self.telemetry

    def get_log_messages(self):
        messages = []
        while not self.message_queue.empty():
            messages.append(self.message_queue.get_nowait())
        return messages

    def check_heartbeat(self):
        """Check if a heartbeat was received recently."""
        if self.is_connected:
            # Consume the heartbeat flag to check if one was received since last check
            try:
                self.heartbeat_queue.get_nowait()
                return True
            except queue.Empty:
                return False
        return False

    def log_message(self, message, level='INFO'):
        """Log a message for display in the GUI."""
        if not self.message_queue.full():
            self.message_queue.put(f"[{level}] {message}")


# --- Joystick Handling (PS4) ---

class PS4Joystick:
    def __init__(self, mav_manager):
        if not pygame:
            logger.error("Pygame is not installed. Joystick functionality disabled.")
            self.js = None
            return

        pygame.init()
        pygame.joystick.init()
        self.mav = mav_manager
        self.js = None
        self.thread = None
        self.running = False
        self.rc_override = {'roll': 0, 'pitch': 0, 'yaw': 0, 'throttle': 0}
        self.js_detected = False
        self.triangle_press_time = 0.0
        self.long_press_threshold = 1.0  # 1 second

        # PID controller state
        self.yaw_error_prev = 0.0
        self.pitch_error_prev = 0.0

        if pygame.joystick.get_count() == 0:
            logger.warning("No joystick detected by Pygame.")
        else:
            try:
                self.js = pygame.joystick.Joystick(0)
                self.js.init()
                self.js_detected = True
                logger.info(f"PS4 Joystick detected: {self.js.get_name()}")

                self.running = True
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()
            except pygame.error as e:
                logger.error(f"Error initializing joystick: {e}")
                self.js_detected = False
                self.js = None

    def quit(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)
        if pygame:
            pygame.quit()
        logger.info("Joystick thread stopped.")

    def _run(self):
        logger.info("Joystick thread started.")
        while self.running:
            try:
                pygame.event.pump()
                if self.js and self.js_detected:

                    # --- Axis Reading (Sticks) ---
                    roll_axis = self.js.get_axis(PS4_AXIS_R_LEFT_RIGHT)
                    pitch_axis = self.js.get_axis(PS4_AXIS_R_UP_DOWN)
                    yaw_axis = self.js.get_axis(PS4_AXIS_L_LEFT_RIGHT)
                    throttle_axis = self.js.get_axis(PS4_AXIS_L_UP_DOWN)

                    # Normalize to 1000-2000 range
                    roll = normalize_ps4_axis(roll_axis)
                    pitch = normalize_ps4_axis(pitch_axis)
                    yaw = normalize_ps4_axis(yaw_axis)
                    throttle = normalize_ps4_axis(throttle_axis)

                    # Apply reverse for Throttle (so UP is high throttle, DOWN is low)
                    # The default normalize function already does this for Y axes
                    # throttle = normalize_ps4_axis(-throttle_axis)

                    # Map L2/R2 to fine-tune roll/pitch/yaw/throttle (not used here, but good reference)
                    # l2 = normalize_ps4_trigger(self.js.get_axis(PS4_AXIS_L2))
                    # r2 = normalize_ps4_trigger(self.js.get_axis(PS4_AXIS_R2))

                    # Update RC override
                    self.rc_override['roll'] = roll
                    self.rc_override['pitch'] = pitch
                    self.rc_override['yaw'] = yaw
                    # Throttle needs to be clamped to [1000, 2000], but we change the lower limit
                    # Since normalize_ps4_axis gives [1000, 2000], we clamp only the upper part
                    self.rc_override['throttle'] = clamp(throttle, RC_DEFAULT_THROTTLE, MAX_RC_US)

                    # --- Button Reading ---

                    # Triangle button for mode cycling / emergency stop
                    triangle_pressed = self.js.get_button(PS4_BUTTON_TRIANGLE)

                    if triangle_pressed:
                        if self.triangle_press_time == 0.0:
                            self.triangle_press_time = time.time()

                        # Check for long press (Emergency Stop)
                        if time.time() - self.triangle_press_time > self.long_press_threshold:
                            self.mav.emergency_stop()
                            self.triangle_press_time = 0.0  # Reset to prevent immediate re-trigger

                    elif self.triangle_press_time > 0.0:
                        press_duration = time.time() - self.triangle_press_time
                        if press_duration < self.long_press_threshold:
                            # Short press (Mode Cycle)
                            self.mav.cycle_mode()

                        self.triangle_press_time = 0.0  # Reset

                    # X button for Arm/Disarm (toggle)
                    # Use get_event for one-shot button presses for arm/disarm
                    for event in pygame.event.get():
                        if event.type == JOYBUTTONDOWN:
                            if event.button == PS4_BUTTON_X:
                                if self.mav.telemetry['armed']:
                                    self.mav.arm_disarm(arm=False)
                                else:
                                    self.mav.arm_disarm(arm=True)

                time.sleep(0.01)  # 100Hz poll rate
            except Exception as e:
                logger.error(f"Joystick thread exception: {e}")
                time.sleep(0.1)

        logger.info("Joystick thread stopped.")

    def get_rc_override(self):
        """Get the current RC values from the joystick."""
        return self.rc_override

    def reset_pid_state(self):
        """Reset PID state for detection control."""
        self.yaw_error_prev = 0.0
        self.pitch_error_prev = 0.0

    def calculate_detection_control(self, bbox_center_x, bbox_width, frame_width, frame_height):
        """
        Calculate Yaw and Pitch RC commands based on detected bounding box.

        :param bbox_center_x: X coordinate of the bounding box center (in pixels).
        :param bbox_width: Width of the bounding box (in pixels).
        :param frame_width: Width of the video frame.
        :param frame_height: Height of the video frame.
        :return: (yaw_rc, pitch_rc) in 1000-2000 PWM range.
        """

        # --- Yaw Control (PD controller for centering on X axis) ---
        target_x = frame_width / 2
        center_tolerance_px = frame_width * YAW_CENTER_TOLERANCE

        # Error: how far the center is from the target center (-1.0 to 1.0)
        yaw_error = (bbox_center_x - target_x) / (frame_width / 2)
        yaw_error = clamp(yaw_error, -1.0, 1.0)

        # PD calculation
        yaw_p_term = YAW_P * yaw_error
        yaw_d_term = YAW_D * (yaw_error - self.yaw_error_prev)

        yaw_rate = (yaw_p_term + yaw_d_term)

        # Deadband for Yaw
        if abs(yaw_error * (frame_width / 2)) < center_tolerance_px:
            yaw_rate = 0.0

        # Scale to PWM range
        yaw_rate_us = int(yaw_rate * YAW_MAX)

        yaw_rc = clamp(RC_CENTER + yaw_rate_us, RC_CENTER - YAW_MAX, RC_CENTER + YAW_MAX)

        self.yaw_error_prev = yaw_error

        # --- Pitch Control (P controller for forward movement based on width) ---
        # Target width could be a fixed value or dynamic based on desired distance.
        # Here, we use a simple linear scaling from width to pitch value (forward only).

        # Width error: Simple mapping from bbox width to forward pitch
        # Assume smaller width -> move faster forward (higher pitch)
        # Assume max width (e.g., 80% of frame) means stop (1500 pitch)

        max_width = frame_width * 0.8
        min_width = 10  # Minimum width to consider for pitch control

        # Pitch PWM is scaled from max_width to min_width, mapping to 1500 to PITCH_FWD_MAX
        if bbox_width > max_width:
            pitch_rc = RC_CENTER
        elif bbox_width < min_width:
            pitch_rc = PITCH_FWD_MAX
        else:
            # Scale bbox_width (max_width to min_width) to PWM (RC_CENTER to PITCH_FWD_MAX)
            # Flipped range for width: high width means low pitch, low width means high pitch
            pitch_rc = scale_to_pwm(
                value=bbox_width,
                min_val=min_width,
                max_val=max_width,
                min_pwm=PITCH_FWD_MAX,  # When width is min_width, pitch is PITCH_FWD_MAX
                max_pwm=RC_CENTER  # When width is max_width, pitch is RC_CENTER (stop)
            )

        # Apply pitch offset (e.g., to keep a small distance)
        pitch_rc += PITCH_OFFSET

        # Clamp pitch to the defined range (forward only)
        pitch_rc = clamp(pitch_rc, RC_CENTER, PITCH_FWD_MAX)

        # --- Smooth Blending with Joystick Pitch ---
        # Only apply pitch control if detection is active AND joystick pitch is near center
        # If joystick pitch is high, the user takes over.

        # Joystick pitch value for reference (should be near 1500)
        js_pitch_val = self.rc_override['pitch']

        # Determine blending factor (t)
        # If joystick pitch is 1500, t=1 (full detection control)
        # If joystick pitch is far from 1500, t=0 (full joystick control)
        # Let's use a simple deadband around 1500 for the joystick pitch

        # For simplicity, assume user is overriding if they push pitch or roll.

        # If the user is actively controlling pitch (pushing forward/backward), use their pitch.
        # We assume the user takes over pitch control if they move the stick from center
        pitch_stick_deadband = 50  # 1450 to 1550 is deadband for blending

        if abs(js_pitch_val - RC_CENTER) > pitch_stick_deadband:
            # User is actively controlling pitch, so we only use the calculated YAW and the user's PITCH.
            # We still need to calculate a throttle compensation if needed
            final_pitch_rc = js_pitch_val
        else:
            # User is not actively controlling pitch, use the calculated pitch
            final_pitch_rc = pitch_rc

        # Yaw always blends with the joystick's yaw. If the user moves the yaw stick, 
        # it overrides/blends with the detection yaw.
        # Simple method: blend yaw only when joystick yaw is near center (1500)
        yaw_stick_deadband = 50
        js_yaw_val = self.rc_override['yaw']

        if abs(js_yaw_val - RC_CENTER) > yaw_stick_deadband:
            final_yaw_rc = js_yaw_val  # User takes over yaw
        else:
            final_yaw_rc = yaw_rc  # Use detection yaw

        # --- Throttle Compensation ---
        throttle_comp = 0
        if THROTTLE_COMPENSATION > 0.0 and final_pitch_rc > THROTTLE_COMP_PITCH_THRESHOLD:
            # Calculate compensation based on pitch value above threshold (i.e., forward movement)
            pitch_delta = final_pitch_rc - THROTTLE_COMP_PITCH_THRESHOLD
            max_delta = PITCH_FWD_MAX - THROTTLE_COMP_PITCH_THRESHOLD

            # Scale pitch_delta to a 0.0 to 1.0 factor
            comp_factor = pitch_delta / max_delta

            # Compensation is a portion of the throttle range (1000-2000)
            throttle_comp = int(comp_factor * (2000 - RC_DEFAULT_THROTTLE) * THROTTLE_COMPENSATION)

        final_throttle_rc = self.rc_override['throttle'] + throttle_comp
        final_throttle_rc = clamp(final_throttle_rc, RC_DEFAULT_THROTTLE, MAX_RC_US)

        return final_yaw_rc, final_pitch_rc, final_throttle_rc


# --- YOLOv8 and DeepSORT Tracking ---

class YOLOTracker:
    def __init__(self, cam_index, conf_thres, iou_thres, downscale_factor, show_window):
        if not cv2 or not YOLO or not DeepSort:
            logger.error(
                "Required detection libraries (OpenCV, Ultralytics, DeepSORT) are not installed. Detection disabled.")
            self.model = None
            self.tracker = None
            self.cap = None
            self.running = False
            self.thread = None
            self.mav = None
            self.detection_active = False
            return

        self.cam_index = cam_index
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.downscale_factor = downscale_factor
        self.show_window = show_window

        self.model = YOLO('yolov8n.pt')  # Load default YOLOv8 Nano model
        self.tracker = DeepSort(max_age=30, n_init=3, nms_max_overlap=0.7)

        self.cap = None
        self.running = False
        self.thread = None
        self.mav = None
        self.js = None
        self.detection_active = False
        self.tracking_data = {'id': -1, 'bbox': (0, 0, 0, 0), 'center': (0, 0), 'frame_size': (0, 0)}
        self.frame_queue = queue.Queue(maxsize=1)

        self.default_rc_channels = dict(DEFAULT_RC_VALUES)

    def set_mav_js(self, mav_manager, ps4_joystick):
        """Set MAVLink and PS4 managers."""
        self.mav = mav_manager
        self.js = ps4_joystick

    def start_camera(self):
        """Start the camera capture and the detection thread."""
        if not self.model or self.running:
            return False

        try:
            self.cap = cv2.VideoCapture(self.cam_index)
            if not self.cap.isOpened():
                logger.error(f"Cannot open camera with index {self.cam_index}")
                self.cap = None
                return False

            # Set resolution (optional)
            # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            self.running = True
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            logger.info(f"YOLO Tracker thread started for camera {self.cam_index}.")
            return True

        except Exception as e:
            logger.error(f"Error starting camera or tracker: {e}")
            if self.cap: self.cap.release()
            self.cap = None
            self.running = False
            return False

    def stop(self):
        """Stop the detection thread and release camera."""
        if not self.running:
            return

        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)

        if self.cap:
            self.cap.release()

        if self.show_window and cv2:
            cv2.destroyAllWindows()

        logger.info("YOLO Tracker stopped.")

    def toggle_detection_active(self):
        """Toggle the state of detection (ON/OFF)."""
        self.detection_active = not self.detection_active
        if not self.detection_active and self.js:
            # If turning off, reset PID state
            self.js.reset_pid_state()
            # Reset tracking data
            self.tracking_data = {'id': -1, 'bbox': (0, 0, 0, 0), 'center': (0, 0), 'frame_size': (0, 0)}

        logger.info(f"Detection Active: {self.detection_active}")

    def get_tracking_data(self):
        """Return the current tracking data for display/control."""
        return self.tracking_data

    def _run(self):
        while self.running and self.cap:
            ret, frame = self.cap.read()
            if not ret:
                logger.warning("Failed to read frame from camera.")
                time.sleep(0.1)
                continue

            frame_height, frame_width = frame.shape[:2]

            # Downscale frame for faster processing (if factor is set)
            if self.downscale_factor != 1.0:
                d_width = int(frame_width * self.downscale_factor)
                d_height = int(frame_height * self.downscale_factor)
                frame_detect = cv2.resize(frame, (d_width, d_height), interpolation=cv2.INTER_LINEAR)
            else:
                frame_detect = frame

            # --- YOLO Detection ---
            results = self.model.predict(
                source=frame_detect,
                conf=self.conf_thres,
                iou=self.iou_thres,
                verbose=False
            )[0]

            # Prepare detections for DeepSORT
            detections = []
            for box in results.boxes.data:
                x1, y1, x2, y2, conf, cls = box.tolist()
                bbox = [int(x1), int(y1), int(x2), int(y2)]  # x1, y1, x2, y2

                # Rescale bounding box back to original frame size if downscaled
                if self.downscale_factor != 1.0:
                    scale_x = frame_width / frame_detect.shape[1]
                    scale_y = frame_height / frame_detect.shape[0]
                    bbox = [
                        int(x1 * scale_x),
                        int(y1 * scale_y),
                        int(x2 * scale_x),
                        int(y2 * scale_y)
                    ]

                # Convert x1, y1, x2, y2 to x, y, w, h
                w = bbox[2] - bbox[0]
                h = bbox[3] - bbox[1]
                bbox_xywh = [bbox[0], bbox[1], w, h]

                # Get class label (assuming 'person' is class 0)
                label = self.model.names[int(cls)]

                detections.append((bbox_xywh, conf, label))

            # --- DeepSORT Tracking ---
            tracks = self.tracker.update_tracks(detections, frame=frame)

            best_track = None
            best_confidence = -1

            # Find the best track (e.g., highest confidence or the one that has been tracked longest)
            # For simplicity, we just use the first 'person' track
            for track in tracks:
                if not track.is_confirmed():
                    continue

                track_id = track.track_id
                ltrb = track.to_ltrb()  # x1, y1, x2, y2

                x1, y1, x2, y2 = map(int, ltrb)

                # Assuming the detection with 'person' label is the target
                if 'person' in [d[2] for d in detections]:  # Simplified check
                    best_track = (track_id, (x1, y1, x2, y2))
                    break  # Use the first confirmed track found

            # --- Control and GUI Update ---
            if best_track:
                track_id, (x1, y1, x2, y2) = best_track

                # BBox center and width
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                bbox_width = x2 - x1

                self.tracking_data.update({
                    'id': track_id,
                    'bbox': (x1, y1, x2, y2),
                    'center': (center_x, center_y),
                    'frame_size': (frame_width, frame_height)
                })

                # Draw BBox and ID
                color = (0, 255, 0)  # Green BBox
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    frame,
                    f"ID: {track_id}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2
                )

                # Draw Center marker
                cv2.circle(frame, (int(center_x), int(center_y)), 5, color, -1)

                # Draw Frame Center marker
                frame_center = (frame_width // 2, frame_height // 2)
                cv2.circle(frame, frame_center, 5, (255, 0, 0), -1)  # Blue

                # Draw Yaw Tolerance lines
                tolerance_px = frame_width * YAW_CENTER_TOLERANCE
                cv2.line(frame, (frame_center[0] - int(tolerance_px), 0),
                         (frame_center[0] - int(tolerance_px), frame_height), (0, 0, 255), 1)  # Red
                cv2.line(frame, (frame_center[0] + int(tolerance_px), 0),
                         (frame_center[0] + int(tolerance_px), frame_height), (0, 0, 255), 1)  # Red

                # Apply control if active and armed
                if self.detection_active and self.mav and self.mav.telemetry['armed'] and self.js:
                    yaw_rc, pitch_rc, throttle_rc = self.js.calculate_detection_control(
                        center_x, bbox_width, frame_width, frame_height
                    )

                    # Log control values on frame for debug
                    cv2.putText(frame, f"YAW: {yaw_rc}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    cv2.putText(frame, f"PITCH: {pitch_rc}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    cv2.putText(frame, f"THROTTLE: {throttle_rc}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                    # Set RC Channels (blended with joystick roll)
                    js_rc = self.js.get_rc_override()
                    self.mav.set_rc_channels(
                        roll=js_rc['roll'],  # Roll is always joystick controlled
                        pitch=pitch_rc,
                        yaw=yaw_rc,
                        throttle=throttle_rc
                    )

            else:
                # No track found
                self.tracking_data.update({
                    'id': -1, 'bbox': (0, 0, 0, 0), 'center': (0, 0), 'frame_size': (frame_width, frame_height)
                })
                # If active, send center RC (blended with joystick roll/pitch/yaw)
                if self.detection_active and self.mav and self.js:
                    js_rc = self.js.get_rc_override()
                    self.mav.set_rc_channels(
                        roll=js_rc['roll'],
                        pitch=js_rc['pitch'],
                        yaw=js_rc['yaw'],
                        throttle=js_rc['throttle']
                    )

            # --- Display Frame ---
            if self.show_window:
                # Put the frame into the queue for the GUI if needed (not implemented here, but good pattern)
                # For this version, we use the standard cv2.imshow

                # Check if window is still open
                if cv2.getWindowProperty("YOLO Tracking", cv2.WND_PROP_VISIBLE) < 1:
                    self.show_window = False  # Close window if user closed it

                cv2.imshow("YOLO Tracking", frame)

                # Handle ESC key press to close the window
                if cv2.waitKey(1) & 0xFF == 27:
                    self.show_window = False

            # Time limit / FPS control (implied by camera FPS and processing time)
            time.sleep(0.01)  # Small sleep to yield

        logger.info("YOLO Tracker process finished.")


# --- TKinter GUI ---

class HybridTelemetryGUI:
    def __init__(self, master, baud, no_window, cam_index, conf, iou, det_downscale):
        self.root = tk.Tk()
        self.root.title("Hybrid Telemetry & PS4-YOLO Control")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.running = True
        self.mav_master = master
        self.mav_baud = baud

        # MAVLink, PS4, and YOLO managers
        self.m = MAVLinkManager(self.mav_master, self.mav_baud)
        self.js = PS4Joystick(self.m) if pygame else None
        self.yolo = YOLOTracker(cam_index, conf, iou, det_downscale, not no_window) if cv2 else None

        if self.yolo and self.js:
            self.yolo.set_mav_js(self.m, self.js)

        # UI Setup
        self._configure_styles()
        self._create_widgets()

        # Start the update loop
        self.root.after(100, self._update_gui)

    def _configure_styles(self):
        s = ttk.Style()
        s.theme_use('clam')

        # General background and foreground
        s.configure('.', background=COLOR_BG, foreground=COLOR_FG)
        s.configure('TFrame', background=COLOR_BG)
        s.configure('TLabel', background=COLOR_BG, foreground=COLOR_FG, font=('Arial', 10))
        s.configure('TButton', background=COLOR_PRIMARY, foreground=COLOR_FG, font=('Arial', 10, 'bold'))
        s.map('TButton',
              background=[('active', COLOR_PRIMARY)],
              foreground=[('active', COLOR_FG)]
              )

        # Custom styles
        s.configure('Header.TLabel', font=('Arial', 12, 'bold'), foreground=COLOR_PRIMARY)
        s.configure('Status.TLabel', font=('Arial', 10, 'bold'))
        s.configure('Data.TLabel', font=('Courier', 10))

        s.configure('ProgressBar.Horizontal', troughcolor=COLOR_BG, background=COLOR_PRIMARY, bordercolor=COLOR_FG,
                    lightcolor=COLOR_PRIMARY, darkcolor=COLOR_PRIMARY)

        # Console Text Area
        self.root.option_add('*Text.background', COLOR_BG)
        self.root.option_add('*Text.foreground', COLOR_FG)
        self.root.option_add('*Text.font', 'Courier 9')

    def _create_widgets(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10 10 10 10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        # --- Connection/Status Panel (Row 0) ---
        status_frame = ttk.LabelFrame(main_frame, text="System Status", padding="5 5 5 5")
        status_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        status_frame.grid_columnconfigure(1, weight=1)

        ttk.Label(status_frame, text="MAVLink Status:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.mav_status_label = ttk.Label(status_frame, text="DISCONNECTED", style='Status.TLabel',
                                          foreground=COLOR_ERROR)
        self.mav_status_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)

        ttk.Label(status_frame, text="Heartbeat:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.heartbeat_indicator = ttk.Label(status_frame, text="●", foreground=COLOR_ERROR, font=('Arial', 12, 'bold'))
        self.heartbeat_indicator.grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)

        ttk.Label(status_frame, text="Joystick Status:").grid(row=0, column=2, sticky=tk.E, padx=5, pady=2)
        js_status = "DETECTED" if self.js and self.js.js_detected else "MISSING"
        js_color = COLOR_PRIMARY if self.js and self.js.js_detected else COLOR_WARN
        ttk.Label(status_frame, text=js_status, style='Status.TLabel', foreground=js_color).grid(row=0, column=3,
                                                                                                 sticky=tk.W, padx=5,
                                                                                                 pady=2)

        ttk.Label(status_frame, text="YOLO/DeepSORT:").grid(row=1, column=2, sticky=tk.E, padx=5, pady=2)
        yolo_status = "ENABLED" if self.yolo else "DISABLED"
        yolo_color = COLOR_PRIMARY if self.yolo else COLOR_WARN
        ttk.Label(status_frame, text=yolo_status, style='Status.TLabel', foreground=yolo_color).grid(row=1, column=3,
                                                                                                     sticky=tk.W,
                                                                                                     padx=5, pady=2)

        # --- Telemetry Panel (Row 1) ---
        telem_frame = ttk.LabelFrame(main_frame, text="Telemetry Data", padding="5 5 5 5")
        telem_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)

        # Attitude
        attitude_frame = ttk.LabelFrame(telem_frame, text="Attitude (deg)", padding="5")
        attitude_frame.grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.roll_val = self._create_data_label(attitude_frame, "Roll:", 0)
        self.pitch_val = self._create_data_label(attitude_frame, "Pitch:", 1)
        self.yaw_val = self._create_data_label(attitude_frame, "Yaw:", 2)

        # Health/Battery
        health_frame = ttk.LabelFrame(telem_frame, text="Health", padding="5")
        health_frame.grid(row=0, column=1, sticky=tk.W, padx=5, pady=5)
        self.mode_val = self._create_data_label(health_frame, "Mode:", 0)
        self.arm_val = self._create_data_label(health_frame, "Armed:", 1, is_status=True)
        self.voltage_val = self._create_data_label(health_frame, "Voltage (V):", 2)
        self.current_val = self._create_data_label(health_frame, "Current (A):", 3)
        self.battery_bar = ttk.Progressbar(health_frame, orient='horizontal', length=100, mode='determinate',
                                           style='ProgressBar.Horizontal')
        self.battery_bar.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), padx=5, pady=5)
        self.battery_val = ttk.Label(health_frame, text="Bat: -%", style='Data.TLabel')
        self.battery_val.grid(row=5, column=0, columnspan=2, sticky=tk.W, padx=5)

        # GPS
        gps_frame = ttk.LabelFrame(telem_frame, text="GPS", padding="5")
        gps_frame.grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.alt_val = self._create_data_label(gps_frame, "Rel Alt (m):", 0)
        self.lat_val = self._create_data_label(gps_frame, "Lat (deg):", 1)
        self.lon_val = self._create_data_label(gps_frame, "Lon (deg):", 2)
        self.fix_val = self._create_data_label(gps_frame, "Fix Type:", 3)
        self.sat_val = self._create_data_label(gps_frame, "Sats:", 4)

        # --- RC Sliders (Row 1, Column 1) ---
        rc_frame = ttk.LabelFrame(main_frame, text="RC Control (1000-2000µs)", padding="5 5 5 5")
        rc_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)

        self.rc_roll_bar = self._create_rc_bar(rc_frame, "Roll:", 0)
        self.rc_pitch_bar = self._create_rc_bar(rc_frame, "Pitch:", 1)
        self.rc_yaw_bar = self._create_rc_bar(rc_frame, "Yaw:", 2)
        self.rc_throttle_bar = self._create_rc_bar(rc_frame, "Throttle:", 3)

        # --- Buttons Panel (Row 2) ---
        buttons_frame = ttk.Frame(main_frame, padding="5 0 5 0")
        buttons_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        connect_btn = ttk.Button(buttons_frame, text="Connect", command=self.m.connect)
        connect_btn.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        arm_btn = ttk.Button(buttons_frame, text="ARM (X)", command=lambda: self.m.arm_disarm(arm=True))
        arm_btn.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        disarm_btn = ttk.Button(buttons_frame, text="DISARM (X)", command=lambda: self.m.arm_disarm(arm=False))
        disarm_btn.grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)

        mode_btn = ttk.Button(buttons_frame, text="Cycle Mode (Tri)", command=self.m.cycle_mode)
        mode_btn.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)

        # YOLO Buttons
        if self.yolo:
            yolo_btn_text = tk.StringVar(value="START YOLO")
            self.yolo_btn = ttk.Button(buttons_frame, textvariable=yolo_btn_text,
                                       command=lambda: self._yolo_toggle(yolo_btn_text))
            self.yolo_btn.grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)
        else:
            ttk.Label(buttons_frame, text="YOLO Disabled", foreground=COLOR_WARN).grid(row=0, column=4, padx=5, pady=5,
                                                                                       sticky=tk.W)

        # --- Log Console (Row 3) ---
        console_frame = ttk.LabelFrame(main_frame, text="MAVLink Messages & Logs", padding="5 5 5 5")
        console_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        main_frame.grid_rowconfigure(3, weight=1)
        console_frame.grid_columnconfigure(0, weight=1)
        console_frame.grid_rowconfigure(0, weight=1)

        self.console_text = tk.Text(console_frame, wrap=tk.WORD, height=10, state=tk.DISABLED)
        self.console_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)

        # Scrollbar for console
        console_scrollbar = ttk.Scrollbar(console_frame, command=self.console_text.yview)
        console_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.console_text['yscrollcommand'] = console_scrollbar.set

        self.console_text.tag_configure('CRITICAL', foreground=COLOR_ERROR)
        self.console_text.tag_configure('ERROR', foreground=COLOR_ERROR)
        self.console_text.tag_configure('WARNING', foreground=COLOR_WARN)
        self.console_text.tag_configure('INFO', foreground=COLOR_FG)
        self.console_text.tag_configure('COMMAND', foreground=COLOR_SECONDARY)

    def _create_data_label(self, parent_frame, label_text, row, is_status=False):
        """Helper to create a label pair for telemetry data."""
        ttk.Label(parent_frame, text=label_text).grid(row=row, column=0, sticky=tk.W, padx=5, pady=2)

        if is_status:
            var = tk.StringVar(value="-")
            label = ttk.Label(parent_frame, textvariable=var, style='Status.TLabel', foreground=COLOR_ERROR)
        else:
            var = tk.StringVar(value="---")
            label = ttk.Label(parent_frame, textvariable=var, style='Data.TLabel')

        label.grid(row=row, column=1, sticky=tk.W, padx=5, pady=2)
        return var

    def _create_rc_bar(self, parent_frame, label_text, row):
        """Helper to create an RC channel bar and label."""
        ttk.Label(parent_frame, text=label_text).grid(row=row, column=0, sticky=tk.W, padx=5, pady=2)

        # Progress bar (simulates slider)
        bar = ttk.Progressbar(parent_frame, orient='horizontal', length=200, mode='determinate',
                              style='ProgressBar.Horizontal')
        bar.grid(row=row, column=1, sticky=(tk.W, tk.E), padx=5, pady=2)
        bar['maximum'] = MAX_RC_US - MIN_RC_US
        bar['value'] = RC_CENTER - MIN_RC_US

        # Label to show value
        val_var = tk.StringVar(value="1500")
        label = ttk.Label(parent_frame, textvariable=val_var, style='Data.TLabel')
        label.grid(row=row, column=2, sticky=tk.W, padx=5)

        return bar, val_var

    def _yolo_toggle(self, btn_text_var):
        """Toggle YOLO detection and camera stream."""
        if self.yolo is None:
            messagebox.showerror("Error", "YOLO/DeepSORT is disabled (missing dependencies).")
            return

        if not self.yolo.running:
            if not self.yolo.start_camera():
                messagebox.showerror("Error", "Could not start camera stream.")
                return
            btn_text_var.set("STOP YOLO")
            self.yolo.toggle_detection_active()  # Start detection active immediately
            self.m.log_message("YOLO Stream & Detection STARTED", level='INFO')
        else:
            self.yolo.stop()
            btn_text_var.set("START YOLO")
            if self.yolo.detection_active:  # Ensure detection active is false if it was running
                self.yolo.toggle_detection_active()
            self.m.log_message("YOLO Stream STOPPED", level='INFO')

    def _update_gui(self):
        """Main periodic GUI update function."""
        if not self.running:
            return

        telemetry = self.m.get_telemetry()

        # 1. Update Status Panel
        self.mav_status_label.config(text=telemetry['status'])
        if telemetry['status'] == 'CONNECTED':
            self.mav_status_label.config(foreground=COLOR_PRIMARY)
        elif 'DISCONNECTED' in telemetry['status']:
            self.mav_status_label.config(foreground=COLOR_ERROR)
        else:
            self.mav_status_label.config(foreground=COLOR_WARN)

        # Heartbeat indicator
        if self.m.check_heartbeat():
            self.heartbeat_indicator.config(foreground=COLOR_PRIMARY)
        else:
            self.heartbeat_indicator.config(foreground=COLOR_ERROR)

        # 2. Update Telemetry
        self.roll_val.set(f"{telemetry['roll']:.2f}")
        self.pitch_val.set(f"{telemetry['pitch']:.2f}")
        self.yaw_val.set(f"{telemetry['yaw']:.2f}")

        self.mode_val.set(telemetry['mode'])

        arm_status = "ARMED" if telemetry['armed'] else "DISARMED"
        arm_color = COLOR_PRIMARY if telemetry['armed'] else COLOR_ERROR
        self.arm_val.set(arm_status)
        self.arm_val.get_label().config(foreground=arm_color)  # Assuming get_label is available or manual access
        # tkinter.StringVar doesn't have get_label, need to get the label widget directly
        self.arm_val.label_widget.config(foreground=arm_color)

        self.voltage_val.set(f"{telemetry['voltage']:.2f}")
        self.current_val.set(f"{telemetry['current']:.2f}")

        # Battery Bar
        bat_rem = telemetry['battery_remaining']
        self.battery_bar['value'] = bat_rem
        self.battery_bar['maximum'] = 100
        self.battery_val.set(f"Bat: {bat_rem}%")
        if bat_rem < 20 and bat_rem > 0:
            self.battery_bar.config(style='ProgressBar.Horizontal', background=COLOR_ERROR)
        elif bat_rem < 40 and bat_rem > 0:
            self.battery_bar.config(style='ProgressBar.Horizontal', background=COLOR_WARN)
        else:
            self.battery_bar.config(style='ProgressBar.Horizontal', background=COLOR_PRIMARY)

        self.alt_val.set(f"{telemetry['alt']:.2f}")
        self.lat_val.set(f"{telemetry['lat']:.6f}")
        self.lon_val.set(f"{telemetry['lon']:.6f}")
        self.fix_val.set(str(telemetry['fix_type']))
        self.sat_val.set(str(telemetry['satellites']))

        # 3. Update RC Bars
        if self.js and self.m.is_connected:
            # Use the actual RC values being sent (which are the JS values or the blended values)
            rc_values = self.m.rc_channels
        elif self.js and not self.m.is_connected:
            # Show raw JS values even if not connected
            rc_values = self.js.get_rc_override()
        else:
            # Fallback to default/last known
            rc_values = self.m.rc_channels

        # Helper to update RC bar
        def update_rc_bar(bar_val_tuple, channel_name):
            bar, val_var = bar_val_tuple
            val = rc_values.get(channel_name, RC_CENTER)
            bar['value'] = val - MIN_RC_US
            val_var.set(str(val))

        update_rc_bar(self.rc_roll_bar, 'roll')
        update_rc_bar(self.rc_pitch_bar, 'pitch')
        update_rc_bar(self.rc_yaw_bar, 'yaw')
        update_rc_bar(self.rc_throttle_bar, 'throttle')

        # 4. Update Console
        new_messages = self.m.get_log_messages()
        if new_messages:
            self.console_text.config(state=tk.NORMAL)
            for msg in new_messages:
                # Simple tagging based on the [LEVEL] prefix
                tag = 'INFO'
                if msg.startswith('[CRITICAL]'):
                    tag = 'CRITICAL'
                elif msg.startswith('[ERROR]'):
                    tag = 'ERROR'
                elif msg.startswith('[WARNING]'):
                    tag = 'WARNING'
                elif msg.startswith('[COMMAND]'):
                    tag = 'COMMAND'

                self.console_text.insert(tk.END, msg + "\n", tag)
            self.console_text.see(tk.END)  # Auto scroll to bottom
            self.console_text.config(state=tk.DISABLED)

        # 5. Update YOLO Button State
        if self.yolo and hasattr(self, 'yolo_btn'):
            if self.yolo.running and self.yolo.detection_active:
                self.yolo_btn.config(style='TButton', background=COLOR_ERROR, text="STOP DETECTION (ON)")
            elif self.yolo.running and not self.yolo.detection_active:
                self.yolo_btn.config(style='TButton', background=COLOR_SECONDARY, text="START DETECTION (OFF)")
            elif not self.yolo.running:
                self.yolo_btn.config(style='TButton', background=COLOR_PRIMARY, text="START YOLO")

        # Rerun update after 100ms
        self.root.after(100, self._update_gui)

    def run(self):
        # Initial connection attempt
        if self.mav_master:
            self.m.connect()

        # Manually attach the label widget to the StringVar for arm status coloring
        # This is a hack because ttk.Label doesn't expose its widget easily from a helper
        for widget in self.root.winfo_children():
            for sub_widget in widget.winfo_children():
                if isinstance(sub_widget, tk.Label) or isinstance(sub_widget, ttk.Label):
                    if sub_widget.cget("text") == self.arm_val.get():
                        self.arm_val.label_widget = sub_widget
                        break
            if hasattr(self.arm_val, 'label_widget'):
                break

        self.root.mainloop()

    def on_closing(self):
        if not self.running:
            return
        if messagebox.askokcancel("Exit", "Close GUI?"):
            self.running = False
            try:
                self.stop_yolo()
            except Exception:
                pass
            try:
                self.m.close()
            except Exception:
                pass
            try:
                if self.js: self.js.quit()
            except Exception:
                pass
            try:
                if pygame: pygame.quit()
            except Exception:
                pass
            self.root.destroy()


# -------------- CLI --------------
def _parse_args():
    p = argparse.ArgumentParser(description="Hybrid Telemetry GUI + PS4 + YOLO")
    p.add_argument("--master", default=None)
    p.add_argument("--baud", type=int, default=None)
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--cam-index", type=int, default=YOLO_CAM_INDEX)
    p.add_argument("--conf", type=float, default=YOLO_CONF)
    p.add_argument("--iou", type=float, default=YOLO_IOU)
    p.add_argument("--det-downscale", type=float, default=DET_DOWNSCALE)
    return p.parse_args()


def main():
    global YOLO_CAM_INDEX, YOLO_CONF, YOLO_IOU, DET_DOWNSCALE, YOLO_SHOW_WINDOW
    args = _parse_args()
    YOLO_CAM_INDEX = args.cam_index
    YOLO_CONF = args.conf
    YOLO_IOU = args.iou
    DET_DOWNSCALE = args.det_downscale

    # If --no-window is set, we use this for both the main GUI and the CV window (if a GUI is needed)
    # But here, --no-window is for the CV window only, the TKinter GUI is always shown unless we exit.
    # YOLO_SHOW_WINDOW = not args.no_window # Handled in the GUI init

    # Set MAVLink connection parameters
    master = args.master if args.master is not None else MAV_MASTER
    baud = args.baud if args.baud is not None else MAV_BAUD

    app = HybridTelemetryGUI(
        master=master,
        baud=baud,
        no_window=args.no_window,
        cam_index=YOLO_CAM_INDEX,
        conf=YOLO_CONF,
        iou=YOLO_IOU,
        det_downscale=DET_DOWNSCALE
    )
    app.run()


if __name__ == '__main__':
    main()