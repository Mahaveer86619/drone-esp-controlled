import cv2
import mediapipe as mp
import numpy as np
import time
import socket
import threading
from dataclasses import dataclass
from typing import Optional, Tuple
import math


@dataclass
class DroneCommand:
    """Represents a drone control command"""

    roll: int = 1500  # 1000-2000, center 1500
    pitch: int = 1500  # 1000-2000, center 1500
    throttle: int = 1000  # 1000-2000, min 1000
    yaw: int = 1500  # 1000-2000, center 1500
    armed: bool = False
    mode: str = "STABILIZE"


class HandGestureController:
    def _init_(self, drone_ip: str = "192.168.159.136", drone_port: int = 14550):
        # MediaPipe hand detection setup - optimized for USB video
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,  # Lowered for USB video
            min_tracking_confidence=0.4,  # Lowered for better tracking
            model_complexity=0,  # Simpler model for faster processing
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Custom drawing specs for better visibility
        self.hand_drawing_spec = self.mp_draw.DrawingSpec(
            color=(0, 255, 0), thickness=3, circle_radius=3
        )
        self.connection_drawing_spec = self.mp_draw.DrawingSpec(
            color=(255, 255, 255), thickness=2, circle_radius=2
        )

        # Drone connection setup
        self.drone_ip = drone_ip
        self.drone_port = drone_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Current drone state
        self.current_command = DroneCommand()
        self.gesture_start_time = {}
        self.last_gesture = None
        self.gesture_confirmed = False

        # Smoothing parameters
        self.smoothing_factor = 0.15
        self.throttle_smoothing = 0.1
        self.deadzone = 0.1

        # Camera setup
        self.camera_index = None
        self.cap = None

        # Control flags
        self.rc_enabled = False
        self.running = True
        self.control_mode = False

        # Gesture detection state
        self.gesture_cooldown = 0
        self.gesture_hold_time = 1.0  # Reduced from 1.5 seconds

        # Overlay configuration
        self.overlay_alpha = 0.8

        # Display configuration
        self.display_width = 1600  # Increased display size
        self.display_height = 900

        # Visual feedback
        self.current_gesture_display = ""
        self.gesture_progress = 0.0

    def find_camera_indices(self, max_tested=5):
        """Find available camera indices"""
        available_cameras = []
        for idx in range(max_tested):
            cap = cv2.VideoCapture(idx, cv2.CAP_ANY)
            if cap.isOpened():
                ret, _ = cap.read()
                cap.release()
                if ret:
                    print(f"[INFO] Found camera at index {idx}")
                    available_cameras.append(idx)

        if not available_cameras:
            raise RuntimeError(f"No usable camera found in indices 0..{max_tested-1}")
        return available_cameras

    def setup_camera(self):
        """Setup camera for video capture - optimized for USB video"""
        available_cameras = self.find_camera_indices()

        print("\nAvailable cameras:")
        for idx in available_cameras:
            print(f"Camera index {idx}")

        while True:
            try:
                selection = int(input("\nSelect camera index: "))
                if selection in available_cameras:
                    self.camera_index = selection
                    break
                print("Invalid selection. Please choose from available cameras.")
            except ValueError:
                print("Please enter a valid number.")

        # Try different backends for better USB support
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        cap = None

        for backend in backends:
            cap = cv2.VideoCapture(self.camera_index, backend)
            if cap.isOpened():
                print(f"Camera opened with backend: {backend}")
                break

        if not cap or not cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {self.camera_index}")

        self.cap = cap

        # Check actual camera capabilities
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        print(f"Camera properties: {actual_width}x{actual_height} @ {actual_fps}fps")

        # Try to set resolution (may not work for all USB devices)
        if actual_width < 640:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Set buffer size to reduce latency
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def send_command(self, command: str):
        """Send command to drone via UDP"""
        try:
            self.sock.sendto(command.encode(), (self.drone_ip, self.drone_port))
            print(f"Sent: {command}")
        except Exception as e:
            print(f"Error sending command: {e}")

    def calculate_hand_metrics(self, hand_landmarks) -> dict:
        """Calculate various hand metrics for gesture recognition"""
        landmarks = []
        for lm in hand_landmarks.landmark:
            landmarks.append([lm.x, lm.y, lm.z])

        landmarks = np.array(landmarks)

        # Calculate finger states
        finger_tips = [4, 8, 12, 16, 20]  # Thumb, index, middle, ring, pinky
        finger_pips = [3, 6, 10, 14, 18]  # PIP joints

        fingers_extended = []

        # Check each finger
        for i, (tip, pip) in enumerate(zip(finger_tips, finger_pips)):
            if i == 0:  # Thumb - check x coordinate
                extended = abs(landmarks[tip][0] - landmarks[pip][0]) > 0.1
            else:  # Other fingers - check y coordinate
                extended = landmarks[tip][1] < landmarks[pip][1]
            fingers_extended.append(extended)

        # Calculate hand center and orientation
        palm_center = np.mean(landmarks[[0, 5, 9, 13, 17]], axis=0)

        # Calculate hand angles
        wrist = landmarks[0]
        middle_base = landmarks[9]
        index_base = landmarks[5]
        pinky_base = landmarks[17]

        # Hand orientation
        hand_vector = middle_base - wrist
        pitch_angle = np.arctan2(-hand_vector[1], hand_vector[2])

        hand_width_vector = pinky_base - index_base
        roll_angle = np.arctan2(hand_width_vector[1], hand_width_vector[0])

        # Palm direction (normal vector)
        v1 = landmarks[5] - landmarks[0]  # wrist to index base
        v2 = landmarks[17] - landmarks[0]  # wrist to pinky base
        palm_normal = np.cross(v1, v2)
        palm_facing_camera = palm_normal[2] < 0

        return {
            "fingers_extended": fingers_extended,
            "palm_center": palm_center,
            "pitch_angle": pitch_angle,
            "roll_angle": roll_angle,
            "num_fingers_extended": sum(fingers_extended),
            "palm_facing_camera": palm_facing_camera,
            "wrist": wrist,
            "landmarks": landmarks,
        }

    def detect_gesture(self, metrics: Optional[dict]) -> str:
        """Detect gesture from hand metrics - optimized for USB video"""
        if not metrics:
            return "NONE"

        fingers = metrics["fingers_extended"]
        num_fingers = metrics["num_fingers_extended"]
        landmarks = metrics["landmarks"]
        wrist = landmarks[0]

        # Calculate specific hand features
        thumb_tip = landmarks[4]
        thumb_base = landmarks[2]
        index_tip = landmarks[8]
        index_pip = landmarks[6]
        middle_tip = landmarks[12]
        ring_tip = landmarks[16]
        pinky_tip = landmarks[20]

        # SIMPLIFIED GESTURE DETECTION FOR USB VIDEO

        # THUMBS UP - ARM (relaxed detection)
        if fingers[0] and num_fingers <= 2 and thumb_tip[1] < thumb_base[1] - 0.05:
            return "ARM"

        # FIST - DISARM (all fingers closed)
        elif num_fingers == 0:
            return "DISARM"

        # STOP HAND - LAND (4-5 fingers extended, more relaxed)
        elif num_fingers >= 4:
            return "LAND"

        # ROCK SIGN - CONTROL (index and pinky clearly extended)
        elif fingers[1] and fingers[4] and num_fingers == 2:
            return "CONTROL"

        # POINTING DOWN - STABILIZE (just index extended and pointing down)
        elif fingers[1] and num_fingers == 1 and index_tip[1] > index_pip[1]:
            return "STABILIZE"

        # L-SHAPE - EMERGENCY (thumb and index making rough L)
        elif fingers[0] and fingers[1] and num_fingers == 2:
            # Check if roughly perpendicular
            thumb_vector = thumb_tip - thumb_base
            index_vector = index_tip - index_pip
            angle = np.arccos(
                np.clip(
                    np.dot(thumb_vector[:2], index_vector[:2])
                    / (
                        np.linalg.norm(thumb_vector[:2])
                        * np.linalg.norm(index_vector[:2])
                    ),
                    -1,
                    1,
                )
            )
            if (
                np.abs(angle - np.pi / 2) < np.pi / 4
            ):  # Within 45 degrees of perpendicular
                return "EMERGENCY"

        # OK SIGN - THROTTLE UP (simplified)
        elif self._is_ok_sign_simple(metrics):
            return "THROTTLE_UP"

        # THREE FINGERS - THROTTLE DOWN (any 3 fingers extended)
        elif num_fingers == 3:
            return "THROTTLE_DOWN"

        return "NONE"

    def _is_ok_sign_simple(self, metrics):
        """Simplified OK sign detection for USB video"""
        landmarks = metrics["landmarks"]
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]

        # Just check if thumb and index are close
        distance = np.sqrt(
            (thumb_tip[0] - index_tip[0]) ** 2 + (thumb_tip[1] - index_tip[1]) ** 2
        )

        return distance < 0.08 and metrics["num_fingers_extended"] >= 3

    def _is_ok_sign(self, metrics):
        """Check if hand is making OK sign"""
        landmarks = metrics["landmarks"]
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]

        # Check if thumb and index tips are close
        distance = np.sqrt(
            (thumb_tip[0] - index_tip[0]) ** 2 + (thumb_tip[1] - index_tip[1]) ** 2
        )

        # Other fingers should be extended
        fingers = metrics["fingers_extended"]
        return distance < 0.05 and fingers[2] and fingers[3] and fingers[4]

    def update_control_from_gesture(self, metrics: Optional[dict]):
        """Update drone control based on hand position in CONTROL mode"""
        if not self.control_mode or not metrics or not self.current_command.armed:
            return

        # Get hand position relative to center
        center_x, center_y = 0.5, 0.5
        hand_x = metrics["palm_center"][0]
        hand_y = metrics["palm_center"][1]

        # Calculate offset from center
        x_offset = (hand_x - center_x) * 2  # Normalize to -1 to 1
        y_offset = (center_y - hand_y) * 2  # Inverted Y axis

        # Apply deadzone
        if abs(x_offset) < self.deadzone:
            x_offset = 0
        if abs(y_offset) < self.deadzone:
            y_offset = 0

        # Map to control values
        # X-axis controls roll and yaw
        roll_input = x_offset * 0.7  # Reduced sensitivity
        yaw_input = x_offset * 0.3

        # Y-axis controls pitch and throttle
        pitch_input = y_offset * 0.7
        throttle_adjustment = y_offset * 0.3

        # Convert to PWM values
        roll_pwm = int(1500 + roll_input * 300)
        pitch_pwm = int(1500 + pitch_input * 300)
        yaw_pwm = int(1500 + yaw_input * 200)

        # Throttle control (maintain altitude with small adjustments)
        if abs(throttle_adjustment) > 0.1:
            throttle_change = throttle_adjustment * 100
            new_throttle = self.current_command.throttle + throttle_change
            self.current_command.throttle = int(max(1200, min(1800, new_throttle)))

        # Apply smoothing
        self.current_command.pitch = int(
            self.current_command.pitch * (1 - self.smoothing_factor)
            + pitch_pwm * self.smoothing_factor
        )
        self.current_command.roll = int(
            self.current_command.roll * (1 - self.smoothing_factor)
            + roll_pwm * self.smoothing_factor
        )
        self.current_command.yaw = int(
            self.current_command.yaw * (1 - self.smoothing_factor)
            + yaw_pwm * self.smoothing_factor
        )

        # Ensure all values are within valid range
        self.current_command.pitch = max(1000, min(2000, self.current_command.pitch))
        self.current_command.roll = max(1000, min(2000, self.current_command.roll))
        self.current_command.yaw = max(1000, min(2000, self.current_command.yaw))

    def draw_enhanced_overlay(
        self, frame: np.ndarray, metrics: Optional[dict], gesture: str
    ) -> np.ndarray:
        """Draw enhanced overlay - adaptive to frame size"""
        height, width = frame.shape[:2]
        overlay = frame.copy()

        # Adaptive sizing based on frame dimensions
        scale = min(width / 1280.0, height / 720.0)
        font_scale = max(0.4, scale * 0.8)
        thickness = max(1, int(scale * 2))

        # Create semi-transparent background panels
        panel_color = (0, 0, 0)
        panel_alpha = 0.7

        # Top status panel - adaptive height
        panel_height = max(40, int(80 * scale))
        cv2.rectangle(overlay, (0, 0), (width, panel_height), panel_color, -1)

        # Status text with adaptive sizing
        status_color = (0, 255, 0) if self.current_command.armed else (0, 0, 255)
        mode_color = (255, 255, 0) if self.control_mode else (255, 255, 255)

        # Adaptive text positioning
        text_y = int(panel_height * 0.6)
        text_spacing = max(150, int(250 * scale))

        cv2.putText(
            overlay,
            f"ARM: {'YES' if self.current_command.armed else 'NO'}",
            (10, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            status_color,
            thickness,
        )
        cv2.putText(
            overlay,
            f"MODE: {self.current_command.mode}",
            (10 + text_spacing, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            thickness,
        )
        cv2.putText(
            overlay,
            f"CTRL: {'ON' if self.control_mode else 'OFF'}",
            (10 + text_spacing * 2, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            mode_color,
            thickness,
        )

        # Flight data - only if armed
        if self.current_command.armed:
            data_start_x = 10 + text_spacing * 3
            cv2.putText(
                overlay,
                f"T:{self.current_command.throttle}",
                (data_start_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale * 0.8,
                (255, 255, 255),
                thickness,
            )
            if width > 800:  # Only show if screen is wide enough
                cv2.putText(
                    overlay,
                    f"P:{self.current_command.pitch}",
                    (data_start_x + 100, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale * 0.8,
                    (255, 255, 255),
                    thickness,
                )
                cv2.putText(
                    overlay,
                    f"R:{self.current_command.roll}",
                    (data_start_x + 200, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale * 0.8,
                    (255, 255, 255),
                    thickness,
                )

        # Current gesture with visual feedback - adaptive sizing
        if gesture != "NONE":
            self.current_gesture_display = gesture

            # Gesture panel - adaptive size
            panel_width = min(400, int(width * 0.6))
            panel_height = max(80, int(120 * scale))
            panel_x = (width - panel_width) // 2
            panel_y = height - panel_height - 10

            cv2.rectangle(
                overlay,
                (panel_x, panel_y),
                (panel_x + panel_width, height - 10),
                panel_color,
                -1,
            )

            # Gesture text
            gesture_color = (0, 255, 255)
            gesture_text = f"{gesture}"
            text_size = cv2.getTextSize(
                gesture_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale * 1.2, thickness
            )[0]
            text_x = panel_x + (panel_width - text_size[0]) // 2
            text_y = panel_y + int(panel_height * 0.4)

            cv2.putText(
                overlay,
                gesture_text,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale * 1.2,
                gesture_color,
                thickness * 2,
            )

            # Progress bar for gesture confirmation
            if gesture in self.gesture_start_time and gesture not in [
                "CONTROL",
                "THROTTLE_UP",
                "THROTTLE_DOWN",
            ]:
                progress = min(
                    1.0,
                    (time.time() - self.gesture_start_time[gesture])
                    / self.gesture_hold_time,
                )
                self.gesture_progress = progress

                # Progress bar
                bar_width = int((panel_width - 40) * progress)
                bar_height = max(15, int(25 * scale))
                bar_x = panel_x + 20
                bar_y = panel_y + int(panel_height * 0.6)

                cv2.rectangle(
                    overlay,
                    (bar_x, bar_y),
                    (bar_x + bar_width, bar_y + bar_height),
                    (0, 255, 0),
                    -1,
                )
                cv2.rectangle(
                    overlay,
                    (bar_x, bar_y),
                    (bar_x + panel_width - 40, bar_y + bar_height),
                    (255, 255, 255),
                    thickness,
                )

                # Progress text
                progress_text = f"{self.gesture_hold_time - (progress * self.gesture_hold_time):.1f}s"
                cv2.putText(
                    overlay,
                    progress_text,
                    (panel_x + panel_width // 2 - 20, bar_y + bar_height - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale * 0.8,
                    (255, 255, 255),
                    thickness,
                )

        # Control mode visualization
        if self.control_mode and metrics:
            # Draw control zone
            control_zone_size = 300
            center_x = width // 2
            center_y = height // 2

            # Outer control zone
            cv2.rectangle(
                overlay,
                (center_x - control_zone_size // 2, center_y - control_zone_size // 2),
                (center_x + control_zone_size // 2, center_y + control_zone_size // 2),
                (0, 255, 0),
                2,
            )

            # Center deadzone
            deadzone_size = int(control_zone_size * self.deadzone)
            cv2.rectangle(
                overlay,
                (center_x - deadzone_size, center_y - deadzone_size),
                (center_x + deadzone_size, center_y + deadzone_size),
                (255, 255, 0),
                1,
            )

            # Crosshair
            cv2.line(
                overlay,
                (center_x - control_zone_size // 2, center_y),
                (center_x + control_zone_size // 2, center_y),
                (0, 255, 0),
                1,
            )
            cv2.line(
                overlay,
                (center_x, center_y - control_zone_size // 2),
                (center_x, center_y + control_zone_size // 2),
                (0, 255, 0),
                1,
            )

            # Hand position indicator
            hand_x = int(metrics["palm_center"][0] * width)
            hand_y = int(metrics["palm_center"][1] * height)
            cv2.circle(overlay, (hand_x, hand_y), 15, (0, 255, 255), -1)
            cv2.circle(overlay, (hand_x, hand_y), 15, (255, 255, 255), 2)

        # Gesture instructions panel - adaptive and compact
        if width > 600:  # Only show if screen is wide enough
            panel_width = min(250, int(width * 0.3))
            instructions_x = width - panel_width - 10
            instructions_y = panel_height + 10

            # Make panel semi-transparent
            instructions_height = min(300, int(height * 0.7))
            cv2.rectangle(
                overlay,
                (instructions_x, instructions_y),
                (width - 10, instructions_y + instructions_height),
                panel_color,
                -1,
            )

            instructions = [
                ("GESTURES:", (255, 255, 0)),
                ("Thumb Up = ARM", (0, 255, 0)),
                ("Fist = DISARM", (0, 0, 255)),
                ("Open Hand = LAND", (255, 165, 0)),
                ("Rock = CONTROL", (255, 255, 0)),
                ("Point Down = STABLE", (0, 255, 255)),
                ("L Shape = STOP", (255, 0, 0)),
                ("OK = THR+", (0, 255, 0)),
                ("3 Fingers = THR-", (255, 0, 255)),
            ]

            line_height = max(25, int(30 * scale))
            for i, (text, color) in enumerate(instructions):
                text_scale = font_scale * (0.9 if i == 0 else 0.7)
                text_thick = thickness + (1 if i == 0 else 0)
                text_y = instructions_y + 25 + i * line_height

                cv2.putText(
                    overlay,
                    text,
                    (instructions_x + 10, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    text_scale,
                    color,
                    text_thick,
                )

        # Blend overlay
        cv2.addWeighted(
            overlay, self.overlay_alpha, frame, 1 - self.overlay_alpha, 0, frame
        )

        # Draw hand skeleton if detected with better visibility
        if metrics and "hand_landmarks" in metrics:
            self.mp_draw.draw_landmarks(
                frame,
                metrics.get("hand_landmarks"),
                self.mp_hands.HAND_CONNECTIONS,
                self.hand_drawing_spec,
                self.connection_drawing_spec,
            )

        return frame

    def process_gesture_action(self, gesture: str):
        """Process detected gesture and execute corresponding action"""
        current_time = time.time()

        # Check for gesture cooldown
        if self.gesture_cooldown > 0:
            self.gesture_cooldown -= 1
            return

        # Initialize gesture timing
        if gesture != "NONE":
            if gesture not in self.gesture_start_time:
                self.gesture_start_time[gesture] = current_time
                print(f"Gesture detected: {gesture}")

            # Check if gesture is held long enough
            hold_time = current_time - self.gesture_start_time[gesture]

            # Instant actions (no hold required)
            if gesture in ["CONTROL", "THROTTLE_UP", "THROTTLE_DOWN"]:
                # Ensure gesture is stable for at least 0.3 seconds
                if hold_time >= 0.3:
                    if gesture == "CONTROL":
                        self.control_mode = not self.control_mode
                        print(
                            f"Control mode: {'ACTIVE' if self.control_mode else 'INACTIVE'}"
                        )
                    elif gesture == "THROTTLE_UP" and self.current_command.armed:
                        self.current_command.throttle = min(
                            1800, self.current_command.throttle + 50
                        )
                        print(f"Throttle increased to: {self.current_command.throttle}")
                    elif gesture == "THROTTLE_DOWN" and self.current_command.armed:
                        self.current_command.throttle = max(
                            1200, self.current_command.throttle - 50
                        )
                        print(f"Throttle decreased to: {self.current_command.throttle}")

                    self.gesture_cooldown = 20  # Short cooldown
                    self.gesture_start_time.pop(gesture, None)

            # Hold-to-confirm actions
            elif hold_time >= self.gesture_hold_time:
                if gesture == "ARM" and not self.current_command.armed:
                    self.arm_drone()
                elif gesture == "DISARM" and self.current_command.armed:
                    self.disarm_drone()
                elif gesture == "LAND":
                    self.land_drone()
                elif gesture == "STABILIZE":
                    self.set_stabilize_mode()
                elif gesture == "EMERGENCY":
                    self.emergency_stop()

                self.gesture_cooldown = 40  # Longer cooldown
                self.gesture_start_time.pop(gesture, None)
        else:
            # Reset all gesture timings if no gesture detected
            self.gesture_start_time.clear()
            self.gesture_progress = 0.0

        self.last_gesture = gesture

    def arm_drone(self):
        """Arm the drone"""
        self.send_command("ARM")
        self.current_command.armed = True
        self.current_command.throttle = 1000  # Start at minimum
        print("DRONE ARMED!")

    def disarm_drone(self):
        """Disarm the drone"""
        self.send_command("DISARM")
        self.current_command.armed = False
        self.current_command.throttle = 1000
        self.control_mode = False
        print("DRONE DISARMED!")

    def land_drone(self):
        """Land the drone"""
        self.send_command("MODE_LAND")
        self.current_command.mode = "LAND"
        self.control_mode = False
        print("LANDING DRONE!")

    def set_stabilize_mode(self):
        """Set stabilize mode"""
        self.send_command("MODE_STABILIZE")
        self.current_command.mode = "STABILIZE"
        print("STABILIZE MODE ACTIVATED!")

    def emergency_stop(self):
        """Emergency stop"""
        self.current_command.throttle = 1000
        self.control_mode = False
        self.send_command("DISARM")
        self.current_command.armed = False
        print("EMERGENCY STOP ACTIVATED!")

    def send_rc_channels(self):
        """Send RC channel values to drone"""
        while self.running and self.rc_enabled:
            cmd = (
                f"RC_CHANNELS:{self.current_command.roll},{self.current_command.pitch},"
            )
            cmd += f"{self.current_command.throttle},{self.current_command.yaw},"
            cmd += "1500,1500,1500,1500"  # Aux channels
            self.send_command(cmd)
            time.sleep(0.05)  # 20Hz update rate

    def run(self):
        """Main control loop with USB video optimizations"""
        self.setup_camera()

        # Start RC sender thread
        rc_thread = threading.Thread(target=self.send_rc_channels, daemon=True)
        rc_thread.start()

        # Enable RC control
        self.send_command("START_OVERRIDE")
        self.rc_enabled = True

        print("[INFO] Hand gesture control started. Press ESC to exit.")

        # Create window - adaptive to actual frame size
        cv2.namedWindow("Hand Gesture Drone Control", cv2.WINDOW_NORMAL)

        # Frame preprocessing for USB video
        frame_count = 0
        gesture_history = []

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("[ERROR] Frame grab failed")
                continue  # Try again instead of breaking

            frame_count += 1

            # Get actual frame dimensions
            height, width = frame.shape[:2]

            # Skip every other frame for USB video to improve performance
            if frame_count % 2 == 0 and width < 800:
                continue

            # Preprocessing for USB video
            if width < 640 or height < 480:
                # Upscale low resolution video
                frame = cv2.resize(frame, (640, 480))

            # Enhance contrast for better hand detection
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            l = clahe.apply(l)
            lab = cv2.merge((l, a, b))
            frame = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)

            # Convert to RGB for MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Process hand detection
            results = self.hands.process(rgb_frame)

            metrics = None

            if results.multi_hand_landmarks:
                # Use first detected hand
                hand_landmarks = results.multi_hand_landmarks[0]

                # Calculate hand metrics
                metrics = self.calculate_hand_metrics(hand_landmarks)
                metrics["hand_landmarks"] = hand_landmarks

            # Detect gesture
            current_gesture = self.detect_gesture(metrics)

            # Gesture stability tracking (avoid flickering)
            gesture_history.append(current_gesture)
            if len(gesture_history) > 5:
                gesture_history.pop(0)

            # Use most common gesture from history
            if gesture_history:
                from collections import Counter

                gesture = Counter(gesture_history).most_common(1)[0][0]
            else:
                gesture = "NONE"

            # Process gesture action
            self.process_gesture_action(gesture)

            # Update control if in CONTROL mode
            if self.control_mode and self.current_command.armed:
                self.update_control_from_gesture(metrics)

            # Draw enhanced overlay
            frame = self.draw_enhanced_overlay(frame, metrics, gesture)

            # Resize window based on frame size
            window_width = max(width, 800)
            window_height = int(window_width * height / width)
            cv2.resizeWindow("Hand Gesture Drone Control", window_width, window_height)

            # Display frame
            cv2.imshow("Hand Gesture Drone Control", frame)

            # Check for exit
            key = cv2.waitKey(1)
            if key == 27:  # ESC key
                break
            elif key == ord("r"):  # Reset
                self.disarm_drone()
                self.control_mode = False

        # Cleanup
        self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        self.running = False
        self.rc_enabled = False

        # Send stop override command
        self.send_command("STOP_OVERRIDE")

        # Close connections
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        self.sock.close()

        print("[INFO] Cleanup complete")


if _name_ == "_main_":
    controller = HandGestureController()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        controller.cleanup()
