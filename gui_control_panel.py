import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import time
import math


class RCController:
    def _init_(self, master):
        self.master = master
        self.master.title("Advanced RC Controller with Failsafe")
        self.master.geometry("1200x900")

        # Connection settings
        self.DRONE_IP = "192.168.159.136"
        self.DRONE_PORT = 14550

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)

        # RC channel values (1000-2000)
        self.channels = {
            "roll": 1500,  # CH1
            "pitch": 1500,  # CH2
            "throttle": 1000,  # CH3 - Start at minimum
            "yaw": 1500,  # CH4
            "ch5": 1500,  # Aux 1 (flight mode)
            "ch6": 1500,  # Aux 2
            "ch7": 1500,  # Aux 3
            "ch8": 1500,  # Aux 4
        }

        # Control settings
        self.rc_active = False
        self.listener_running = True
        self.current_status = {}
        self.armed = False
        self.dragging = False

        # Trim values
        self.trims = {"roll": 0, "pitch": 0, "yaw": 0}

        # Expo and rates
        self.expo = 0.3
        self.rate = 1.0

        # Throttle control settings
        self.throttle_scale = 0.5  # 50% scaling by default
        self.throttle_expo = 0.3  # 30% expo curve
        self.throttle_deadband = 30  # 30 PWM units deadband
        self.hover_throttle = 1400  # Changed from 1500 to 1400 (lower hover point)

        # Flight modes
        self.flight_modes = {
            "STABILIZE": 0,
            "ALT_HOLD": 2,
            "LOITER": 5,
            "RTL": 6,
            "LAND": 9,
            "POSHOLD": 16,
            "GUIDED": 4,
        }

        # Signal monitoring
        self.last_response_time = time.time()
        self.signal_lost = False
        self.failsafe_active = False
        self.signal_timeout = 2.0  # 2 seconds for GUI warning

        self.create_widgets()
        self.start_threads()
        self.monitor_signal()

    def create_widgets(self):
        # Main container with tabs
        notebook = ttk.Notebook(self.master)
        notebook.pack(fill="both", expand=True, padx=10, pady=10)

        # Flight control tab
        flight_tab = ttk.Frame(notebook)
        notebook.add(flight_tab, text="Flight Control")
        self.create_flight_controls(flight_tab)

        # Configuration tab
        config_tab = ttk.Frame(notebook)
        notebook.add(config_tab, text="Configuration")
        self.create_config_controls(config_tab)

        # Calibration tab
        calib_tab = ttk.Frame(notebook)
        notebook.add(calib_tab, text="Calibration")
        self.create_calibration_controls(calib_tab)

        # Status tab
        status_tab = ttk.Frame(notebook)
        notebook.add(status_tab, text="Status & Log")
        self.create_status_display(status_tab)

    def create_flight_controls(self, parent):
        # Top status bar
        status_frame = tk.Frame(parent, bg="black", height=60)
        status_frame.pack(fill="x", padx=5, pady=5)
        status_frame.pack_propagate(False)

        # Status indicators
        self.arm_indicator = tk.Label(
            status_frame,
            text="DISARMED",
            fg="red",
            bg="black",
            font=("Arial", 16, "bold"),
        )
        self.arm_indicator.pack(side="left", padx=20)

        self.mode_indicator = tk.Label(
            status_frame,
            text="Mode: Unknown",
            fg="yellow",
            bg="black",
            font=("Arial", 16, "bold"),
        )
        self.mode_indicator.pack(side="left", padx=20)

        self.battery_indicator = tk.Label(
            status_frame,
            text="Battery: --%",
            fg="green",
            bg="black",
            font=("Arial", 16, "bold"),
        )
        self.battery_indicator.pack(side="left", padx=20)

        self.signal_indicator = tk.Label(
            status_frame,
            text="Signal: --",
            fg="cyan",
            bg="black",
            font=("Arial", 16, "bold"),
        )
        self.signal_indicator.pack(side="left", padx=20)

        # Failsafe indicator
        self.failsafe_indicator = tk.Label(
            status_frame, text="", fg="red", bg="black", font=("Arial", 16, "bold")
        )
        self.failsafe_indicator.pack(side="right", padx=20)

        # Countdown timer
        self.countdown_label = tk.Label(
            status_frame, text="", fg="yellow", bg="black", font=("Arial", 14, "bold")
        )
        self.countdown_label.pack(side="right", padx=10)

        # Main control area
        control_frame = tk.Frame(parent)
        control_frame.pack(fill="both", expand=True, padx=10, pady=10)

        # Left stick (throttle/yaw)
        left_frame = tk.LabelFrame(control_frame, text="Throttle/Yaw", padx=20, pady=20)
        left_frame.grid(row=0, column=0, padx=20, pady=20)

        self.left_stick = self.create_joystick(left_frame, "left")

        # Center controls
        center_frame = tk.Frame(control_frame)
        center_frame.grid(row=0, column=1, padx=20, pady=20)

        # Arm/Disarm buttons
        arm_frame = tk.Frame(center_frame)
        arm_frame.pack(pady=10)

        self.arm_btn = tk.Button(
            arm_frame,
            text="ARM",
            command=self.arm_drone,
            width=15,
            height=3,
            bg="green",
            font=("Arial", 14, "bold"),
        )
        self.arm_btn.pack(side="left", padx=5)

        self.disarm_btn = tk.Button(
            arm_frame,
            text="DISARM",
            command=self.disarm_drone,
            width=15,
            height=3,
            bg="red",
            font=("Arial", 14, "bold"),
        )
        self.disarm_btn.pack(side="left", padx=5)

        # Flight mode selector
        mode_frame = tk.LabelFrame(center_frame, text="Flight Mode", padx=10, pady=10)
        mode_frame.pack(pady=10)

        self.mode_var = tk.StringVar(value="STABILIZE")
        mode_dropdown = ttk.Combobox(
            mode_frame,
            textvariable=self.mode_var,
            values=list(self.flight_modes.keys()),
            state="readonly",
            width=20,
            font=("Arial", 12),
        )
        mode_dropdown.pack(pady=5)
        mode_dropdown.bind("<<ComboboxSelected>>", self.change_flight_mode)

        # Emergency buttons
        emergency_frame = tk.Frame(center_frame)
        emergency_frame.pack(pady=20)

        tk.Button(
            emergency_frame,
            text="EMERGENCY\nSTOP",
            command=self.emergency_stop,
            width=12,
            height=3,
            bg="red",
            fg="white",
            font=("Arial", 12, "bold"),
        ).pack(side="left", padx=5)

        tk.Button(
            emergency_frame,
            text="RTL",
            command=self.return_to_launch,
            width=12,
            height=3,
            bg="orange",
            fg="white",
            font=("Arial", 12, "bold"),
        ).pack(side="left", padx=5)

        tk.Button(
            emergency_frame,
            text="LAND",
            command=self.land,
            width=12,
            height=3,
            bg="yellow",
            fg="black",
            font=("Arial", 12, "bold"),
        ).pack(side="left", padx=5)

        # Right stick (pitch/roll)
        right_frame = tk.LabelFrame(control_frame, text="Pitch/Roll", padx=20, pady=20)
        right_frame.grid(row=0, column=2, padx=20, pady=20)

        self.right_stick = self.create_joystick(right_frame, "right")

        # Bottom controls
        bottom_frame = tk.Frame(parent)
        bottom_frame.pack(fill="x", padx=10, pady=10)

        # RC enable/disable
        rc_frame = tk.Frame(bottom_frame)
        rc_frame.pack(side="left", padx=20)

        tk.Button(
            rc_frame,
            text="Enable RC",
            command=self.enable_rc,
            width=15,
            height=2,
            bg="green",
            font=("Arial", 12),
        ).pack(side="left", padx=5)

        tk.Button(
            rc_frame,
            text="Disable RC",
            command=self.disable_rc,
            width=15,
            height=2,
            bg="red",
            font=("Arial", 12),
        ).pack(side="left", padx=5)

        # Throttle mode buttons
        throttle_mode_frame = tk.Frame(bottom_frame)
        throttle_mode_frame.pack(side="left", padx=20)

        tk.Label(throttle_mode_frame, text="Throttle Mode:", font=("Arial", 12)).pack()
        tk.Button(
            throttle_mode_frame,
            text="SMOOTH",
            command=self.set_throttle_smooth,
            width=10,
            height=1,
            bg="lightblue",
            font=("Arial", 10),
        ).pack(side="left", padx=2)
        tk.Button(
            throttle_mode_frame,
            text="NORMAL",
            command=self.set_throttle_normal,
            width=10,
            height=1,
            bg="lightgreen",
            font=("Arial", 10),
        ).pack(side="left", padx=2)
        tk.Button(
            throttle_mode_frame,
            text="SPORT",
            command=self.set_throttle_sport,
            width=10,
            height=1,
            bg="orange",
            font=("Arial", 10),
        ).pack(side="left", padx=2)

        # Aux channels
        aux_frame = tk.LabelFrame(
            bottom_frame, text="Auxiliary Channels", padx=10, pady=10
        )
        aux_frame.pack(side="right", padx=20)

        for i in range(5, 9):
            ch_frame = tk.Frame(aux_frame)
            ch_frame.pack(side="left", padx=10)
            tk.Label(ch_frame, text=f"CH{i}").pack()
            scale = tk.Scale(
                ch_frame,
                from_=2000,
                to=1000,
                orient="vertical",
                command=lambda v, ch=f"ch{i}": self.update_channel(ch, v),
            )
            scale.set(1500)
            scale.pack()
            setattr(self, f"ch{i}_scale", scale)

    def create_joystick(self, parent, stick_type):
        canvas = tk.Canvas(
            parent,
            width=300,
            height=300,
            bg="black",
            highlightthickness=2,
            highlightbackground="gray",
        )
        canvas.pack()

        # Draw crosshairs
        canvas.create_line(0, 150, 300, 150, fill="green", width=2)
        canvas.create_line(150, 0, 150, 300, fill="green", width=2)

        # Draw circle
        canvas.create_oval(50, 50, 250, 250, outline="gray", width=2)

        # Create stick indicator
        if stick_type == "left":
            # Start throttle at bottom (1000 PWM)
            stick = canvas.create_oval(
                140, 240, 160, 260, fill="red", outline="white", width=2
            )
        else:
            # Right stick starts at center
            stick = canvas.create_oval(
                140, 140, 160, 160, fill="red", outline="white", width=2
            )

        # Bind mouse events
        canvas.bind(
            "<Button-1>", lambda e: self.start_stick_drag(e, canvas, stick, stick_type)
        )
        canvas.bind(
            "<B1-Motion>", lambda e: self.drag_stick(e, canvas, stick, stick_type)
        )
        canvas.bind(
            "<ButtonRelease-1>",
            lambda e: self.release_stick(e, canvas, stick, stick_type),
        )

        return {"canvas": canvas, "stick": stick}

    def start_stick_drag(self, event, canvas, stick, stick_type):
        self.dragging = True
        self.drag_stick(event, canvas, stick, stick_type)

    def drag_stick(self, event, canvas, stick, stick_type):
        if not self.dragging:
            return

        # Constrain to circle
        cx, cy = 150, 150
        dx, dy = event.x - cx, event.y - cy
        distance = math.sqrt(dx * 2 + dy * 2)

        if distance > 100:
            dx = dx * 100 / distance
            dy = dy * 100 / distance

        # Update stick position
        x, y = cx + dx, cy + dy
        canvas.coords(stick, x - 10, y - 10, x + 10, y + 10)

        # Update channel values
        if stick_type == "left":
            # MODIFIED THROTTLE CONTROL - Bottom is 1000, Top is 2000
            # Map from -100 to +100 stick position to 1000-2000 PWM
            stick_range = 200  # Total movement range (-100 to +100)
            throttle_range = 1000  # PWM range (1000 to 2000)

            # Calculate throttle (inverted - bottom is 1000, top is 2000)
            normalized_position = (100 - dy) / stick_range  # 0 at bottom, 1 at top
            raw_throttle = 1000 + (normalized_position * throttle_range)

            # Apply expo curve
            expo_position = normalized_position
            if self.throttle_expo > 0:
                expo_position = (
                    normalized_position * (1 - self.throttle_expo)
                    + (normalized_position**3) * self.throttle_expo
                )

            # Apply scaling
            scaled_range = throttle_range * self.throttle_scale
            throttle = 1000 + (expo_position * scaled_range)

            # Apply deadband at bottom
            if throttle < (1000 + self.throttle_deadband):
                throttle = 1000  # Stick to minimum

            throttle = int(max(1000, min(2000, throttle)))
            self.channels["throttle"] = throttle

            # Update test display
            if hasattr(self, "throttle_test_label"):
                self.throttle_test_label.config(text=f"Output: {throttle} PWM")

            # Yaw
            yaw = int(1500 + dx * 5)
            yaw = max(1000, min(2000, yaw))
            self.channels["yaw"] = yaw + self.trims["yaw"]

        else:  # right stick
            # Pitch (inverted - forward is down)
            pitch = int(1500 - dy * 5)
            pitch = max(1000, min(2000, pitch))
            self.channels["pitch"] = pitch + self.trims["pitch"]

            # Roll
            roll = int(1500 + dx * 5)
            roll = max(1000, min(2000, roll))
            self.channels["roll"] = roll + self.trims["roll"]

    def release_stick(self, event, canvas, stick, stick_type):
        self.dragging = False

        if stick_type == "right":
            # Spring back to center for right stick
            canvas.coords(stick, 140, 140, 160, 160)
            self.channels["pitch"] = 1500 + self.trims["pitch"]
            self.channels["roll"] = 1500 + self.trims["roll"]
        elif stick_type == "left":
            # For left stick, only spring back yaw to center
            # Keep throttle at current position
            current_throttle = self.channels["throttle"]

            # Calculate stick position based on current throttle
            # Map 1000-2000 to stick position (250 to 50)
            throttle_normalized = (current_throttle - 1000) / 1000.0
            stick_y = 250 - (throttle_normalized * 200)  # Bottom=250, Top=50

            # Ensure stick_y is within bounds
            stick_y = max(50, min(250, stick_y))

            # Position stick at current throttle, center yaw
            canvas.coords(stick, 140, stick_y - 10, 160, stick_y + 10)
            self.channels["yaw"] = 1500 + self.trims["yaw"]

    def create_config_controls(self, parent):
        # Trim settings
        trim_frame = tk.LabelFrame(parent, text="Trim Settings", padx=20, pady=20)
        trim_frame.pack(fill="x", padx=20, pady=20)

        for i, axis in enumerate(["roll", "pitch", "yaw"]):
            frame = tk.Frame(trim_frame)
            frame.grid(row=0, column=i, padx=20)

            tk.Label(frame, text=f"{axis.title()} Trim", font=("Arial", 12)).pack()

            scale = tk.Scale(
                frame,
                from_=50,
                to=-50,
                orient="vertical",
                command=lambda v, a=axis: self.update_trim(a, int(v)),
            )
            scale.set(0)
            scale.pack()

            setattr(self, f"{axis}_trim", scale)

        # Throttle Settings Frame
        throttle_frame = tk.LabelFrame(
            parent, text="Throttle Settings", padx=20, pady=20
        )
        throttle_frame.pack(fill="x", padx=20, pady=20)

        # Throttle scaling
        tk.Label(throttle_frame, text="Throttle Scale:", font=("Arial", 12)).grid(
            row=0, column=0, padx=10
        )
        self.throttle_scale_var = tk.DoubleVar(value=self.throttle_scale)
        self.throttle_scale_slider = tk.Scale(
            throttle_frame,
            from_=0.1,
            to=1.0,
            resolution=0.05,
            orient="horizontal",
            variable=self.throttle_scale_var,
            command=self.update_throttle_scale,
        )
        self.throttle_scale_slider.grid(row=0, column=1, padx=10)
        tk.Label(
            throttle_frame, text="Lower = Slower response", font=("Arial", 10)
        ).grid(row=0, column=2, padx=10)

        # Throttle expo
        tk.Label(throttle_frame, text="Throttle Expo:", font=("Arial", 12)).grid(
            row=1, column=0, padx=10
        )
        self.throttle_expo_var = tk.DoubleVar(value=self.throttle_expo)
        self.throttle_expo_slider = tk.Scale(
            throttle_frame,
            from_=0.0,
            to=0.8,
            resolution=0.05,
            orient="horizontal",
            variable=self.throttle_expo_var,
            command=self.update_throttle_expo,
        )
        self.throttle_expo_slider.grid(row=1, column=1, padx=10)
        tk.Label(
            throttle_frame, text="Higher = More expo curve", font=("Arial", 10)
        ).grid(row=1, column=2, padx=10)

        # Throttle deadband
        tk.Label(throttle_frame, text="Throttle Deadband:", font=("Arial", 12)).grid(
            row=2, column=0, padx=10
        )
        self.throttle_deadband_var = tk.IntVar(value=self.throttle_deadband)
        self.throttle_deadband_slider = tk.Scale(
            throttle_frame,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.throttle_deadband_var,
            command=self.update_throttle_deadband,
        )
        self.throttle_deadband_slider.grid(row=2, column=1, padx=10)
        tk.Label(
            throttle_frame, text="Stick center dead zone", font=("Arial", 10)
        ).grid(row=2, column=2, padx=10)

        # Hover throttle
        tk.Label(throttle_frame, text="Hover Throttle:", font=("Arial", 12)).grid(
            row=3, column=0, padx=10
        )
        self.hover_throttle_var = tk.IntVar(value=self.hover_throttle)
        self.hover_throttle_slider = tk.Scale(
            throttle_frame,
            from_=1200,
            to=1600,  # Changed range
            orient="horizontal",
            variable=self.hover_throttle_var,
            command=self.update_hover_throttle,
        )
        self.hover_throttle_slider.grid(row=3, column=1, padx=10)
        tk.Label(throttle_frame, text="Throttle for hover", font=("Arial", 10)).grid(
            row=3, column=2, padx=10
        )

        # Test output display
        self.throttle_test_label = tk.Label(
            throttle_frame, text="Output: ----", font=("Arial", 12, "bold")
        )
        self.throttle_test_label.grid(row=4, column=0, columnspan=3, pady=10)

        # Expo and rates
        expo_frame = tk.LabelFrame(
            parent, text="Control Expo & Rates", padx=20, pady=20
        )
        expo_frame.pack(fill="x", padx=20, pady=20)

        tk.Label(expo_frame, text="Control Expo:", font=("Arial", 12)).grid(
            row=0, column=0, padx=10
        )
        self.expo_scale = tk.Scale(
            expo_frame,
            from_=0,
            to=1,
            resolution=0.1,
            orient="horizontal",
            command=lambda v: setattr(self, "expo", float(v)),
        )
        self.expo_scale.set(self.expo)
        self.expo_scale.grid(row=0, column=1, padx=10)

        tk.Label(expo_frame, text="Control Rate:", font=("Arial", 12)).grid(
            row=1, column=0, padx=10
        )
        self.rate_scale = tk.Scale(
            expo_frame,
            from_=0.5,
            to=2.0,
            resolution=0.1,
            orient="horizontal",
            command=lambda v: setattr(self, "rate", float(v)),
        )
        self.rate_scale.set(self.rate)
        self.rate_scale.grid(row=1, column=1, padx=10)

        # Failsafe Information
        failsafe_frame = tk.LabelFrame(parent, text="Failsafe System", padx=20, pady=20)
        failsafe_frame.pack(fill="x", padx=20, pady=20)

        info_text = """FAILSAFE BEHAVIOR:
- If no commands received for 1 second → LAND immediately
- If WiFi disconnects for 3 seconds → LAND immediately
- During failsafe, drone will:
  - Switch to LAND mode
  - Descend at current position
  - Ignore all RC commands
  - Auto-disarm on ground

WARNINGS:
- Always maintain good signal strength
- Stay within WiFi range
- Monitor battery levels
- Keep landing area clear
- Test failsafe before flight"""

        info_label = tk.Label(
            failsafe_frame,
            text=info_text,
            justify="left",
            font=("Courier", 10),
            bg="lightyellow",
            padx=10,
            pady=10,
        )
        info_label.pack(fill="x")

        # Test failsafe button
        test_frame = tk.Frame(failsafe_frame)
        test_frame.pack(pady=10)

        tk.Button(
            test_frame,
            text="Test Failsafe",
            command=self.test_failsafe,
            width=15,
            height=2,
            bg="orange",
            font=("Arial", 12),
        ).pack(side="left", padx=5)

        tk.Button(
            test_frame,
            text="Check Status",
            command=self.check_failsafe_status,
            width=15,
            height=2,
            bg="blue",
            font=("Arial", 12),
        ).pack(side="left", padx=5)

        # Connection settings
        conn_frame = tk.LabelFrame(parent, text="Connection Settings", padx=20, pady=20)
        conn_frame.pack(fill="x", padx=20, pady=20)

        tk.Label(conn_frame, text="IP Address:", font=("Arial", 12)).grid(
            row=0, column=0, padx=10
        )
        self.ip_entry = tk.Entry(conn_frame, width=20, font=("Arial", 12))
        self.ip_entry.insert(0, self.DRONE_IP)
        self.ip_entry.grid(row=0, column=1, padx=10)

        tk.Label(conn_frame, text="Port:", font=("Arial", 12)).grid(
            row=1, column=0, padx=10
        )
        self.port_entry = tk.Entry(conn_frame, width=20, font=("Arial", 12))
        self.port_entry.insert(0, str(self.DRONE_PORT))
        self.port_entry.grid(row=1, column=1, padx=10)

        tk.Button(
            conn_frame,
            text="Update Connection",
            command=self.update_connection,
            width=20,
            height=2,
            bg="blue",
            fg="white",
            font=("Arial", 12),
        ).grid(row=2, column=0, columnspan=2, pady=10)

    def create_calibration_controls(self, parent):
        # ESC Calibration
        esc_frame = tk.LabelFrame(parent, text="ESC Calibration", padx=20, pady=20)
        esc_frame.pack(fill="x", padx=20, pady=20)

        warning_label = tk.Label(
            esc_frame,
            text="⚠ REMOVE PROPELLERS BEFORE CALIBRATION ⚠",
            fg="red",
            font=("Arial", 14, "bold"),
        )
        warning_label.pack(pady=10)

        instructions = tk.Label(
            esc_frame,
            text="""
ESC Calibration Procedure:
1. Remove all propellers
2. Ensure battery is connected
3. Click 'Start ESC Calibration'
4. Listen for ESC confirmation beeps
5. Wait for completion message
        """,
            justify="left",
            font=("Arial", 11),
        )
        instructions.pack(pady=10)

        tk.Button(
            esc_frame,
            text="Start ESC Calibration",
            command=self.start_esc_calibration,
            width=20,
            height=2,
            bg="orange",
            font=("Arial", 12),
        ).pack(pady=10)

        # Motor Test
        motor_frame = tk.LabelFrame(parent, text="Motor Test", padx=20, pady=20)
        motor_frame.pack(fill="x", padx=20, pady=20)

        tk.Label(
            motor_frame,
            text="⚠ REMOVE PROPELLERS BEFORE TESTING ⚠",
            fg="red",
            font=("Arial", 14, "bold"),
        ).pack(pady=10)

        # Motor selection
        motor_select_frame = tk.Frame(motor_frame)
        motor_select_frame.pack(pady=10)

        tk.Label(motor_select_frame, text="Motor:", font=("Arial", 12)).pack(
            side="left", padx=5
        )
        self.motor_var = tk.StringVar(value="1")
        motor_dropdown = ttk.Combobox(
            motor_select_frame,
            textvariable=self.motor_var,
            values=["1", "2", "3", "4", "5", "6", "7", "8"],
            state="readonly",
            width=5,
        )
        motor_dropdown.pack(side="left", padx=5)

        # Throttle selection
        tk.Label(motor_select_frame, text="Throttle %:", font=("Arial", 12)).pack(
            side="left", padx=5
        )
        self.test_throttle_var = tk.StringVar(value="10")
        throttle_dropdown = ttk.Combobox(
            motor_select_frame,
            textvariable=self.test_throttle_var,
            values=["5", "10", "15", "20", "25", "30"],
            state="readonly",
            width=5,
        )
        throttle_dropdown.pack(side="left", padx=5)

        # Duration selection
        tk.Label(motor_select_frame, text="Duration (ms):", font=("Arial", 12)).pack(
            side="left", padx=5
        )
        self.test_duration_var = tk.StringVar(value="2000")
        duration_dropdown = ttk.Combobox(
            motor_select_frame,
            textvariable=self.test_duration_var,
            values=["1000", "2000", "3000", "5000"],
            state="readonly",
            width=8,
        )
        duration_dropdown.pack(side="left", padx=5)

        tk.Button(
            motor_frame,
            text="Test Motor",
            command=self.test_motor,
            width=20,
            height=2,
            bg="blue",
            font=("Arial", 12),
        ).pack(pady=10)

        # Motor diagram
        diagram_frame = tk.Frame(motor_frame)
        diagram_frame.pack(pady=20)

        canvas = tk.Canvas(diagram_frame, width=300, height=300, bg="white")
        canvas.pack()

        # Draw quadcopter frame
        canvas.create_line(50, 150, 250, 150, width=3)  # horizontal
        canvas.create_line(150, 50, 150, 250, width=3)  # vertical

        # Draw motors and labels
        motors = [
            (50, 50, "1"),  # Front right
            (250, 50, "2"),  # Front left
            (250, 250, "3"),  # Rear left
            (50, 250, "4"),  # Rear right
        ]

        for x, y, label in motors:
            # Motor circle
            # Motor circle
            canvas.create_oval(
                x - 20, y - 20, x + 20, y + 20, fill="lightblue", width=2
            )
            # Motor number
            canvas.create_text(x, y, text=label, font=("Arial", 16, "bold"))
            # Rotation direction
            if label in ["1", "3"]:  # Clockwise
                canvas.create_arc(
                    x - 25,
                    y - 25,
                    x + 25,
                    y + 25,
                    start=0,
                    extent=270,
                    style="arc",
                    width=2,
                    outline="green",
                )
                canvas.create_polygon(
                    x + 20, y - 5, x + 25, y, x + 15, y + 5, fill="green"
                )
            else:  # Counter-clockwise
                canvas.create_arc(
                    x - 25,
                    y - 25,
                    x + 25,
                    y + 25,
                    start=90,
                    extent=270,
                    style="arc",
                    width=2,
                    outline="red",
                )
                canvas.create_polygon(
                    x - 20, y + 5, x - 25, y, x - 15, y - 5, fill="red"
                )

        # Add front arrow
        canvas.create_polygon(150, 20, 140, 35, 160, 35, fill="black")
        canvas.create_text(150, 10, text="FRONT", font=("Arial", 12, "bold"))

        # Calibration status
        status_frame = tk.LabelFrame(
            parent, text="Calibration Status", padx=20, pady=20
        )
        status_frame.pack(fill="x", padx=20, pady=20)

        self.calib_status = tk.Text(status_frame, height=5, font=("Courier", 10))
        self.calib_status.pack(fill="x")

    def create_status_display(self, parent):
        # Status display
        status_frame = tk.LabelFrame(parent, text="System Status", padx=20, pady=20)
        status_frame.pack(fill="x", padx=20, pady=20)

        self.status_text = tk.Text(status_frame, height=10, font=("Courier", 10))
        self.status_text.pack(fill="x")

        # Log display
        log_frame = tk.LabelFrame(parent, text="System Log", padx=20, pady=20)
        log_frame.pack(fill="both", expand=True, padx=20, pady=20)

        self.log_text = tk.Text(log_frame, height=20, font=("Courier", 10))
        self.log_text.pack(fill="both", expand=True)

        # Clear log button
        tk.Button(
            log_frame,
            text="Clear Log",
            command=self.clear_log,
            width=15,
            height=2,
            bg="gray",
            font=("Arial", 12),
        ).pack(pady=10)

    # Control methods
    def update_throttle_scale(self, value):
        self.throttle_scale = float(value)

    def update_throttle_expo(self, value):
        self.throttle_expo = float(value)

    def update_throttle_deadband(self, value):
        self.throttle_deadband = int(value)

    def update_hover_throttle(self, value):
        self.hover_throttle = int(value)

    def update_channel(self, channel, value):
        self.channels[channel] = int(value)

    def update_trim(self, axis, value):
        self.trims[axis] = value

    def send_command(self, command):
        try:
            self.sock.sendto(command.encode(), (self.DRONE_IP, self.DRONE_PORT))
            self.log(f"Sent: {command}")
        except Exception as e:
            self.log(f"Error: {e}")

    def enable_rc(self):
        self.rc_active = True
        self.send_command("START_OVERRIDE")
        self.log("RC Control Enabled")

    def disable_rc(self):
        self.rc_active = False
        self.send_command("STOP_OVERRIDE")
        self.log("RC Control Disabled")

    def arm_drone(self):
        # Safety check - only arm if throttle is low
        if self.channels["throttle"] > 1100:
            messagebox.showwarning(
                "Safety Warning", "Lower throttle to minimum before arming!"
            )
            return

        self.send_command("ARM")
        self.log("Arming drone...")

    def disarm_drone(self):
        self.send_command("DISARM")
        self.log("Disarming drone...")

    def emergency_stop(self):
        self.channels["throttle"] = 1000
        self.disable_rc()
        self.send_command("DISARM")
        self.log("EMERGENCY STOP ACTIVATED!")

    def return_to_launch(self):
        self.mode_var.set("RTL")
        self.change_flight_mode(None)

    def land(self):
        self.mode_var.set("LAND")
        self.change_flight_mode(None)

    def change_flight_mode(self, event):
        mode = self.mode_var.get()
        if mode == "STABILIZE":
            self.send_command("MODE_STABILIZE")
        else:
            self.send_command(f"MODE_{mode}")
        self.log(f"Changing flight mode to {mode}")

    def set_throttle_smooth(self):
        self.send_command("THROTTLE_MODE:SMOOTH")
        self.log("Throttle mode set to SMOOTH")

    def set_throttle_normal(self):
        self.send_command("THROTTLE_MODE:NORMAL")
        self.log("Throttle mode set to NORMAL")

    def set_throttle_sport(self):
        self.send_command("THROTTLE_MODE:SPORT")
        self.log("Throttle mode set to SPORT")

    def start_esc_calibration(self):
        response = messagebox.askyesno(
            "ESC Calibration",
            "Have you removed all propellers?\n\n"
            "ESC calibration will spin motors at high speed!",
        )
        if response:
            self.send_command("START_ESC_CAL")
            self.calib_status.insert(tk.END, "Starting ESC calibration...\n")
            self.calib_status.insert(tk.END, "Listen for ESC beeps:\n")
            self.calib_status.insert(
                tk.END, "- High beep(s) = Max throttle registered\n"
            )
            self.calib_status.insert(
                tk.END, "- Low beep(s) = Min throttle registered\n"
            )
            self.calib_status.insert(
                tk.END, "- Confirmation beeps = Calibration complete\n"
            )

    def test_motor(self):
        motor = int(self.motor_var.get()) - 1  # Convert to 0-based
        throttle = int(self.test_throttle_var.get())
        duration = int(self.test_duration_var.get())

        response = messagebox.askyesno(
            "Motor Test",
            f"Test motor {self.motor_var.get()} at {throttle}% throttle?\n\n"
            "Ensure propellers are removed!",
        )
        if response:
            command = f"MOTOR_TEST:{motor},{throttle},{duration}"
            self.send_command(command)
            self.calib_status.insert(
                tk.END,
                f"Testing motor {self.motor_var.get()} "
                f"at {throttle}% for {duration}ms\n",
            )

    def test_failsafe(self):
        response = messagebox.askyesno(
            "Test Failsafe",
            "This will trigger the failsafe system.\n"
            "The drone will switch to LAND mode.\n\n"
            "Only test with propellers removed!\n\n"
            "Continue?",
        )
        if response:
            self.send_command("TEST_FAILSAFE")
            self.log("Testing failsafe system...")

    def check_failsafe_status(self):
        self.send_command("DEBUG")
        self.log("Checking failsafe status...")

    def update_connection(self):
        self.DRONE_IP = self.ip_entry.get()
        self.DRONE_PORT = int(self.port_entry.get())
        self.log(f"Updated connection to {self.DRONE_IP}:{self.DRONE_PORT}")

    def clear_log(self):
        self.log_text.delete(1.0, tk.END)

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)

    # Signal monitoring
    def monitor_signal(self):
        current_time = time.time()
        time_since_response = current_time - self.last_response_time

        # Show countdown when approaching failsafe
        if time_since_response > 0.5 and time_since_response < self.signal_timeout:
            remaining = max(0, 1.0 - time_since_response)
            if remaining > 0:
                self.countdown_label.config(text=f"Failsafe in: {remaining:.1f}s")
            else:
                self.countdown_label.config(text="FAILSAFE ACTIVE!", fg="red")
        else:
            self.countdown_label.config(text="")

        # Check for signal timeout
        if time_since_response > self.signal_timeout:
            if not self.signal_lost:
                self.signal_lost = True
                self.handle_signal_loss()
        else:
            if self.signal_lost:
                self.signal_lost = False
                self.handle_signal_restored()

        # Update signal strength indicator
        if self.failsafe_active:
            self.signal_indicator.config(text="FAILSAFE: LANDING", fg="red")
            self.failsafe_indicator.config(text="⚠ FAILSAFE ACTIVE ⚠")
            self.failsafe_indicator.config(bg="red", fg="yellow")
        elif self.signal_lost:
            self.signal_indicator.config(text="Signal: LOST", fg="red")
            self.failsafe_indicator.config(text="⚠ SIGNAL LOST ⚠")
            self.failsafe_indicator.config(bg="orange", fg="black")
        elif time_since_response > 1.0:
            self.signal_indicator.config(text="Signal: WEAK", fg="yellow")
            self.failsafe_indicator.config(text="")
        else:
            self.failsafe_indicator.config(text="")

        # Schedule next check
        self.master.after(200, self.monitor_signal)

    def handle_signal_loss(self):
        # Visual alerts
        self.log("⚠ SIGNAL LOST! Drone will land in 1 second if not restored!")

        # Flash the window
        self.flash_window()

        # Don't disable controls immediately - give 1 second warning
        self.master.after(800, self.check_failsafe_triggered)

    def check_failsafe_triggered(self):
        if self.signal_lost and not self.failsafe_active:
            # Show critical warning
            messagebox.showwarning(
                "FAILSAFE ACTIVATED",
                "Connection lost for 1 second!\n"
                "DRONE IS LANDING AUTOMATICALLY!\n\n"
                "Move closer to restore signal.",
            )
            self.set_controls_enabled(False)

    def handle_signal_restored(self):
        self.log("✓ Signal restored")
        if not self.failsafe_active:
            self.set_controls_enabled(True)

    def flash_window(self):
        # Flash the window border to get attention
        original_bg = self.master.cget("bg")
        self.master.config(bg="red")
        self.master.after(200, lambda: self.master.config(bg=original_bg))
        self.master.after(400, lambda: self.master.config(bg="red"))
        self.master.after(600, lambda: self.master.config(bg=original_bg))

    def set_controls_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        self.arm_btn.config(state=state)
        self.disarm_btn.config(state=state)
        # Add other controls as needed

    def update_status_display(self):
        # Update status indicators
        if "ARMED" in self.current_status:
            armed = self.current_status["ARMED"] == "1"
            self.armed = armed
            self.arm_indicator.config(text="ARMED" if armed else "DISARMED")
            self.arm_indicator.config(fg="green" if armed else "red")

        if "MODE" in self.current_status:
            self.mode_indicator.config(text=f"Mode: {self.current_status['MODE']}")

        if "BATTERY" in self.current_status:
            battery = self.current_status.get("BATTERY", "0")
            self.battery_indicator.config(text=f"Battery: {battery}%")

        if "RSSI" in self.current_status:
            self.signal_indicator.config(
                text=f"Signal: {self.current_status['RSSI']} dBm"
            )

        # Update status text
        status_text = "System Status:\n"
        for key, value in self.current_status.items():
            status_text += f"{key}: {value}\n"

        self.status_text.delete(1.0, tk.END)
        self.status_text.insert(1.0, status_text)

    def send_rc_channels(self):
        while self.listener_running:
            if self.rc_active:
                # Send all 8 channels
                cmd = f"RC_CHANNELS:{self.channels['roll']},{self.channels['pitch']},"
                cmd += f"{self.channels['throttle']},{self.channels['yaw']},"
                cmd += f"{self.channels['ch5']},{self.channels['ch6']},"
                cmd += f"{self.channels['ch7']},{self.channels['ch8']}"
                self.send_command(cmd)
            time.sleep(0.05)  # 20Hz update rate

    def listen_for_responses(self):
        while self.listener_running:
            try:
                data, addr = self.sock.recvfrom(1024)
                response = data.decode().strip()
                self.master.after(0, lambda: self.process_response(response))
            except socket.timeout:
                continue
            except Exception as e:
                if self.listener_running:
                    self.master.after(0, lambda: self.log(f"Listener error: {e}"))
                break

    def process_response(self, response):
        self.last_response_time = time.time()  # Update signal timestamp
        self.log(f"Received: {response}")

        if response.startswith("STATUS:"):
            self.parse_status(response)
        elif response.startswith("ACK:"):
            self.parse_ack(response)
        elif response.startswith("FAILSAFE:"):
            self.parse_failsafe(response)

    def parse_status(self, status_msg):
        try:
            status_data = status_msg.replace("STATUS:", "")
            parts = status_data.split(";")

            for part in parts:
                if ":" in part:
                    key, value = part.split(":", 1)
                    self.current_status[key] = value

            # Check failsafe status
            if "FAILSAFE" in self.current_status:
                self.failsafe_active = self.current_status["FAILSAFE"] == "1"

            self.update_status_display()
        except Exception as e:
            self.log(f"Error parsing status: {e}")

    def parse_ack(self, ack_msg):
        try:
            parts = ack_msg.split()
            if len(parts) >= 4:
                cmd_id = parts[1].split("=")[1].rstrip(",")
                result = parts[2].split("=")[1]

                if result == "0":
                    self.log(f"✓ Command {cmd_id} accepted")
                else:
                    self.log(f"✗ Command {cmd_id} failed with result {result}")
        except Exception as e:
            self.log(f"Error parsing ACK: {e}")

    def parse_failsafe(self, failsafe_msg):
        status = failsafe_msg.replace("FAILSAFE:", "")

        if status.startswith("FAILSAFE_LAND:"):
            reason = status.replace("FAILSAFE_LAND:", "")
            self.failsafe_active = True
            self.log(f"⚠ FAILSAFE ACTIVATED: {reason} - Drone is landing!")
            self.mode_indicator.config(text="Mode: LANDING (FAILSAFE)", fg="red")

            # Show critical alert
            messagebox.showwarning(
                "FAILSAFE - LANDING",
                f"Failsafe triggered: {reason}\n\n"
                "DRONE IS LANDING AUTOMATICALLY!\n"
                "DO NOT PANIC - This is for safety.",
            )

        elif status == "RECOVERED":
            self.failsafe_active = False
            self.log("✓ Failsafe recovered - control restored")
            messagebox.showinfo(
                "Failsafe Recovered",
                "Signal restored!\n" "You may regain control if safe.",
            )

    def periodic_status(self):
        self.send_command("STATUS")
        self.master.after(2000, self.periodic_status)

    def start_threads(self):
        # Start listener thread
        self.listener_thread = threading.Thread(
            target=self.listen_for_responses, daemon=True
        )
        self.listener_thread.start()

        # Start RC sender thread
        self.rc_thread = threading.Thread(target=self.send_rc_channels, daemon=True)
        self.rc_thread.start()

        # Start periodic status updates
        self.master.after(1000, self.periodic_status)

    def on_closing(self):
        self.listener_running = False
        self.master.destroy()


if _name_ == "_main_":
    root = tk.Tk()
    app = RCController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
