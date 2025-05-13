## Key Features

### 1. Failsafe System

Automatically lands the drone in case of signal loss or control timeout. Features dual-trigger system:

- Primary: 1-second command timeout
- Secondary: 3-second WiFi loss timeout

```cpp
void executeFailsafe(const char* reason) {
    if (failsafeActive) return;
    failsafeActive = true;

    // Stop RC override and switch to LAND mode
    rcOverrideActive = false;
    set_flight_mode(LAND);

    // Center all controls
    rcChannels.ch1 = 1500; // Roll
    rcChannels.ch2 = 1500; // Pitch
    rcChannels.ch3 = 1000; // Min throttle
    rcChannels.ch4 = 1500; // Yaw
    send_rc_override();
}
```

### 2. Hand Gesture Control

Uses computer vision to interpret hand gestures for drone control:

- Thumb up → ARM
- Open palm → LAND
- Rock sign → Toggle control mode
- Point down → STABILIZE mode

```python
def detect_gesture(self, metrics: dict) -> str:
    if not metrics:
        return "NONE"

    fingers = metrics["fingers_extended"]
    num_fingers = metrics["num_fingers_extended"]

    # Thumb up - ARM
    if fingers[0] and num_fingers <= 2 and thumb_tip[1] < thumb_base[1] - 0.05:
        return "ARM"

    # Open palm - LAND
    elif num_fingers >= 4:
        return "LAND"
```

### 3. Smooth Throttle Control

Implements rate-limited throttle changes for smooth altitude control:

```cpp
void updateSmoothThrottle() {
    unsigned long currentTime = millis();

    if (currentTime - lastThrottleUpdate >= THROTTLE_UPDATE_INTERVAL) {
        int throttleDiff = targetThrottle - currentSmoothThrottle;

        if (abs(throttleDiff) > THROTTLE_SMOOTH_RATE) {
            if (throttleDiff > 0) {
                currentSmoothThrottle += THROTTLE_SMOOTH_RATE;
            } else {
                currentSmoothThrottle -= THROTTLE_SMOOTH_RATE;
            }
        } else {
            currentSmoothThrottle = targetThrottle;
        }
    }
}
```

### 4. GUI Control Interface

Features a comprehensive control panel with:

- Real-time telemetry display
- Virtual joysticks
- Flight mode selection
- Failsafe monitoring

```python
def create_flight_controls(self, parent):
    # Status indicators
    self.arm_indicator = tk.Label(
        status_frame,
        text="DISARMED",
        fg="red",
        bg="black",
        font=("Arial", 16, "bold"),
    )

    # Virtual joysticks
    self.left_stick = self.create_joystick(left_frame, "left")  # Throttle/Yaw
    self.right_stick = self.create_joystick(right_frame, "right")  # Pitch/Roll
```

### 5. MAVLink Communication

Handles bidirectional communication with flight controller using MAVLink protocol:

```cpp
void send_rc_override() {
    mavlink_msg_rc_channels_override_pack(SYSTEM_ID, COMPONENT_ID, &msg,
        TARGET_SYSTEM, TARGET_COMPONENT,
        rcChannels.ch1, rcChannels.ch2,
        currentSmoothThrottle,
        rcChannels.ch4,
        rcChannels.ch5, rcChannels.ch6,
        rcChannels.ch7, rcChannels.ch8,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    send_mavlink_message();
}
```
## System Requirements

### Hardware
- ESP32 development board
- Flight controller with MAVLink support
- USB camera (for gesture control)
- WiFi network

### Software Dependencies
- Python 3.7+
  - OpenCV
  - MediaPipe
  - tkinter
- Arduino IDE with ESP32 support
- MAVLink library

## Setup Instructions

1. Flash ESP32 with the provided firmware
2. Connect ESP32 to flight controller via UART
3. Install Python dependencies
4. Configure WiFi settings in ESP32 code
5. Launch control interface:
   ```bash
   python gui_control.py
   ```

## Safety Features

- Automatic failsafe landing
- Command timeout protection
- Signal strength monitoring
- Battery voltage monitoring
- Emergency stop function
- Smooth control transitions
- Multiple control redundancy