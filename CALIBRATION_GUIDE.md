# Eye Tracking Animatronic Eye Calibration Guide

Complete guide for setting up and calibrating your eye-tracking animatronic eye mechanism using OpenCV, ESP32, and PCA9685 servo controller.

## Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [Software Requirements](#software-requirements)
3. [Hardware Setup](#hardware-setup)
4. [ESP32 Setup](#esp32-setup)
5. [PC Setup](#pc-setup)
6. [Initial Testing](#initial-testing)
7. [Servo Calibration](#servo-calibration)
8. [Eye Tracking Calibration](#eye-tracking-calibration)
9. [Fine-Tuning](#fine-tuning)
10. [Troubleshooting](#troubleshooting)

---

## Hardware Requirements

### Essential Components
- **ESP32 Development Board** (ESP32 WROOM or similar)
- **PCA9685 16-Channel PWM Servo Driver**
- **6 Micro Servos** (SG90, MG90S, or similar):
  - 1x Left/Right eye movement
  - 1x Up/Down eye movement
  - 4x Eyelid servos (Top Left, Bottom Left, Top Right, Bottom Right)
- **USB Cable** for ESP32 (for programming and serial communication)
- **5V-6V Power Supply** for servos (2A or higher recommended)
- **Jumper Wires**
- **PC with Webcam** (or external USB webcam)

### Optional Components
- Breadboard for prototyping
- Power distribution board
- Capacitor (1000µF) for power stability

---

## Software Requirements

### PC Software
```bash
# Python 3.7 or higher
python --version

# Required Python packages
pip install opencv-python
pip install mediapipe
pip install pyserial
pip install numpy
```

### ESP32 Software
- **Thonny IDE** (recommended): https://thonny.org/
- **MicroPython firmware** for ESP32: https://micropython.org/download/esp32/
- **esptool** for flashing (optional): `pip install esptool`

---

## Hardware Setup

### Wiring Diagram

```
┌─────────────────┐
│     ESP32       │
│                 │
│  GPIO 22 (SCL)  ├─────────┐
│  GPIO 21 (SDA)  ├─────┐   │
│  GPIO 17 (TX)   │     │   │
│  GPIO 16 (RX)   │     │   │
│  GND            ├───┐ │   │
│  VIN (5V)       │   │ │   │
└─────────────────┘   │ │   │
                      │ │   │
┌─────────────────┐   │ │   │
│    PCA9685      │   │ │   │
│                 │   │ │   │
│  SCL            ├───┘ │   │
│  SDA            ├─────┘   │
│  VCC            ├─────────┘
│  GND            ├───┬─────────┐
│  V+             ├─┐ │         │
└─────────────────┘ │ │         │
                    │ │         │
┌───────────────┐   │ │         │
│  Power Supply │   │ │         │
│  5V-6V 2A     │   │ │         │
│               │   │ │         │
│  (+)          ├───┘ │         │
│  (-)          ├─────┘         │
└───────────────┘               │
                                │
    All servo GND wires ────────┘

PCA9685 Servo Connections:
Channel 0 → Left/Right Servo (signal)
Channel 1 → Up/Down Servo (signal)
Channel 2 → Top Left Eyelid (signal)
Channel 3 → Bottom Left Eyelid (signal)
Channel 4 → Top Right Eyelid (signal)
Channel 5 → Bottom Right Eyelid (signal)
```

### Important Wiring Notes

1. **Power Supply**: 
   - DO NOT power servos from ESP32's 3.3V or 5V pin
   - Use external 5V-6V power supply (2A minimum)
   - Connect PCA9685 V+ to power supply positive
   - Connect all grounds together (ESP32, PCA9685, Power Supply)

2. **I2C Connection**:
   - ESP32 GPIO 22 (SCL) → PCA9685 SCL
   - ESP32 GPIO 21 (SDA) → PCA9685 SDA
   - Use pull-up resistors if experiencing I2C issues (4.7kΩ to 3.3V)

3. **Serial Connection** (for PC communication):
   - USB cable from PC to ESP32 will handle serial communication
   - Note the COM port (Windows) or /dev/tty* (Linux/Mac)

---

## ESP32 Setup

### Step 1: Install MicroPython on ESP32

1. **Download MicroPython firmware**:
   - Visit: https://micropython.org/download/esp32/
   - Download the latest stable firmware (.bin file)

2. **Flash MicroPython**:

   **Option A: Using Thonny IDE**
   - Open Thonny IDE
   - Go to Tools → Options → Interpreter
   - Select "MicroPython (ESP32)"
   - Click "Install or update MicroPython"
   - Select your ESP32 port and the downloaded firmware
   - Click "Install"

   **Option B: Using esptool**
   ```bash
   # Install esptool
   pip install esptool

   # Erase flash
   esptool.py --port COM3 erase_flash

   # Flash MicroPython (replace COM3 with your port)
   esptool.py --chip esp32 --port COM3 write_flash -z 0x1000 esp32-*.bin
   ```

3. **Verify installation**:
   - Open Thonny
   - Select ESP32 port in interpreter settings
   - You should see the MicroPython REPL prompt `>>>`

### Step 2: Upload ESP32 Code

1. **Open Thonny IDE**
2. **Copy the code** from `esp32_receiver.py`
3. **Save to ESP32**:
   - File → Save As...
   - Select "MicroPython device"
   - Save as `main.py`
4. **Verify**: Press the reset button on ESP32, code should run automatically

---

## PC Setup

### Step 1: Install Python Dependencies

```bash
# Create virtual environment (optional but recommended)
python -m venv eye_tracker_env
source eye_tracker_env/bin/activate  # On Windows: eye_tracker_env\Scripts\activate

# Install required packages
pip install opencv-python
pip install mediapipe
pip install pyserial
pip install numpy
```

### Step 2: Find ESP32 Serial Port

**Windows**:
```powershell
# Open Device Manager
# Look under "Ports (COM & LPT)"
# Note the COM port (e.g., COM3, COM4)
```

**Linux/Mac**:
```bash
# List available ports
ls /dev/tty*

# Usually something like:
# Linux: /dev/ttyUSB0 or /dev/ttyACM0
# Mac: /dev/tty.usbserial-*
```

### Step 3: Test Serial Connection

```bash
# Run the PC eye tracker (will list available ports)
python pc_eye_tracker.py
```

---

## Initial Testing

### Test 1: Verify ESP32 is Running

1. Open Thonny IDE
2. Connect to ESP32
3. Check the Shell/Console for output:
   ```
   Initializing I2C and PCA9685...
   Initializing UART...
   Servos initialized to neutral position
   Eye Mechanism initialized
   Waiting for serial data from PC...
   ```

### Test 2: Manual Servo Test

In Thonny's REPL, test individual servos:

```python
from machine import Pin, I2C
from esp32_receiver import PCA9685, SERVO_CHANNELS

i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
pca = PCA9685(i2c)
pca.set_pwm_freq(50)

# Test each servo (should move to center)
pca.set_servo_angle(SERVO_CHANNELS["LR"], 90)
pca.set_servo_angle(SERVO_CHANNELS["UD"], 90)

# Test movement
pca.set_servo_angle(SERVO_CHANNELS["LR"], 60)   # Left
time.sleep(1)
pca.set_servo_angle(SERVO_CHANNELS["LR"], 120)  # Right
time.sleep(1)
pca.set_servo_angle(SERVO_CHANNELS["LR"], 90)   # Center
```

### Test 3: Webcam Test

```bash
# Run the eye tracker
python pc_eye_tracker.py

# You should see:
# - Webcam feed window
# - Your face detected with mesh overlay
# - Green circle showing gaze position
# - LR, UD values updating
```

---

## Servo Calibration

### Step 1: Identify Servo Directions

Each servo may need different min/max values depending on mounting orientation.

1. **Test Left/Right Servo**:
   - Run manual test: angles 60, 90, 120
   - Note the actual movement direction
   - If reversed, swap min/max in `servo_limits`

2. **Test Up/Down Servo**:
   - Run manual test: angles 60, 90, 120
   - Note the actual movement direction
   - If reversed, swap min/max in `servo_limits`

3. **Test Eyelid Servos**:
   - For each eyelid, test angles 10 and 90
   - Determine which angle is "closed" (eyelids touching)
   - Adjust `servo_limits` so that:
     - `min` value = CLOSED position
     - `max` value = OPEN position

### Step 2: Adjust Servo Limits

Edit `esp32_receiver.py`:

```python
servo_limits = {
    "LR": (60, 120),    # Adjust based on your mechanism
    "UD": (60, 120),    # Adjust based on your mechanism
    "TL": (90, 10),     # Top Left: 90=closed, 10=open (inverted)
    "BL": (10, 90),     # Bottom Left: 10=closed, 90=open
    "TR": (10, 90),     # Top Right: 10=closed, 90=open
    "BR": (90, 10),     # Bottom Right: 90=closed, 10=open (inverted)
}
```

### Step 3: Test Movement Range

```python
# Test full range of motion
for angle in range(60, 121, 10):
    pca.set_servo_angle(SERVO_CHANNELS["LR"], angle)
    time.sleep(0.5)
```

Adjust limits to prevent:
- Mechanical binding
- Servo strain
- Unnatural eye positions

---

## Eye Tracking Calibration

### Step 1: Run Calibration Mode

```bash
python pc_eye_tracker.py
```

Press **'C'** to enter calibration mode

### Step 2: Calibrate Center Position

1. Look **directly at the center** of your screen
2. Press **'C'** to capture center position
3. Verify: "Center set to: X=0.xxx, Y=0.xxx"

### Step 3: Calibrate Movement Range

1. Look **FAR LEFT** → Press **'L'**
2. Look **FAR RIGHT** → Press **'R'**
3. Look **FAR UP** → Press **'U'**
4. Look **FAR DOWN** → Press **'D'**

### Step 4: Save Calibration

Press **'S'** to save calibration settings

You should see:
```
=== Calibration Saved ===
Center: X=0.500, Y=0.500
Range: X=0.150, Y=0.120
```

### Step 5: Adjust Blink Threshold

Press **'B'** and enter a value between 0.15 and 0.25:
- Lower value = more sensitive (easier to trigger)
- Higher value = less sensitive (need to close eyes more)

Default: **0.20**

---

## Fine-Tuning

### Smoothing Adjustment

**In `pc_eye_tracker.py`**:
```python
self.calibration = {
    'smoothing': 0.3   # Adjust 0.1 (smooth) to 0.9 (responsive)
}
```

**In `esp32_receiver.py`**:
```python
self.smoothing = 0.3  # Adjust 0.1 (smooth) to 0.9 (responsive)
```

### Movement Range Adjustment

If eyes move too much or too little:

**In `pc_eye_tracker.py`** (line ~206-211):
```python
# Adjust the multiplier (currently ±30 degrees)
lr_angle = 90 - (norm_x * 30)  # Try 20, 25, 35, 40
ud_angle = 90 + (norm_y * 30)  # Try 20, 25, 35, 40
```

### Blink Duration

**In `esp32_receiver.py`**:
```python
self.blink_duration = 150  # milliseconds (try 100-300)
```

---

## Troubleshooting

### ESP32 Issues

**Problem**: "No I2C devices found"
- **Solution**: Check I2C wiring (GPIO 21, 22)
- Add pull-up resistors (4.7kΩ)
- Verify PCA9685 has power

**Problem**: "Servos not moving"
- **Solution**: 
  - Check external power supply is connected
  - Verify servo signal wires in correct PCA9685 channels
  - Test servos with manual commands

**Problem**: "Serial data not received"
- **Solution**:
  - Verify UART initialization (GPIO 16, 17)
  - Check baud rate matches (115200)
  - Try different USB cable

### PC Software Issues

**Problem**: "Could not open webcam"
- **Solution**:
  - Check webcam permissions
  - Try different camera index: `cv2.VideoCapture(1)` or `(2)`
  - Test with `cv2.VideoCapture(-1)` to auto-detect

**Problem**: "No face detected"
- **Solution**:
  - Ensure good lighting
  - Position face clearly in frame
  - Adjust `min_detection_confidence=0.3` for easier detection

**Problem**: "Serial port error"
- **Solution**:
  - Close other programs using the port (Arduino IDE, Thonny)
  - Verify correct port name
  - Check USB cable connection
  - Try different USB port

### Calibration Issues

**Problem**: "Eyes move in wrong direction"
- **Solution**: Swap min/max values in `servo_limits`

**Problem**: "Eyes drift or jitter"
- **Solution**: 
  - Increase smoothing value (0.1-0.3)
  - Reduce movement range in calibration
  - Check for mechanical friction

**Problem**: "Blinks not detected"
- **Solution**:
  - Press 'B' to adjust blink threshold
  - Try values between 0.15-0.25
  - Ensure good face lighting

**Problem**: "Eyelids don't close properly"
- **Solution**:
  - Adjust `servo_limits` for eyelids
  - Verify min value fully closes eyelids
  - Check mechanical assembly

### Performance Issues

**Problem**: "Laggy or slow tracking"
- **Solution**:
  - Reduce webcam resolution
  - Disable face mesh drawing (comment out in code)
  - Increase smoothing for less frequent updates

**Problem**: "High CPU usage"
- **Solution**:
  - Lower webcam FPS
  - Reduce MediaPipe detection confidence
  - Close other applications

---

## Advanced Configuration

### Custom Servo Limits

Edit values based on your hardware:

```python
# Wider range (more movement)
servo_limits = {
    "LR": (40, 140),
    "UD": (40, 140),
}

# Narrower range (subtle movement)
servo_limits = {
    "LR": (70, 110),
    "UD": (70, 110),
}
```

### Eyelid Following Behavior

Adjust how eyelids move with vertical gaze:

```python
# In esp32_receiver.py control_ud_and_lids() function
top_close_factor = 0.6 * (1 - ud_progress)     # Adjust 0.6 (try 0.3-0.8)
bottom_close_factor = 0.6 * ud_progress         # Adjust 0.6 (try 0.3-0.8)
```

### Serial Communication Format

Default format: `"LR:90,UD:90,BL:0\n"`

To modify, update both:
- `pc_eye_tracker.py` → `send_to_esp32()` function
- `esp32_receiver.py` → `parse_command()` function

---

## Testing Checklist

- [ ] ESP32 powers on and runs code
- [ ] All 6 servos respond to test commands
- [ ] Webcam captures video feed
- [ ] Face detection works
- [ ] Gaze position displays on screen
- [ ] Serial communication established
- [ ] Eyes move left/right correctly
- [ ] Eyes move up/down correctly
- [ ] Eyelids open/close correctly
- [ ] Blink detection works
- [ ] Movement is smooth (not jittery)
- [ ] Calibration saves properly
- [ ] System recovers after PC disconnect

---

## Usage Tips

1. **Lighting**: Ensure good, even lighting on your face
2. **Distance**: Sit 50-80cm from webcam for best tracking
3. **Background**: Avoid cluttered backgrounds
4. **Position**: Face camera directly for best accuracy
5. **Calibration**: Recalibrate if you change sitting position
6. **Power**: Ensure stable power supply for servos

---

## Next Steps

Once calibrated, you can:
- Add expressions (winks, squints)
- Implement autonomous behaviors
- Add sound reactivity
- Connect additional sensors
- Create multiple eye mechanisms for characters

---

## Support

For issues or questions:
1. Check this guide's Troubleshooting section
2. Review code comments in source files
3. Test individual components separately
4. Verify all connections and power

Happy Building! 🤖👁️
