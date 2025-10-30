# Eye Tracking Animatronic Eyes System

Transform your animatronic eye mechanism into a real-time eye-tracking system that mirrors your eye movements using computer vision!

## Overview

This system uses:
- **PC with Webcam**: Captures your face and tracks eye movements using OpenCV and MediaPipe
- **Serial Communication**: Sends eye position data to ESP32 in real-time
- **ESP32 + PCA9685**: Receives commands and controls 6 servos for realistic eye movement

## Quick Start

### 1. Install PC Requirements

```bash
pip install opencv-python mediapipe pyserial numpy
```

### 2. Upload ESP32 Code

- Open Thonny IDE
- Connect your ESP32
- Upload `esp32_receiver.py` as `main.py` to your ESP32

### 3. Run the Eye Tracker

```bash
python pc_eye_tracker.py
```

### 4. Calibrate

- Press **'C'** to enter calibration mode
- Follow on-screen instructions
- Press **'S'** to save

## File Descriptions

### `pc_eye_tracker.py` (Run on PC)
- Captures webcam video
- Detects face and tracks eye/iris position using MediaPipe
- Detects blinks using Eye Aspect Ratio (EAR)
- Sends eye position data via serial to ESP32
- Provides live calibration interface

**Key Features:**
- Real-time gaze tracking
- Automatic blink detection
- Smoothing for natural movement
- Interactive calibration mode
- Visual feedback with FPS counter

### `esp32_receiver.py` (Upload to ESP32)
- Receives serial commands from PC
- Controls 6 servos via PCA9685
- Implements smooth servo movements
- Coordinates eyelid movement with vertical gaze
- Handles blink animations
- Timeout protection (returns to neutral if connection lost)

**Key Features:**
- Serial command parser
- Smooth servo interpolation
- Realistic eyelid behavior
- Blink animation
- Automatic neutral position on timeout

### `CALIBRATION_GUIDE.md`
Complete step-by-step guide covering:
- Hardware assembly
- Wiring diagrams
- Software installation
- Servo calibration
- Eye tracking calibration
- Troubleshooting

## Hardware Setup (Quick Reference)

```
ESP32 Connections:
├── GPIO 22 (SCL) → PCA9685 SCL
├── GPIO 21 (SDA) → PCA9685 SDA
├── GND → PCA9685 GND, Power Supply (-)
└── USB → PC (for serial communication)

PCA9685 Connections:
├── Channel 0 → Left/Right Servo
├── Channel 1 → Up/Down Servo
├── Channel 2 → Top Left Eyelid
├── Channel 3 → Bottom Left Eyelid
├── Channel 4 → Top Right Eyelid
├── Channel 5 → Bottom Right Eyelid
└── V+ → External 5V-6V Power Supply (+)
```

## Serial Communication Protocol

**Format:** `LR:90,UD:90,BL:0\n`

- **LR**: Left/Right angle (60-120°)
- **UD**: Up/Down angle (60-120°)
- **BL**: Blink state (0=open, 1=closed)

**Example:** `LR:85,UD:95,BL:0\n`

## Calibration Quick Steps

1. **Run PC tracker**: `python pc_eye_tracker.py`
2. **Press 'C'**: Enter calibration mode
3. **Look CENTER**: Press 'C' to set center
4. **Look FAR LEFT**: Press 'L'
5. **Look FAR RIGHT**: Press 'R'
6. **Look FAR UP**: Press 'U'
7. **Look FAR DOWN**: Press 'D'
8. **Press 'S'**: Save calibration

## Keyboard Controls (PC Tracker)

| Key | Action |
|-----|--------|
| `Q` | Quit the application |
| `C` | Calibration mode / Set center point |
| `L` | Capture far left gaze (during calibration) |
| `R` | Capture far right gaze (during calibration) |
| `U` | Capture far up gaze (during calibration) |
| `D` | Capture far down gaze (during calibration) |
| `S` | Save calibration and exit calibration mode |
| `B` | Adjust blink detection threshold |

## Adjustable Parameters

### PC Side (`pc_eye_tracker.py`)

```python
# Smoothing (line ~25)
'smoothing': 0.3  # 0.1=smooth, 0.9=responsive

# Movement range (line ~206-211)
lr_angle = 90 - (norm_x * 30)  # Adjust ±30 degrees
ud_angle = 90 + (norm_y * 30)  # Adjust ±30 degrees

# Blink threshold (line ~40)
self.blink_threshold = 0.2  # 0.15-0.25
```

### ESP32 Side (`esp32_receiver.py`)

```python
# Servo limits (line ~26-35)
servo_limits = {
    "LR": (60, 120),
    "UD": (60, 120),
    # ... adjust for your hardware
}

# Smoothing (line ~115)
self.smoothing = 0.3  # 0.1=smooth, 0.9=responsive

# Blink duration (line ~121)
self.blink_duration = 150  # milliseconds
```

## Troubleshooting Quick Fixes

### No face detected
- Improve lighting
- Reduce `min_detection_confidence` to 0.3
- Ensure face is clearly visible

### Eyes move in wrong direction
- Swap min/max values in `servo_limits`

### Jittery movement
- Increase smoothing value (try 0.2 or 0.1)
- Check for mechanical friction

### Blink not detected
- Press 'B' to adjust threshold
- Try values between 0.15-0.25

### Serial connection failed
- Close other programs using the port
- Verify correct COM port
- Check USB cable

### Servos not moving
- Verify external power supply is connected
- Check servo signal wires
- Test with manual commands

## Performance Tips

1. **Good Lighting**: Essential for accurate face detection
2. **Distance**: Sit 50-80cm from webcam
3. **Direct View**: Face the camera directly
4. **Stable Position**: Minimize head movement
5. **Clean Background**: Avoid cluttered backgrounds

## How It Works

### Eye Tracking Pipeline

```
Webcam → OpenCV → MediaPipe Face Mesh → Iris Detection
    ↓
Gaze Position Calculation (X, Y coordinates)
    ↓
Blink Detection (Eye Aspect Ratio)
    ↓
Map to Servo Angles (60-120°)
    ↓
Apply Smoothing
    ↓
Serial Command → ESP32
```

### ESP32 Processing

```
Serial Input → Parse Command → Apply Smoothing
    ↓
Update LR Servo (Channel 0)
    ↓
Update UD Servo + Eyelids (Channels 1-5)
    ↓
Handle Blink Animation
```

## Advanced Features

### Eyelid Following
Eyelids automatically adjust based on vertical gaze:
- Looking up: Top eyelids slightly close
- Looking down: Bottom eyelids slightly close

### Timeout Protection
If serial connection is lost, eyes automatically return to neutral position after 1 second.

### Blink Animation
Natural blink duration (150ms default) with smooth eyelid closure and opening.

## Dependencies

### PC Requirements
- Python 3.7+
- opencv-python (cv2)
- mediapipe
- pyserial
- numpy

### ESP32 Requirements
- MicroPython firmware
- Built-in modules: machine, time, sys

## Credits

Based on Will Cogley's EYEMECH ε3.2 control code, adapted for eye tracking functionality.

## License

See LICENSE.md for details.

---

**Need Help?** Check `CALIBRATION_GUIDE.md` for detailed instructions and troubleshooting!
