# Quick Start Guide - Simplified Direct PWM Control

## What You Need

### Hardware
- ✅ ESP32 Development Board
- ✅ 6x Micro Servos (SG90 or similar)
- ✅ 5-6V Power Supply (2A minimum)
- ✅ USB Cable (for PC connection)
- ✅ Jumper wires

### NO PCA9685 NEEDED! 
ESP32 controls servos directly using built-in PWM.

---

## Wiring Diagram

```
Power Supply (5-6V, 2A+)
    │
    ├─(+)─┬─→ All Servo RED wires (Power)
    │     │
    └─(-)─┴─→ All Servo BROWN/BLACK wires (GND) + ESP32 GND


ESP32 GPIO Pins → Servo Signal Wires:
    GPIO 13 ──→ LR Servo (Left/Right) ORANGE/YELLOW wire
    GPIO 12 ──→ UD Servo (Up/Down) ORANGE/YELLOW wire
    GPIO 14 ──→ TL Servo (Top Left Eyelid) ORANGE/YELLOW wire
    GPIO 27 ──→ BL Servo (Bottom Left Eyelid) ORANGE/YELLOW wire
    GPIO 26 ──→ TR Servo (Top Right Eyelid) ORANGE/YELLOW wire
    GPIO 25 ──→ BR Servo (Bottom Right Eyelid) ORANGE/YELLOW wire
    GND ─────→ Power Supply GND


PC USB ──→ ESP32 USB (for serial communication)
```

### Servo Wire Colors
- **RED** = Power (+5V) → Connect to power supply (+)
- **BROWN/BLACK** = Ground (GND) → Connect to power supply (-) AND ESP32 GND
- **ORANGE/YELLOW** = Signal → Connect to ESP32 GPIO pin

### CRITICAL: Common Ground
**All grounds MUST be connected together:**
- ESP32 GND
- Power supply GND (-)
- All servo GND wires (brown/black)

---

## Software Setup

### 1. Install Arduino IDE

Download from: https://www.arduino.cc/en/software

### 2. Add ESP32 Board Support

1. File → Preferences
2. Add to "Additional Boards Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Tools → Board → Boards Manager
4. Search "ESP32" and install **"esp32 by Espressif Systems"**

### 3. Install ESP32Servo Library

1. Tools → Manage Libraries
2. Search **"ESP32Servo"**
3. Install **"ESP32Servo by Kevin Harrington"**

### 4. Upload Code

1. Open `esp32_receiver.ino`
2. Tools → Board → ESP32 Dev Module
3. Tools → Port → [Select your COM port]
4. Click Upload (→)

### 5. Verify

1. Tools → Serial Monitor (115200 baud)
2. Should see:
   ```
   === ESP32 Eye Mechanism Initializing ===
   Direct PWM Control - No PCA9685 needed!
   Servos initialized
   Waiting for serial data from PC...
   ```

---

## PC Software Setup

### Install Python Dependencies

```bash
pip install opencv-python mediapipe pyserial numpy
```

### Run Eye Tracker

```bash
python pc_eye_tracker.py
```

---

## Testing

### Test 1: Serial Monitor Test

Open Serial Monitor and type:

```
LR:90,UD:90,BL:0
```

Eyes should move to center position.

### Test 2: Movement Test

```
LR:60,UD:90,BL:0    # Look left
LR:120,UD:90,BL:0   # Look right
LR:90,UD:60,BL:0    # Look down
LR:90,UD:120,BL:0   # Look up
LR:90,UD:90,BL:1    # Blink
```

### Test 3: Eye Tracking

```bash
python pc_eye_tracker.py
```

Press **'C'** for calibration, then follow instructions.

---

## Customization

### Change GPIO Pins

Edit in `esp32_receiver.ino`:

```cpp
#define PIN_LR  13  // Change to your GPIO
#define PIN_UD  12
#define PIN_TL  14
#define PIN_BL  27
#define PIN_TR  26
#define PIN_BR  25
```

### Adjust Servo Limits

```cpp
ServoLimits servoLimits[] = {
  {60, 120},   // LR: Adjust range
  {60, 120},   // UD: Adjust range
  {90, 10},    // TL: Swap if inverted
  {10, 90},    // BL: Swap if inverted
  {10, 90},    // TR: Swap if inverted
  {90, 10}     // BR: Swap if inverted
};
```

### Adjust Smoothing

```cpp
const float smoothing = 0.3;  // 0.1=smooth, 0.9=responsive
```

---

## Troubleshooting

### Servos Not Moving

1. **Check power supply** - Must be 5-6V, 2A minimum
2. **Check common ground** - ESP32 GND, power supply GND, and all servo GND must connect together
3. **Check signal wires** - Servo signal (orange/yellow) to correct ESP32 GPIO

### Servo Moving Wrong Direction

Swap min/max values in `servoLimits`:

```cpp
{120, 60},  // Was {60, 120} - now inverted
```

### Serial Connection Failed

1. Close Arduino Serial Monitor before running Python script
2. Check correct COM port selected
3. Verify USB cable supports data (not charge-only)

### ESP32 Upload Failed

1. Hold **BOOT** button while clicking Upload
2. Release after "Connecting..." appears
3. Try different USB cable or port

---

## Pin Reference

| Servo | GPIO Pin | Purpose |
|-------|----------|---------|
| LR | 13 | Left/Right eye movement |
| UD | 12 | Up/Down eye movement |
| TL | 14 | Top Left eyelid |
| BL | 27 | Bottom Left eyelid |
| TR | 26 | Top Right eyelid |
| BR | 25 | Bottom Right eyelid |

**Note:** These pins are defaults. You can use any PWM-capable GPIO on ESP32.

---

## Advantages of Direct PWM Control

✅ **No external controller needed** - Saves money and space  
✅ **Simpler wiring** - Direct GPIO to servo connections  
✅ **Lower latency** - No I2C communication overhead  
✅ **Easier debugging** - Fewer components to troubleshoot  
✅ **More flexible** - Can use any available GPIO pins  

---

## Full Documentation

For detailed setup and calibration:
- **ARDUINO_SETUP_GUIDE.md** - Complete Arduino setup
- **CALIBRATION_GUIDE.md** - Hardware calibration guide
- **EYE_TRACKING_README.md** - Eye tracking system overview

---

**Ready to Build!** 🚀👁️🤖
