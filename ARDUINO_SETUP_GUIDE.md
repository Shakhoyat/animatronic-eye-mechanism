# Arduino/C++ Setup Guide for ESP32 Eye Tracking

Complete guide for setting up the C++ version of the eye-tracking animatronic system using Arduino IDE or PlatformIO.

## Table of Contents

1. [Requirements](#requirements)
2. [Arduino IDE Setup](#arduino-ide-setup)
3. [PlatformIO Setup](#platformio-setup)
4. [Upload Code](#upload-code)
5. [Testing](#testing)
6. [Troubleshooting](#troubleshooting)

---

## Requirements

### Hardware
- ESP32 Development Board
- 6 Micro Servos (SG90 or similar)
- USB Cable (for PC connection)
- 5V-6V Power Supply (2A minimum for servos)

**Note:** No PCA9685 needed! ESP32 controls servos directly with built-in PWM.

### Software
Choose one:
- **Arduino IDE 2.x** (recommended for beginners)
- **PlatformIO** (recommended for advanced users)

---

## Arduino IDE Setup

### Step 1: Install Arduino IDE

1. Download Arduino IDE 2.x from: https://www.arduino.cc/en/software
2. Install and launch Arduino IDE

### Step 2: Install ESP32 Board Support

1. Open Arduino IDE
2. Go to **File → Preferences**
3. In "Additional Boards Manager URLs", add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Click **OK**
5. Go to **Tools → Board → Boards Manager**
6. Search for "ESP32"
7. Install **"esp32 by Espressif Systems"**
8. Wait for installation to complete

### Step 3: Install Required Library

1. Go to **Tools → Manage Libraries** (or Sketch → Include Library → Manage Libraries)
2. Search and install:
   - **"ESP32Servo"** by Kevin Harrington

### Step 4: Configure Board Settings

1. Go to **Tools → Board → esp32**
2. Select **"ESP32 Dev Module"**
3. Configure the following settings:
   ```
   Board: ESP32 Dev Module
   Upload Speed: 115200
   CPU Frequency: 240MHz (WiFi/BT)
   Flash Frequency: 80MHz
   Flash Mode: QIO
   Flash Size: 4MB (32Mb)
   Partition Scheme: Default 4MB with spiffs
   Core Debug Level: None
   PSRAM: Disabled
   Port: [Select your COM port]
   ```

### Step 5: Select COM Port

1. Connect ESP32 to PC via USB
2. Go to **Tools → Port**
3. Select the COM port (e.g., COM3, COM4)
   - On Windows: COMx
   - On Linux: /dev/ttyUSB0 or /dev/ttyACM0
   - On Mac: /dev/cu.usbserial-*

### Step 6: Open the Code

1. Open `esp32_receiver.ino` in Arduino IDE
2. The file should open in a new window

### Step 7: Upload to ESP32

1. Click the **Upload** button (→ arrow icon)
2. Wait for compilation and upload
3. You should see:
   ```
   Connecting........_____....._____
   Writing at 0x00008000... (100%)
   Leaving...
   Hard resetting via RTS pin...
   ```

### Step 8: Verify Upload

1. Open **Serial Monitor** (Tools → Serial Monitor)
2. Set baud rate to **115200**
3. Press **Reset** button on ESP32
4. You should see:
   ```
   === ESP32 Eye Mechanism Initializing ===
   Initializing servos to neutral position...
   Servos initialized
   Eye Mechanism initialized
   Waiting for serial data from PC...
   Format: LR:90,UD:90,BL:0
   ```

---

## PlatformIO Setup

### Step 1: Install PlatformIO

**Option A: VS Code Extension**
1. Install Visual Studio Code
2. Open Extensions (Ctrl+Shift+X)
3. Search "PlatformIO IDE"
4. Click Install

**Option B: Standalone**
1. Download from: https://platformio.org/install

### Step 2: Create New Project

1. Open PlatformIO Home
2. Click **"New Project"**
3. Configure:
   - **Name:** ESP32_Eye_Tracker
   - **Board:** Espressif ESP32 Dev Module
   - **Framework:** Arduino
4. Click **Finish**

### Step 3: Configure platformio.ini

Replace the contents of `platformio.ini` with:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps = 
    madhephaestus/ESP32Servo@^0.13.0
upload_speed = 921600
```

### Step 4: Add Code

1. Copy `esp32_receiver.ino` content
2. Replace contents of `src/main.cpp` with the code
3. Save the file

### Step 5: Build and Upload

1. Click **PlatformIO: Build** (checkmark icon in bottom toolbar)
2. After successful build, click **PlatformIO: Upload** (arrow icon)
3. Wait for upload to complete

### Step 6: Monitor Serial Output

1. Click **PlatformIO: Serial Monitor** (plug icon)
2. You should see initialization messages

---

## Upload Code

### Quick Upload Steps (Arduino IDE)

1. **Connect ESP32** to PC via USB
2. **Select Port**: Tools → Port → [Your COM port]
3. **Click Upload** button
4. **Wait** for "Done uploading" message
5. **Open Serial Monitor** to verify

### Troubleshooting Upload

**"Failed to connect to ESP32"**
- Hold **BOOT** button on ESP32 while clicking Upload
- Release after "Connecting..." appears

**"A fatal error occurred: MD5 of file does not match"**
- Disconnect and reconnect USB cable
- Try a different USB port
- Use a different USB cable (data cable, not just charging cable)

**"Serial port [COM3] not found"**
- Install CP2102 or CH340 USB driver (depending on your ESP32 board)
- Check Device Manager (Windows) for the correct port
- Try different USB ports

---

## Testing

### Test 1: Serial Monitor Test

1. Open Serial Monitor (115200 baud)
2. You should see initialization messages
3. Type manually: `LR:60,UD:90,BL:0` and press Enter
4. Eyes should move left

### Test 2: Test Commands

Try these commands in Serial Monitor:

```
LR:90,UD:90,BL:0    // Center position
LR:60,UD:90,BL:0    // Look left
LR:120,UD:90,BL:0   // Look right
LR:90,UD:60,BL:0    // Look down
LR:90,UD:120,BL:0   // Look up
LR:90,UD:90,BL:1    // Blink
```

### Test 3: Run PC Eye Tracker

```bash
python pc_eye_tracker.py
```

The PC tracker will automatically send commands to ESP32.

---

## Troubleshooting

### Servos Not Moving

**Check Power Supply:**
- Verify 5V-6V external power supply is connected to servo power rails
- Check all GND connections (ESP32 GND must connect to servo GND and power supply GND)
- ESP32 can only provide signal, NOT power for servos

**Check Wiring:**
- Signal wires from ESP32 GPIOs to servo signal pins
- Default pins: GPIO13, 12, 14, 27, 26, 25
- Adjust pin numbers in code if using different GPIOs

**Check Servo Connection:**
- Red wire = Power (+5V)
- Brown/Black wire = Ground (GND)
- Orange/Yellow wire = Signal (from ESP32 GPIO)

### Serial Communication Issues

**PC Can't Find ESP32:**
- Close Arduino IDE Serial Monitor before running Python script
- Only one program can access serial port at a time

**No Data Received:**
- Verify baud rate is 115200 in both PC and ESP32 code
- Check USB cable (some cables are charge-only)
- Try different USB port

### Compilation Errors

**"ESP32Servo.h: No such file"**
- Install "ESP32Servo" library by Kevin Harrington via Library Manager

**Compilation errors**
- Verify ESP32 board support is installed correctly
- Select "ESP32 Dev Module" as board

**"espressif32 platform not installed"** (PlatformIO)
- Run: `pio platform install espressif32`

### Servo Movement Issues

**Servos Moving in Wrong Direction:**
- Swap min/max values in `servoLimits` array:
  ```cpp
  {120, 60},  // Swapped LR values
  ```

**Servos Jittering:**
- Increase smoothing value (try 0.1 or 0.2):
  ```cpp
  const float smoothing = 0.1;  // More smooth
  ```
- Add capacitor (1000µF) across power supply terminals

**Limited Range:**
- Adjust servo limits:
  ```cpp
  ServoLimits servoLimits[] = {
    {40, 140},   // Wider range
    {40, 140},
    // ...
  };
  ```

---

## Adjustable Parameters

### GPIO Pins (Line 18-23)

```cpp
#define PIN_LR  13  // Left/Right servo
#define PIN_UD  12  // Up/Down servo
#define PIN_TL  14  // Top Left eyelid
#define PIN_BL  27  // Bottom Left eyelid
#define PIN_TR  26  // Top Right eyelid
#define PIN_BR  25  // Bottom Right eyelid
```

### Servo Limits (Line 26-35)

```cpp
ServoLimits servoLimits[] = {
  {60, 120},   // LR: Adjust for your mechanism
  {60, 120},   // UD: Adjust for your mechanism
  {90, 10},    // TL: Top Left eyelid (inverted)
  {10, 90},    // BL: Bottom Left eyelid
  {10, 90},    // TR: Top Right eyelid
  {90, 10}     // BR: Bottom Right eyelid (inverted)
};
```

### Smoothing

```cpp
const float smoothing = 0.3;  // 0.1 = smooth, 0.9 = responsive
```

### Blink Duration

```cpp
const int blinkDuration = 150;  // milliseconds (try 100-300)
```

### Timeout Duration

```cpp
const unsigned long timeoutDuration = 1000;  // milliseconds
```

### Eyelid Follow Factor

```cpp
float topCloseFactor = 0.6 * (1.0 - udProgress);     // Adjust 0.6
float bottomCloseFactor = 0.6 * udProgress;          // Adjust 0.6
```

---

## Wiring Diagram

```
ESP32                         Servos
GPIO 13 (Signal) ─────────→  LR Servo (Orange/Yellow wire)
GPIO 12 (Signal) ─────────→  UD Servo (Orange/Yellow wire)
GPIO 14 (Signal) ─────────→  TL Servo (Orange/Yellow wire)
GPIO 27 (Signal) ─────────→  BL Servo (Orange/Yellow wire)
GPIO 26 (Signal) ─────────→  TR Servo (Orange/Yellow wire)
GPIO 25 (Signal) ─────────→  BR Servo (Orange/Yellow wire)
GND ──────────────────────→  All Servo GND (Brown/Black wires)

Power Supply (5-6V, 2A+)
(+) ──────────────────────→  All Servo Power (Red wires)
(-) ──────────────────────→  All Servo GND + ESP32 GND

USB Cable
ESP32 USB ────────────────→  PC (for serial communication)
```

**Important:**
- Connect ALL grounds together (ESP32, servos, power supply)
- ESP32 only provides signal, NOT power to servos
- Use external 5-6V power supply rated for at least 2A

---

## Performance Optimization

### Reduce Latency
1. Increase smoothing value (0.5-0.9)
2. Reduce loop delay:
   ```cpp
   delay(5);  // Instead of delay(10)
   ```

### Reduce CPU Usage
1. Increase loop delay:
   ```cpp
   delay(20);  // Instead of delay(10)
   ```

### Improve Stability
1. Add buffer clearing:
   ```cpp
   // In setup()
   while (Serial.available()) Serial.read();
   ```

---

## Next Steps

1. ✅ Upload code to ESP32
2. ✅ Test with Serial Monitor
3. ✅ Run PC eye tracker
4. ✅ Calibrate system
5. ✅ Fine-tune parameters
6. ✅ Enjoy your eye-tracking animatronic eyes!

---

## Additional Resources

- **Arduino ESP32 Documentation:** https://docs.espressif.com/projects/arduino-esp32/
- **ESP32Servo Library:** https://github.com/madhephaestus/ESP32Servo
- **PlatformIO Documentation:** https://docs.platformio.org/

---

## Support

If you encounter issues:
1. Check wiring and connections
2. Verify power supply
3. Test individual components
4. Check Serial Monitor output
5. Review this guide's troubleshooting section

Happy Building! 🤖👁️
