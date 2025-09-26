# 🎭 Animatronic Eye Tracking System

A complete eye tracking system that uses your PC webcam to control animatronic eye movements via ESP32 and servo motors. **Safety-first design** prevents hardware damage through conservative movement limits and emergency stop features.

## 🚀 Quick Start Guide

### ⚠️ SAFETY FIRST - READ THIS!

**CRITICAL**: This system can damage your servos if not set up properly. Follow these safety steps:

1. **Test each servo manually** before automation
2. **Use external 5V power supply** for servos (2A minimum)
3. **Keep emergency stop accessible** (spacebar in Python app)
4. **Start with conservative servo limits** and increase gradually

## 📦 What's Included

```
animatronic-eye-mechanism/
├── esp32/
│   └── animatronic_eye_controller.ino    # ESP32 Arduino code
├── python/
│   ├── eye_tracker.py                    # Main Python application
│   └── requirements.txt                  # Python dependencies
├── docs/
│   └── [documentation files]
└── README.md                             # This file
```

## 🔧 Hardware Requirements

### ESP32 Setup
- **ESP32 development board** (ESP32-WROOM-32 recommended)
- **7 servo motors**:
  - Eye mechanisms (0-3): Micro servos - **RANGE LIMITED**
  - Base movement (4-6): Standard servos (SG90, MG90S)
- **External 5V power supply** (2A minimum) - **REQUIRED**

### Wiring Guide
```
ESP32 Pin → Servo Function                → Safe Range
Pin 2  → Eye Servo 0 (vertical)          → 60°-120° (CONSERVATIVE)
Pin 4  → Eye Servo 1 (horizontal)        → 60°-120° (CONSERVATIVE)  
Pin 5  → Eye Servo 2 (lid/mechanism)     → 70°-110° (VERY LIMITED)
Pin 18 → Eye Servo 3 (secondary)         → 60°-120° (CONSERVATIVE)
Pin 19 → Base Pan (left/right)           → 0°-180° (Full rotation)
Pin 21 → Base Tilt (up/down)             → 0°-180° (Full rotation)
Pin 22 → Base Roll (side tilt)           → 0°-180° (Full rotation)
```

**⚠️ POWER WARNING**: Connect ESP32 and servos to **common ground**. Use external 5V supply for servos!

## 📱 Software Installation

### Step 1: Python Setup (PC Side)
```bash
# Navigate to project directory
cd animatronic-eye-mechanism/python

# Install dependencies
pip install -r requirements.txt
```

### Step 2: ESP32 Setup (Arduino IDE)
1. **Install ESP32 board support**:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

2. **Install required libraries** (Library Manager):
   - `ESP32Servo` by Kevin Harrington
   - `ArduinoJson` by Benoit Blanchon
   - `WiFi` (built-in with ESP32)

## ⚙️ Configuration

### Step 1: ESP32 WiFi Setup
Edit `esp32/animatronic_eye_controller.ino`:
```cpp
const char* ssid = "YOUR_WIFI_NAME";          // Your WiFi network name
const char* password = "YOUR_WIFI_PASSWORD";  // Your WiFi password
```

### Step 2: Safety Calibration ⚠️
**CRITICAL STEP**: Test servo ranges before automation!

1. **Uncomment the test line** in ESP32 code:
   ```cpp
   // In setup() function, uncomment this line:
   testServos(); // UNCOMMENT THIS FOR FIRST-TIME SETUP!
   ```

2. **Upload code** and **watch Serial Monitor** (115200 baud)

3. **Observe each servo carefully**:
   - Stop immediately if any binding/stress occurs
   - Note safe movement ranges for each servo
   - Adjust `servoLimits` array in code accordingly

4. **Update servo limits** to match YOUR hardware:
   ```cpp
   const ServoLimits servoLimits[NUM_SERVOS] = {
     {60, 120, 90},   // Servo 0: Adjust min/max to YOUR safe range!
     {60, 120, 90},   // Servo 1: Test and adjust!
     {70, 110, 90},   // Servo 2: This one may need very limited range
     {60, 120, 90},   // Servo 3: Adjust to YOUR hardware!
     {0, 180, 90},    // Base servos usually safe for full range
     {0, 180, 90},    
     {0, 180, 90}     
   };
   ```

5. **Re-comment the test line** and upload final code

### Step 3: Network Configuration
1. **Upload ESP32 code** and check Serial Monitor for IP address
2. **Update Python client** with ESP32's IP:
   ```python
   ESP32_IP = "192.168.1.100"  # Replace with YOUR ESP32's actual IP!
   ```

## 🚀 Running the System

### Step 1: Start ESP32
1. **Power on ESP32** with servo connections
2. **Check Serial Monitor** for successful WiFi connection
3. **Note the IP address** displayed

### Step 2: Start Eye Tracker
```bash
cd python
python eye_tracker.py
```

### Step 3: Calibration Process
1. **Position yourself** 2-3 feet from camera
2. **Look directly at the camera**
3. **Press 'C'** to calibrate center position
4. **System is now tracking** your eye movements!

## 🎮 Controls

### During Operation
- **'C'**: Calibrate center (look at camera first)
- **'Q'**: Quit application
- **Spacebar**: Emergency stop (stops all servos immediately)

### Visual Indicators
- **Blue rectangles**: Detected faces
- **Green rectangles**: Detected eyes
- **Red dots**: Eye centers
- **White crosshair**: Calibrated center
- **Gray box**: Dead zone (no movement area)

## 🛠️ Troubleshooting

### Hardware Issues
**Problem**: Servo binding/jerky movement
- **STOP IMMEDIATELY** and disconnect servo power
- Check servo limits in ESP32 code
- Test individual servos manually
- Reduce movement ranges

**Problem**: ESP32 won't connect to WiFi
- Verify WiFi credentials in code
- Ensure 2.4GHz network (ESP32 doesn't support 5GHz)
- Check distance to router

### Software Issues  
**Problem**: Camera not detected
- Try different camera index in Python code
- Check camera permissions
- Ensure no other app is using camera

**Problem**: Eye detection not working
- Improve lighting on your face
- Adjust distance from camera (2-3 feet optimal)
- Recalibrate with 'C' key

**Problem**: Connection to ESP32 fails
- Verify ESP32 IP address in Python code
- Check both devices on same network
- Check Windows firewall settings

## 🎯 Performance Tuning

### Sensitivity Adjustment
In `eye_tracker.py`, modify these values:
```python
self.dead_zone = 30          # Larger = less sensitive
self.smoothing_window = 5    # Larger = smoother movement  
self.sensitivity = 0.8       # 0.1-1.0, lower = less movement
self.command_interval = 0.1  # Faster = more responsive
```

### Servo Movement Tuning
In ESP32 code, adjust movement ranges:
```cpp
// Conservative eye servo movement (±15°)
int eyeServo0 = servoLimits[0].centerAngle + (gazeX * 15);

// More aggressive base servo movement (±45°)  
int basePan = servoLimits[4].centerAngle + (gazeX * 45);
```

## 🆘 Emergency Procedures

### If Hardware Shows Stress:
1. **Press spacebar** in Python app (emergency stop)
2. **Disconnect servo power** immediately
3. **Reset ESP32** if needed
4. **Check all connections** and servo limits
5. **Reduce movement ranges** before restarting

### Recovery Steps:
1. **Test each servo individually** 
2. **Update servo limits** to safer ranges
3. **Restart system** with new limits
4. **Gradually increase** ranges if needed

## 📊 System Architecture

```
[PC Camera] → [OpenCV] → [Eye Tracking] → [WiFi] → [ESP32] → [Servos]
     ↓           ↓           ↓             ↓        ↓         ↓
 Video Feed  Face/Eye    Gaze Vector   Commands  PWM     Physical
            Detection                              Signals  Movement
```

## 🔜 Future Enhancements

- **Advanced blink patterns** (sleepy eyes, expressions)
- **Sound reactive movement** (look toward sound source)
- **Multiple face tracking** (switch between people)
- **Mobile app control** (manual servo positioning)
- **Preset expressions** (angry, happy, surprised)

## 📞 Support

If you encounter issues:

1. **Check Serial Monitor** output from ESP32
2. **Verify all wiring** connections  
3. **Test servo ranges manually** before automation
4. **Start with minimal movement ranges** and increase gradually

**Remember**: Hardware safety is more important than perfect tracking!

## 🏆 Credits

Created for animatronic enthusiasts who prioritize safety and smooth operation.

**Version**: 1.0 - Safety First Edition
**License**: MIT
**Author**: Animatronic Eye Tracking System