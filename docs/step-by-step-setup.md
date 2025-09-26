# 🚀 Step-by-Step Setup Guide

Follow this guide **exactly** to set up your animatronic eye tracking system safely.

## 📋 Before You Begin

**REQUIRED ITEMS:**
- ESP32 development board
- 7 servo motors (micro servos for eyes + standard servos for base)
- External 5V power supply (2A minimum)
- Jumper wires and breadboard
- PC with webcam
- WiFi network (2.4GHz)

**⚠️ SAFETY WARNING**: This guide prioritizes hardware safety. Never skip safety steps!

---

## 🔌 STEP 1: Hardware Assembly

### 1.1 Power Supply Setup
```
⚠️ CRITICAL: Use external power for servos!

1. Connect 5V power supply POSITIVE to breadboard red rail
2. Connect 5V power supply NEGATIVE to breadboard blue rail  
3. Connect ESP32 GND pin to breadboard blue rail (common ground)
4. Keep ESP32 powered via USB for now
```

### 1.2 Servo Connections
**Connect servos to breadboard and ESP32 as follows:**

```
Servo 0 (Eye Mechanism):
- Red wire    → Breadboard +5V rail
- Brown wire  → Breadboard GND rail  
- Orange wire → ESP32 Pin 2

Servo 1 (Eye Mechanism):
- Red wire    → Breadboard +5V rail
- Brown wire  → Breadboard GND rail
- Orange wire → ESP32 Pin 4

Servo 2 (Eye Mechanism):  
- Red wire    → Breadboard +5V rail
- Brown wire  → Breadboard GND rail
- Orange wire → ESP32 Pin 5

Servo 3 (Eye Mechanism):
- Red wire    → Breadboard +5V rail  
- Brown wire  → Breadboard GND rail
- Orange wire → ESP32 Pin 18

Servo 4 (Base Pan):
- Red wire    → Breadboard +5V rail
- Brown wire  → Breadboard GND rail
- Orange wire → ESP32 Pin 19

Servo 5 (Base Tilt):
- Red wire    → Breadboard +5V rail
- Brown wire  → Breadboard GND rail  
- Orange wire → ESP32 Pin 21

Servo 6 (Base Roll):
- Red wire    → Breadboard +5V rail
- Brown wire  → Breadboard GND rail
- Orange wire → ESP32 Pin 22
```

### 1.3 Final Wiring Check
- [ ] All servo red wires to +5V rail
- [ ] All servo brown/black wires to GND rail
- [ ] All servo signal wires to correct ESP32 pins
- [ ] ESP32 GND connected to common ground
- [ ] No loose connections
- [ ] No short circuits between power rails

---

## 💻 STEP 2: Software Installation

### 2.1 Arduino IDE Setup
1. **Download and install Arduino IDE** from arduino.cc
2. **Add ESP32 board support:**
   - File → Preferences
   - Additional Board Manager URLs: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Tools → Board → Boards Manager
   - Search "ESP32" and install

### 2.2 Install Required Libraries
In Arduino IDE Library Manager (Sketch → Include Library → Manage Libraries):
- Search and install: **"ESP32Servo"** by Kevin Harrington
- Search and install: **"ArduinoJson"** by Benoit Blanchon

### 2.3 Python Installation
Open PowerShell and run:

```bash
cd c:\Users\skt_pie\Music\animatronic-eye-mechanism\python
pip install -r requirements.txt
```

---

## ⚙️ STEP 3: ESP32 Configuration

### 3.1 Update WiFi Credentials
1. Open `esp32/animatronic_eye_controller.ino` in Arduino IDE
2. Find these lines and update with YOUR WiFi info:
```cpp
const char* ssid = "YOUR_WIFI_NAME";          // Replace with actual name
const char* password = "YOUR_WIFI_PASSWORD";  // Replace with actual password
```

### 3.2 Enable Servo Testing (CRITICAL STEP)
Find this line in the `setup()` function:
```cpp
// testServos(); // UNCOMMENT THIS FOR FIRST-TIME SETUP!
```

**Uncomment it to:**
```cpp
testServos(); // UNCOMMENT THIS FOR FIRST-TIME SETUP!
```

### 3.3 Upload ESP32 Code
1. Select board: Tools → Board → ESP32 Arduino → ESP32 Dev Module  
2. Select port: Tools → Port → (your ESP32 port)
3. Upload the code: Sketch → Upload

---

## 🔧 STEP 4: Critical Safety Testing

### 4.1 Power On and Monitor
1. **Connect external 5V power** to breadboard
2. **Connect ESP32 via USB**
3. **Open Serial Monitor** (Tools → Serial Monitor, set to 115200 baud)
4. **Watch for WiFi connection confirmation**

### 4.2 Servo Safety Testing
**⚠️ WATCH CAREFULLY - STOP IF ANY SERVO BINDS!**

The ESP32 will automatically test each servo:
1. **Servo 0-6 will be tested individually**
2. **Each servo moves: center → min → max → center**
3. **IMMEDIATELY STOP if you see:**
   - Servo binding or struggling
   - Unusual noises
   - Mechanical stress
   - Overheating

**If any problems occur:**
1. **Disconnect power immediately**  
2. **Check mechanical mounting**
3. **Adjust servo limits** (next step)

### 4.3 Adjust Servo Limits (If Needed)
If any servo showed stress during testing:

1. **Open Arduino code**
2. **Find the `servoLimits` array**
3. **Reduce the range** for problematic servos:

```cpp
const ServoLimits servoLimits[NUM_SERVOS] = {
  {70, 110, 90},   // Servo 0: Reduced range if needed
  {70, 110, 90},   // Servo 1: Adjust based on your hardware
  {80, 100, 90},   // Servo 2: Very conservative if this one binds
  {70, 110, 90},   // Servo 3: Adjust as needed
  {0, 180, 90},    // Base servos usually OK for full range
  {0, 180, 90},    
  {0, 180, 90}     
};
```

4. **Upload updated code** and test again

### 4.4 Disable Testing Mode
Once all servos move safely:
1. **Re-comment the test line:**
```cpp
// testServos(); // Testing complete - commented out
```
2. **Upload final code**

---

## 🌐 STEP 5: Network Configuration

### 5.1 Get ESP32 IP Address
1. **Check Serial Monitor** after ESP32 connects to WiFi
2. **Note the IP address displayed** (example: 192.168.1.100)

### 5.2 Update Python Client
1. **Open `python/eye_tracker.py`**
2. **Find this line:**
```python
ESP32_IP = "192.168.1.100"  # UPDATE THIS TO YOUR ESP32's IP!
```
3. **Replace with your actual ESP32 IP address**

---

## 🎯 STEP 6: System Testing

### 6.1 Start the Eye Tracker
1. **Open PowerShell in project directory:**
```bash
cd c:\Users\skt_pie\Music\animatronic-eye-mechanism\python
python eye_tracker.py
```

2. **Check connection status** in camera window

### 6.2 Calibrate Eye Tracking
1. **Position yourself** 2-3 feet from camera
2. **Look directly at the camera**
3. **Press 'C' key** to calibrate center position
4. **You should see "Calibration complete!" message**

### 6.3 Test Eye Movements
1. **Move your eyes left/right** - servos should follow
2. **Move your eyes up/down** - servos should follow  
3. **Blink** - should see "BLINK!" indicator
4. **Press SPACEBAR** to test emergency stop

---

## ✅ STEP 7: Final Verification

### 7.1 System Health Check
- [ ] All servos moving smoothly
- [ ] No binding or unusual noises
- [ ] Eye tracking responds accurately
- [ ] Emergency stop works (spacebar)
- [ ] Network connection stable

### 7.2 Performance Tuning (Optional)
If movement is too sensitive/aggressive:

**In Python (`eye_tracker.py`):**
```python
self.dead_zone = 40         # Increase for less sensitivity
self.sensitivity = 0.6      # Reduce for smaller movements
```

**In ESP32 code:**
```cpp
int eyeServo0 = servoLimits[0].centerAngle + (gazeX * 10);  // Reduce multiplier
```

---

## 🎮 Usage Instructions

### Controls:
- **'C'**: Calibrate (look at camera first)
- **'Q'**: Quit application
- **Spacebar**: Emergency stop

### Status Indicators:
- **🟢 Connected**: ESP32 communication OK
- **🔴 Disconnected**: Network issues
- **BLINK!**: Blink detection active
- **Gaze X/Y**: Current eye direction

---

## 🆘 Troubleshooting

### Common Issues:

**"No camera found"**
- Check camera connections
- Try different camera index in code
- Close other camera applications

**"ESP32 connection failed"**  
- Verify IP address in Python code
- Check WiFi connection on both devices
- Check Windows firewall settings

**"Servo not moving smoothly"**
- Check power supply capacity (2A minimum)
- Verify all ground connections
- Reduce movement ranges in code

**"Eye detection not working"**
- Improve lighting on face
- Adjust distance from camera
- Recalibrate with 'C' key

---

## 🏆 Success!

If you've completed all steps successfully:
- ✅ Hardware is safely connected
- ✅ Software is properly configured  
- ✅ System responds to eye movements
- ✅ Emergency stops work

**Congratulations!** Your animatronic eye tracking system is ready to use!

## 🔜 Next Steps

- Fine-tune sensitivity settings
- Add custom movement behaviors
- Integrate with other animatronic features
- Create preset expressions

**Remember**: Always prioritize hardware safety over perfect performance!

---

**Setup Completed On**: ________________  
**Notes**: ____________________________