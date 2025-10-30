# Eye Tracking Animatronic System - Status

## ✅ SYSTEM READY FOR HARDWARE TESTING

### Current Configuration: 7 Servo System

#### Hardware Layout
1. **Eyeball Movement (Fast tracking)**
   - Servo 1 (GPIO 13): Left/Right movement (40-140°)
   - Servo 2 (GPIO 12): Up/Down movement (40-140°)

2. **Eyelids (Blink control)**
   - Servo 3 (GPIO 14): Top eyelid (10-170°)
   - Servo 4 (GPIO 27): Bottom eyelid (10-170°)

3. **Face Tilt (Natural head movement)**
   - Servo 5 (GPIO 26): Left/Right tilt (70-110°)
   - Servo 6 (GPIO 25): Up/Down tilt (70-110°)

4. **Head Rotation**
   - Servo 7 (GPIO 33): Left/Right head turn (45-135°)

### Software Status

#### ✅ PC Side (`pc_eye_tracker.py`)
- **Status**: Complete and tested
- **Features**:
  - Eye tracking with MediaPipe Face Mesh
  - Blink detection with EAR algorithm
  - Smart head movement calculation (tilt follows eyeball with 30% intensity)
  - Head rotation follows horizontal gaze (50% intensity)
  - Live camera view window
  - Calibration mode
  - Serial communication at 115200 baud

#### ✅ ESP32 Side (`esp32_receiver.ino`)
- **Status**: Code complete, ready for upload
- **Features**:
  - 7 servo control with smooth movement
  - Dual smoothing (fast eyeballs, slower head movements)
  - Blink animation (150ms duration)
  - Timeout protection (returns to neutral after 1 second)
  - Serial command parsing

### Communication Protocol
```
Format: "ELR:90,EUD:90,TLR:90,TUD:90,HR:90,BL:0\n"

Where:
- ELR: Eyeball Left/Right (40-140)
- EUD: Eyeball Up/Down (40-140)
- TLR: Tilt Left/Right (70-110)
- TUD: Tilt Up/Down (70-110)
- HR: Head Rotate (45-135)
- BL: Blink (0 or 1)
```

### Movement Logic

#### Eyeball Tracking
- **Direct tracking**: Eyes follow camera input 1:1
- **Response**: Fast (smoothing = 0.3)
- **Range**: ±50° from center

#### Face Tilt
- **Follows eyeball**: Moves 30% of eyeball movement
- **Response**: Slower (smoothing = 0.15) for natural motion
- **Range**: ±20° from center (less extreme than eyeballs)
- **Example**: If eyes look 30° right, head tilts 9° right

#### Head Rotation
- **Follows horizontal gaze**: Rotates 50% of eyeball left/right
- **Response**: Slower (smoothing = 0.15)
- **Range**: ±45° from center
- **Example**: If eyes look 40° left, head rotates 20° left

### Next Steps

#### 1. Upload ESP32 Code
```
1. Open Arduino IDE
2. File → Open → esp32_receiver.ino
3. Tools → Board → ESP32 Dev Module
4. Tools → Port → (select your ESP32 COM port)
5. Click Upload button
```

#### 2. Test Serial Communication
```powershell
# Run PC tracker
python pc_eye_tracker.py

# Should see:
# - Camera window opens
# - Eye tracking active
# - Serial commands sent to ESP32
```

#### 3. Manual Servo Test
Open Serial Monitor (115200 baud) and send test commands:
```
ELR:90,EUD:90,TLR:90,TUD:90,HR:90,BL:0    # All servos center
ELR:120,EUD:90,TLR:90,TUD:90,HR:90,BL:0   # Eyes look right
ELR:60,EUD:90,TLR:90,TUD:90,HR:90,BL:0    # Eyes look left
ELR:90,EUD:60,TLR:90,TUD:90,HR:90,BL:0    # Eyes look up
ELR:90,EUD:120,TLR:90,TUD:90,HR:90,BL:0   # Eyes look down
ELR:90,EUD:90,TLR:90,TUD:90,HR:90,BL:1    # Blink
```

#### 4. Calibration
See `CALIBRATION_GUIDE.md` for detailed servo adjustment procedures.

### Troubleshooting

#### No servo movement
- Check 5V power supply connected to servos
- Verify ESP32 ground connected to servo ground
- Check GPIO pin connections match code

#### Jerky movement
- Increase smoothing factors in esp32_receiver.ino:
  ```cpp
  const float smoothing = 0.2;      // Lower = smoother (0.3 default)
  const float smoothingHead = 0.1;  // Lower = smoother (0.15 default)
  ```

#### Servo jitter when idle
- Reduce servo limits to avoid mechanical binding
- Check power supply provides adequate current (2A minimum)

#### Eyes/head not coordinated
- Adjust multipliers in pc_eye_tracker.py:
  ```python
  tilt_lr = int(90 + (lr_angle - 90) * 0.3)    # Change 0.3 to adjust
  head_offset = (lr_angle - 90) * 0.5          # Change 0.5 to adjust
  ```

### Dependencies Verified

#### Python (PC)
- ✅ Python 3.10
- ✅ opencv-python
- ✅ mediapipe 0.10.9
- ✅ pyserial
- ✅ numpy 1.23.5
- ✅ protobuf 3.20.3

#### Arduino (ESP32)
- ✅ ESP32 board support
- ✅ ESP32Servo library

### Files Ready
- ✅ `pc_eye_tracker.py` - PC eye tracking (tested)
- ✅ `esp32_receiver.ino` - ESP32 servo control (ready to upload)
- ✅ `ARDUINO_SETUP_GUIDE.md` - Arduino IDE setup
- ✅ `CALIBRATION_GUIDE.md` - Hardware calibration
- ✅ `QUICK_START.md` - Wiring diagram
- ✅ `EYE_TRACKING_README.md` - Quick reference

---

**System is ready for hardware testing! Upload the ESP32 code and connect your servos.**
