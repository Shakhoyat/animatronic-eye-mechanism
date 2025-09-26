# 🛡️ Hardware Safety Checklist

**CRITICAL**: Complete this checklist BEFORE powering on your system!

## ✅ Pre-Setup Safety Checklist

### Power Supply Safety
- [ ] **External 5V power supply** for servos (minimum 2A capacity)
- [ ] **Common ground** connection between ESP32 and servo power supply
- [ ] **ESP32 powered separately** via USB during development
- [ ] **All connections secure** and properly insulated
- [ ] **Power supply rated** for total servo load (7 servos × ~300mA = ~2.1A)

### Wiring Verification
- [ ] **Servo signal wires** connected to correct ESP32 pins
- [ ] **Servo power (red) wires** connected to external 5V supply
- [ ] **Servo ground (brown/black) wires** connected to common ground
- [ ] **No short circuits** between power and ground
- [ ] **All connections tight** and properly seated

### Mechanical Safety
- [ ] **Each servo can move freely** in its mounting
- [ ] **No mechanical binding** when moved by hand
- [ ] **Eye mechanism servos** identified and marked (these need limited range)
- [ ] **Base servos** identified and can handle full rotation
- [ ] **Emergency stop accessible** (spacebar key readily available)

### Software Configuration
- [ ] **WiFi credentials** updated in ESP32 code
- [ ] **testServos()** uncommented for initial testing
- [ ] **Servo limits** set to conservative values initially
- [ ] **Python script** has correct ESP32 IP address
- [ ] **All required libraries** installed on both platforms

## ⚠️ First-Time Testing Protocol

### Phase 1: Individual Servo Testing
1. **Power on ESP32** with servos connected
2. **Monitor Serial output** for successful startup
3. **Observe each servo** during automatic test sequence
4. **IMMEDIATELY STOP** if any servo shows:
   - Binding or resistance
   - Unusual noise
   - Excessive heat
   - Mechanical stress

### Phase 2: Range Calibration  
1. **Note safe ranges** for each servo during testing
2. **Update servoLimits array** with conservative values
3. **Re-comment testServos()** line
4. **Upload updated code** with new limits

### Phase 3: System Integration
1. **Start Python eye tracker**
2. **Test network connection** to ESP32
3. **Calibrate eye tracking** with 'C' key
4. **Start with small movements** and increase gradually

## 🚨 Emergency Stop Procedures

### If Hardware Shows Stress:
1. **Press SPACEBAR immediately** (sends emergency stop)
2. **Disconnect servo power supply**
3. **Close Python application** (Ctrl+C or window close)
4. **Reset ESP32** if unresponsive

### Recovery Actions:
1. **Inspect all servos** for damage
2. **Check mechanical mounting** for binding
3. **Reduce servo limits** in code
4. **Test manually** before restarting automation

## 📋 System Limits Reference

### Conservative Starting Limits
```
Servo 0 (Eye): 60°-120° (±30° from center)
Servo 1 (Eye): 60°-120° (±30° from center)  
Servo 2 (Eye): 70°-110° (±20° from center) - Most restrictive
Servo 3 (Eye): 60°-120° (±30° from center)
Servo 4 (Base): 0°-180° (±90° from center)
Servo 5 (Base): 0°-180° (±90° from center)
Servo 6 (Base): 0°-180° (±90° from center)
```

### Warning Signs to Watch For:
- **Binding sounds** (grinding, clicking)
- **Servo overheating** (hot to touch)
- **Jerky movement** (not smooth)
- **Mechanical stress** (parts flexing)
- **Power supply voltage drop** (brown-outs)

## 🔧 Troubleshooting Guide

### Common Issues:

**Servo not responding:**
- Check signal wire connection
- Verify servo limits in code
- Test servo with manual commands

**Servo moving to wrong position:**
- Check servo index in pin array
- Verify servo mounting/orientation
- Calibrate servo center position

**System freezing/crashing:**
- Reduce command frequency
- Check for power supply issues
- Monitor Serial output for errors

**Network connection issues:**
- Verify WiFi credentials
- Check ESP32 IP address
- Ensure firewall not blocking connection

## ✅ Final Safety Verification

Before leaving system unattended:
- [ ] **All servos moving smoothly** within safe ranges
- [ ] **No overheating** of servos or power supply
- [ ] **Emergency stop** tested and working
- [ ] **Movement limits** appropriate for your hardware
- [ ] **Network connection** stable
- [ ] **Mechanical mounting** secure and stable

## 📞 Emergency Contacts

**If you need help:**
1. **Stop the system immediately**
2. **Disconnect power to servos**
3. **Document what happened** (settings, behavior)
4. **Check Serial Monitor** for error messages
5. **Test hardware manually** before restarting

Remember: **Your hardware safety is the top priority!**

---

**Date Completed**: ________________
**Completed By**: ________________  
**Notes**: ____________________________________________________