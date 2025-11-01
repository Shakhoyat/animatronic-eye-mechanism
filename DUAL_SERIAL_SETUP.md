# Dual Serial Setup Guide

## Overview
This setup allows you to:
1. **Send data** from your laptop to ESP32 using PySerial (via USB-to-Serial adapter)
2. **Monitor debug output** in Arduino IDE Serial Monitor simultaneously

## Hardware Configuration

### ESP32 Connections

#### Serial (USB) - Arduino IDE Monitor
- **USB cable** connected to your computer
- Used for: Debug output, monitoring servo values
- Baud rate: **115200**

#### Serial1 (UART1) - Python PySerial Input
- **RX (GPIO 16)** - Connect to TX of USB-to-Serial adapter
- **TX (GPIO 17)** - Not used (optional)
- **GND** - Connect to GND of USB-to-Serial adapter
- Used for: Receiving JSON data from Python
- Baud rate: **115200**

### Wiring Diagram
```
Laptop/PC
   │
   ├─── USB Cable ────────────────► ESP32 USB Port (Serial)
   │                                  └─► Arduino IDE Monitor
   │
   └─── USB-to-Serial Adapter ────► GPIO16 (RX1) + GND (Serial1)
                                      └─► Python PySerial
```

## Software Setup

### Option 1: Using USB-to-Serial Adapter (Recommended)

**Required Hardware:**
- USB-to-Serial adapter (FTDI, CP2102, CH340, etc.)

**Wiring:**
1. USB-to-Serial TX → ESP32 GPIO16 (RX1)
2. USB-to-Serial GND → ESP32 GND
3. ESP32 USB → Computer (for Arduino IDE monitor)

**Python Configuration:**
Update the COM port in your Python script to match the USB-to-Serial adapter port (NOT the ESP32's main port).

Example:
```python
# Find your USB-to-Serial adapter port
# Windows: Usually COM3, COM4, COM5, etc.
# Check Device Manager → Ports (COM & LPT)

ser = serial.Serial('COM5', 115200, timeout=1)  # Replace COM5 with your adapter's port
```

### Option 2: Without Additional Adapter (Software Only)

If you don't have a USB-to-Serial adapter, you can:

1. **Run Python script first** → Sends data to ESP32 via USB
2. **Close Python script** → Release the COM port
3. **Open Arduino IDE Serial Monitor** → View debug output

**Note:** You cannot do both simultaneously without additional hardware.

## Testing the Setup

### Step 1: Upload ESP32 Code
1. Open `esp32_receiver.ino` in Arduino IDE
2. Connect ESP32 via USB
3. Select the correct COM port (e.g., COM3)
4. Upload the code

### Step 2: Open Arduino Serial Monitor
1. Tools → Serial Monitor
2. Set baud rate to **115200**
3. You should see:
   ```
   ESP32 Eye Tracking Receiver Starting...
   === Dual Serial Mode ===
   Serial (USB): Arduino IDE Monitor
   Serial1 (UART1): Python PySerial
   Serial1 initialized on RX=GPIO16, TX=GPIO17
   ```

### Step 3: Connect USB-to-Serial Adapter
1. Connect adapter's TX to ESP32 GPIO16
2. Connect adapter's GND to ESP32 GND
3. Note the adapter's COM port (e.g., COM5)

### Step 4: Run Python Script
1. Update `eye track.py` to use the adapter's COM port
2. Run the Python script
3. Watch the Arduino Serial Monitor - you should see JSON data being received and parsed

## Troubleshooting

### "Port already in use" Error
- **Cause:** Both Python and Arduino IDE trying to use the same COM port
- **Solution:** Make sure Python uses the USB-to-Serial adapter port, NOT the ESP32's main USB port

### No Data Received
- **Check wiring:** Ensure TX → RX1 (GPIO16) and GND → GND
- **Check COM port:** Verify Python is using the correct adapter port
- **Check baud rate:** Both must be 115200

### Garbage Characters
- **Cause:** Baud rate mismatch
- **Solution:** Ensure both Serial and Serial1 are set to 115200

## Benefits of This Setup

✅ **Real-time debugging** - See what ESP32 receives and processes
✅ **No port conflicts** - Python and Arduino IDE use different ports
✅ **Live monitoring** - Watch servo values update in real-time
✅ **Easy troubleshooting** - Debug messages visible while system runs

## Alternative: Software-Only Solution

If you don't have a USB-to-Serial adapter, modify the code to revert to single serial:

```cpp
// In loop(), change Serial1 back to Serial
if (Serial.available()) {
    // ... read from Serial
}
```

But remember: You can't use Arduino Serial Monitor and Python simultaneously without the adapter.
