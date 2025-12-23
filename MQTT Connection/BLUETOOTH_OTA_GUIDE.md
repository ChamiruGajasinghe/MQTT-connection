# ESP32 Bluetooth OTA Update Guide

## Overview
Your ESP32 is now equipped with **Bluetooth OTA (Over-The-Air)** update capability. You can update firmware wirelessly in two ways:

---

## Method 1: WiFi OTA Update (Recommended)

### Using Arduino IDE:
1. **Open Arduino IDE**
2. Go to **Tools > Port**
3. You should see **"ESP32-LED-Controller at [IP_ADDRESS]"** in the network ports
4. Select this network port
5. Click **Upload** button
6. When prompted, enter password: `esp32update`
7. Upload will proceed wirelessly!

### Using PlatformIO:
```bash
platformio run --target upload --upload-port ESP32-LED-Controller.local
```
Or use the IP address directly:
```bash
platformio run --target upload --upload-port 192.168.8.110
```

---

## Method 2: Bluetooth Serial Monitor

### Features Available via Bluetooth:
Your ESP32 broadcasts as: **"ESP32-LED-Controller"**

**Available Commands:**
- `STATUS` - Get current system status (WiFi IP, MQTT status, GPIO states)
- `RESTART` - Restart the ESP32

### How to Connect:
1. **Windows:**
   - Pair with "ESP32-LED-Controller" in Bluetooth settings
   - Use Serial Bluetooth Terminal app or Putty with the COM port

2. **Android:**
   - Download "Serial Bluetooth Terminal" app
   - Connect to "ESP32-LED-Controller"
   - Send commands: STATUS or RESTART

3. **iOS:**
   - Use "BLE Terminal" or similar app
   - Connect to "ESP32-LED-Controller"

---

## OTA Configuration Details

### Current Settings:
- **Hostname:** `ESP32-LED-Controller`
- **OTA Password:** `esp32update`
- **Bluetooth Name:** `ESP32-LED-Controller`

### To Change Settings:
Edit in [esp32_main.cpp](../../../Documents/PlatformIO/Projects/Mqtt%20connection/src/esp32_main.cpp):
```cpp
ArduinoOTA.setHostname("YOUR-NAME-HERE");
ArduinoOTA.setPassword("your-password");
SerialBT.begin("YOUR-BT-NAME");
```

---

## Troubleshooting

### OTA Not Showing Up:
1. Ensure ESP32 is connected to WiFi
2. Check Serial Monitor for "✓ OTA Ready" message
3. ESP32 and computer must be on same network
4. Try pinging: `ping ESP32-LED-Controller.local`

### Bluetooth Not Connecting:
1. Make sure Bluetooth is enabled on your device
2. Check if device is already paired/connected elsewhere
3. Restart ESP32: Send "RESTART" command or power cycle
4. Check Serial Monitor for "✓ Bluetooth Started" message

### Upload Fails:
1. Check OTA password is correct: `esp32update`
2. Ensure no firewall blocking port 3232
3. Try uploading via USB (COM4) first
4. Restart ESP32 and try again

---

## Security Notes

⚠️ **Important:** 
- Change the default OTA password `esp32update` for production use
- Bluetooth Serial has no password protection by default
- Consider disabling Bluetooth after initial setup if not needed

---

## Technical Details

### What was added:
- **ArduinoOTA library** - Handles wireless firmware updates
- **BluetoothSerial** - Enables Bluetooth communication
- **OTA progress callbacks** - Shows update progress via Serial and Bluetooth
- **Bluetooth command handler** - Processes STATUS and RESTART commands

### Memory Usage:
- Flash: 57.3% (750,573 bytes)
- RAM: 13.7% (44,964 bytes)

### Update Process:
1. OTA client connects to ESP32
2. Authentication with password
3. Firmware transferred in chunks
4. Progress shown on Serial & Bluetooth
5. ESP32 automatically restarts with new firmware

---

## Next Steps

To update firmware wirelessly:
1. Make code changes
2. Select network port in IDE
3. Click Upload
4. Enter password when prompted
5. Wait for update to complete

No USB cable needed! 🎉
