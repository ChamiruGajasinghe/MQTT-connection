# 🚀 HOW TO RUN - ESP32 IoT Monitoring System

Complete step-by-step guide to set up and run the ESP32 Light Intensity & Power Backup System with web dashboard.

---

## 📋 Table of Contents

1. [System Requirements](#system-requirements)
2. [Hardware Setup](#hardware-setup)
3. [Software Installation](#software-installation)
4. [ESP32 Firmware Upload](#esp32-firmware-upload)
5. [Web Dashboard Setup](#web-dashboard-setup)
6. [Running the System](#running-the-system)
7. [Testing & Verification](#testing--verification)
8. [Troubleshooting](#troubleshooting)
9. [Advanced Configuration](#advanced-configuration)

---

## 📦 System Requirements

### Hardware
- **ESP32 Dev Module** (38-pin board with ESP32-D0WD-V3 chip)
- **INA3221 Triple-Channel Power Monitor** (I2C address 0x40)
- **3-bit Light Intensity Sensor** (binary input)
- **USB Cable** (Type-A to Micro-USB)
- **Power supplies** for battery backup and main power monitoring
- **Optional**: External LEDs for GPIO 13 and GPIO 14

### Software
- **Windows OS** (current setup)
- **Visual Studio Code** with PlatformIO extension
- **Node.js** (v14 or higher) - Download from [nodejs.org](https://nodejs.org/)
- **Git** (optional, for version control)
- **Web Browser** (Chrome, Firefox, or Edge)

### Network
- **WiFi Network** with internet access
- **MQTT Broker**: HiveMQ public cloud (broker.hivemq.com)

---

## 🔧 Hardware Setup

### Step 1: Wire the ESP32

#### Pin Connections

| Component | ESP32 Pin | GPIO | Description |
|-----------|-----------|------|-------------|
| Built-in LED | - | GPIO 2 | Status indicator |
| System Control | - | GPIO 13 | External LED (inverted logic) |
| Intensity Control | - | GPIO 14 | External LED (inverted logic) |
| Emergency Light | - | GPIO 27 | Emergency light output |
| Light Sensor B0 | - | GPIO 32 | Intensity bit 0 (LSB) |
| Light Sensor B1 | - | GPIO 33 | Intensity bit 1 |
| Light Sensor B2 | - | GPIO 35 | Intensity bit 2 (MSB) |
| INA3221 SDA | SDA | GPIO 21 | I2C Data |
| INA3221 SCL | SCL | GPIO 22 | I2C Clock |
| INA3221 VCC | 3.3V | - | Power supply |
| INA3221 GND | GND | - | Ground |

#### Light Intensity Encoding
```
Binary Input → Percentage
000 (0) → 0%
001 (1) → 14.3%
010 (2) → 28.6%
011 (3) → 42.9%
100 (4) → 57.1%
101 (5) → 71.4%
110 (6) → 85.7%
111 (7) → 100%
```

### Step 2: Connect INA3221 Channels

- **Channel 1**: Connect to battery backup power source (voltage + current shunt)
- **Channel 2**: Connect to main power source (voltage + current shunt)

### Step 3: Connect USB Cable

- Plug the ESP32 into your computer via USB
- Note the COM port (e.g., COM4)

---

## 💻 Software Installation

### Step 1: Install Visual Studio Code

1. Download from [code.visualstudio.com](https://code.visualstudio.com/)
2. Install with default settings

### Step 2: Install PlatformIO Extension

1. Open VS Code
2. Click Extensions icon (left sidebar)
3. Search for "PlatformIO IDE"
4. Click **Install**
5. Wait for installation to complete (may take several minutes)
6. Restart VS Code if prompted

### Step 3: Install Node.js

1. Download from [nodejs.org](https://nodejs.org/) (LTS version recommended)
2. Run installer
3. Accept default settings
4. Verify installation:
   ```powershell
   node --version
   npm --version
   ```

---

## 📤 ESP32 Firmware Upload

### Step 1: Open Project in PlatformIO

1. Open VS Code
2. Click **PlatformIO Home** icon (bottom toolbar)
3. Select **Open Project**
4. Navigate to:
   ```
   C:\Users\chami\OneDrive\Documents\PlatformIO\Projects\Mqtt connection
   ```
5. Click **Open**

### Step 2: Configure WiFi Credentials

1. Open `src/main.cpp`
2. Find lines 18-19:
   ```cpp
   const char* ssid = "Chamix";
   const char* password = "12345678";
   ```
3. **IMPORTANT**: Change to your WiFi network:
   ```cpp
   const char* ssid = "YOUR_WIFI_NAME";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
4. Save the file (Ctrl+S)

### Step 3: Verify COM Port

1. Open `platformio.ini`
2. Verify the COM port matches your device:
   ```ini
   upload_port = COM4
   monitor_port = COM4
   ```
3. If different, change to your actual COM port
4. Save the file

### Step 4: Build and Upload

1. Click the **PlatformIO** icon (ant icon) in left sidebar
2. Expand **esp32dev** → **General**
3. Click **Build** (checkmark icon) - wait for successful build
4. Click **Upload** (arrow icon) - firmware will upload to ESP32
5. Wait for "SUCCESS" message

### Step 5: Verify Connection

1. Click **Monitor** (plug icon) in PlatformIO
2. Watch for output:
   ```
   Connecting to WiFi: YOUR_WIFI_NAME
   ✓ WiFi Connected!
   IP Address: 192.168.x.x
   ✓ Connected to MQTT Broker
   ```
3. If you see this, the ESP32 is ready!

**Note**: Keep the Serial Monitor open to see real-time logs from the ESP32.

---

## 🌐 Web Dashboard Setup

### Step 1: Navigate to Web Directory

Open PowerShell or Command Prompt:
```powershell
cd "C:\Users\chami\OneDrive\Desktop\Projects\IOT\MQTT Connection\web"
```

### Step 2: Install Dependencies

First time only:
```powershell
npm install
```

This installs:
- Express.js (web server)
- MQTT.js (browser MQTT client)

### Step 3: Start the Web Server

```powershell
npm start
```

You should see:
```
=============================================
🌐 IoT Monitoring System - GLOBAL ACCESS
=============================================
📡 Local Access:    http://localhost:3000
🌍 Network Access:  http://[YOUR_IP]:3000
📊 Dashboard:       http://localhost:3000/dashboard.html
📈 History:         http://localhost:3000/history.html
🎛️  Control:         http://localhost:3000/global-control.html
🔌 MQTT Broker:     wss://broker.hivemq.com:8884/mqtt
=============================================
```

**Keep this terminal window open** - the server must run continuously.

---

## ⚡ Running the System

### Complete Startup Sequence

#### 1. Power On ESP32
- Connect ESP32 via USB or external power
- Built-in LED will blink rapidly during WiFi connection
- LED will stay solid once connected

#### 2. Start Web Server (if not running)
```powershell
cd "C:\Users\chami\OneDrive\Desktop\Projects\IOT\MQTT Connection\web"
npm start
```

#### 3. Access Dashboard

Open your web browser and navigate to:
```
http://localhost:3000/dashboard.html
```

#### 4. Verify System Status

Check the dashboard shows:
- **MQTT Status**: Connected (green)
- **Light Intensity**: Current reading (0-100%)
- **Battery Voltage/Current**: Real-time values
- **Main Power Voltage/Current**: Real-time values
- **Power Status**: Normal (green) or Power Cut (red)

---

## 🎮 Using the Dashboard

### Main Dashboard (`/dashboard.html`)

#### Real-Time Monitoring
- **Light Intensity Gauge**: Visual 0-100% display
- **Battery Monitor**: Voltage (V), Current (A), Power (W)
- **Main Power Monitor**: Voltage (V), Current (A), Power (W)
- **Live Charts**: Historical data visualization

#### Manual Controls
- **Built-in LED**: Toggle ON/OFF (GPIO 2)
- **System Control**: Toggle GPIO 13 (inverted logic)
- **Intensity Control**: Toggle GPIO 14 (inverted logic)
- **Emergency Light**: Manual ON/OFF control (GPIO 27)

#### Features
- **Command Log**: View all MQTT messages
- **Theme Toggle**: Switch between dark/light mode
- **Auto-Scroll**: Follow latest updates
- **Power Cut Alerts**: Visual and text notifications

### History Page (`/history.html`)

View power cut event history:
- Timestamp of each event
- Duration of power cut
- Starting voltage
- Ending voltage
- Voltage drop (ΔV)
- Energy consumed (Wh)
- Export data to CSV

### Global Control (`/global-control.html`)

Simplified remote control interface:
- Quick toggle switches
- Status indicators
- Minimal design for mobile access

---

## 🔍 Testing & Verification

### Test 1: Basic LED Control

1. Open dashboard: `http://localhost:3000/dashboard.html`
2. Click **Built-in LED** toggle → LED on ESP32 should light up
3. Click again → LED should turn off
4. Check Serial Monitor for confirmation messages

### Test 2: Light Intensity Reading

1. Connect 3-bit binary input to GPIO 32, 33, 35
2. Change input values (000 to 111)
3. Dashboard should update with percentage (0-100%)

### Test 3: Power Monitoring

1. Verify INA3221 is properly connected
2. Dashboard should show:
   - Battery voltage and current
   - Main power voltage and current
   - Calculated power (V × A)

### Test 4: Emergency Power Cut Simulation

1. Reduce main power voltage below 9.0V
2. Watch Serial Monitor for:
   ```
   ⚠️ POWER CUT DETECTED!
   Starting emergency sequence...
   GPIO13 ON immediately
   GPIO14 ON after 200ms
   ```
3. Dashboard should show:
   - Power Status: POWER CUT (red)
   - Emergency mode activated
4. After 60 seconds:
   - GPIO13 and GPIO14 turn OFF
   - If light intensity < 40%, Emergency Light (GPIO27) turns ON
5. Restore power > 9.0V → System returns to normal

### Test 5: Manual Emergency Override

1. Click **Emergency Light** toggle on dashboard
2. GPIO27 should turn ON regardless of power status
3. This overrides automatic emergency logic

---

## 🔧 Troubleshooting

### ESP32 Issues

#### Problem: WiFi Not Connecting
```
Solution:
1. Verify SSID and password in main.cpp
2. Check WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
3. Ensure router is broadcasting SSID
4. Move ESP32 closer to router
```

#### Problem: Upload Failed
```
Solution:
1. Check USB cable is data-capable (not charge-only)
2. Verify correct COM port in platformio.ini
3. Hold BOOT button on ESP32 during upload
4. Try different USB port
5. Install/update USB drivers (CP2102 or CH340)
```

#### Problem: MQTT Connection Failed
```
Solution:
1. Verify ESP32 has internet access (ping google.com from router)
2. Check broker.hivemq.com is accessible
3. Restart ESP32 (reset button)
4. Check Serial Monitor for error messages
```

#### Problem: INA3221 Not Detected
```
Solution:
1. Verify I2C connections (SDA=GPIO21, SCL=GPIO22)
2. Check INA3221 power (3.3V and GND)
3. Verify I2C address is 0x40 (default)
4. Test with I2C scanner sketch
```

### Web Dashboard Issues

#### Problem: Web Server Won't Start
```
Solution:
1. Verify Node.js is installed: node --version
2. Run: npm install (in web directory)
3. Check port 3000 is not in use
4. Try different port: PORT=8080 npm start
```

#### Problem: Dashboard Not Updating
```
Solution:
1. Open browser console (F12)
2. Check for WebSocket connection errors
3. Verify MQTT broker is accessible
4. Clear browser cache (Ctrl+Shift+Delete)
5. Check ESP32 is publishing data (Serial Monitor)
```

#### Problem: MQTT Connection Failed (Browser)
```
Solution:
1. Verify internet connection
2. Check browser allows WebSocket connections
3. Try different browser
4. Check firewall settings
5. Verify wss://broker.hivemq.com:8884/mqtt is accessible
```

### Network Issues

#### Problem: Can't Access from Other Devices
```
Solution:
1. Get your computer's local IP:
   PowerShell: ipconfig
   Look for "IPv4 Address"
2. Ensure devices are on same WiFi network
3. Access via: http://[YOUR_IP]:3000
4. Check Windows Firewall allows port 3000
5. Add firewall rule:
   Control Panel → Windows Defender Firewall → Advanced Settings
   → Inbound Rules → New Rule → Port 3000
```

---

## ⚙️ Advanced Configuration

### Change MQTT Topics

Edit `src/main.cpp` (lines 25-40):
```cpp
const char* mqtt_topic = "your/custom/topic";
// Update corresponding topics in dashboard.html
```

### Change WiFi at Runtime

Currently requires re-upload. For WiFi Manager:
1. Add WiFiManager library to platformio.ini
2. Implement captive portal setup

### Enable OTA Updates

See `BLUETOOTH_OTA_GUIDE.md` for Bluetooth OTA setup.

### Adjust Emergency Thresholds

In `src/main.cpp`:
```cpp
const float POWER_CUT_THRESHOLD = 9.0;  // Change voltage threshold
const unsigned long EMERGENCY_DURATION = 60000;  // Change duration (ms)
const unsigned long GPIO14_DELAY = 200;  // Change GPIO14 delay (ms)
```

### Deploy to Cloud

For internet access without port forwarding:

**Option 1: ngrok**
```powershell
# Install ngrok
# Run: ngrok http 3000
# Access via: https://[random].ngrok.io
```

**Option 2: Heroku**
```powershell
# Deploy web server to Heroku
git init
heroku create
git push heroku main
```

**Option 3: Render/Vercel**
- Push code to GitHub
- Deploy from Render.com or Vercel.com

---

## 📊 System Architecture

```
┌─────────────────────┐         ┌───────────────────┐         ┌─────────────────────┐
│   Web Dashboard     │         │   MQTT Broker     │         │       ESP32         │
│  (Any Browser)      │────────▶│ (HiveMQ Cloud)    │────────▶│    + INA3221        │
│  localhost:3000     │         │ broker.hivemq.com │         │    + Sensors        │
│                     │◀────────│                   │◀────────│    GPIO Controls    │
└─────────────────────┘         └───────────────────┘         └─────────────────────┘
     WebSocket                        Internet                      WiFi
   wss://...8884/mqtt                MQTT Port 1883               USB/COM4
```

---

## 📈 MQTT Topics Reference

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `esp32/led/control` | Web → ESP32 | Built-in LED control (ON/OFF) |
| `esp32/led/status` | ESP32 → Web | Built-in LED status |
| `esp32/led2/control` | Web → ESP32 | GPIO13 control (ON/OFF) |
| `esp32/led2/status` | ESP32 → Web | GPIO13 status |
| `esp32/led4/control` | Web → ESP32 | GPIO14 control (ON/OFF) |
| `esp32/led4/status` | ESP32 → Web | GPIO14 status |
| `esp32/emergency/control` | Web → ESP32 | Emergency light control |
| `esp32/emergency/status` | ESP32 → Web | Emergency light status |
| `esp32/light/intensity` | ESP32 → Web | Light intensity (0-100%) |
| `esp32/sensor/voltage` | ESP32 → Web | Battery voltage (V) |
| `esp32/sensor/current` | ESP32 → Web | Battery current (A) |
| `esp32/sensor2/voltage` | ESP32 → Web | Main power voltage (V) |
| `esp32/sensor2/current` | ESP32 → Web | Main power current (A) |
| `esp32/powercut/status` | ESP32 → Web | Power status (NORMAL/CUT) |
| `esp32/history/powercut` | ESP32 → Web | Power cut event history (JSON) |
| `esp32/command/status` | ESP32 → Web | Command execution status |

---

## 🎯 Quick Reference Commands

### ESP32 Upload
```powershell
# From PlatformIO Projects folder
cd "C:\Users\chami\OneDrive\Documents\PlatformIO\Projects\Mqtt connection"
# Use PlatformIO UI → Upload
```

### Web Server
```powershell
# Start server
cd "C:\Users\chami\OneDrive\Desktop\Projects\IOT\MQTT Connection\web"
npm start

# Install dependencies (first time)
npm install

# Check Node.js version
node --version
```

### Serial Monitor
```powershell
# From PlatformIO UI → Monitor
# OR manually:
# Use Arduino IDE → Tools → Serial Monitor → 115200 baud
```

---

## 📝 Important Notes

1. **Keep Serial Monitor Open**: Essential for debugging and seeing real-time ESP32 logs

2. **Web Server Must Run**: The web server (`npm start`) must be running to access the dashboard

3. **Same Network**: For local access, devices must be on the same WiFi network

4. **Internet Required**: Both ESP32 and web browser need internet to connect to MQTT broker

5. **USB Power**: ESP32 can be powered via USB or external 5V supply

6. **Backup Configuration**: Save your WiFi credentials before uploading new firmware

7. **Emergency Mode**: Automatic emergency light activation only works when main power < 9.0V

8. **Manual Override**: Manual emergency light control overrides automatic logic

---

## 📞 Support & Documentation

- **Project Documentation**: `PROJECT_DOCUMENTATION.md`
- **Arduino Code**: `MQTT Connection/test2/test2.ino`
- **Main Firmware**: `Mqtt connection/src/main.cpp`
- **Web Dashboard**: `MQTT Connection/web/dashboard.html`
- **OTA Guide**: `BLUETOOTH_OTA_GUIDE.md`

---

## ✅ Pre-Flight Checklist

Before running the system:

- [ ] ESP32 properly wired with all sensors connected
- [ ] INA3221 on I2C bus (SDA=21, SCL=22)
- [ ] WiFi credentials updated in `main.cpp`
- [ ] COM port verified in `platformio.ini`
- [ ] Firmware successfully uploaded to ESP32
- [ ] ESP32 shows "WiFi Connected" in Serial Monitor
- [ ] Node.js installed and verified
- [ ] npm dependencies installed in web folder
- [ ] Web server running (`npm start`)
- [ ] Dashboard accessible at `http://localhost:3000`
- [ ] MQTT connection status shows "Connected"
- [ ] Light intensity sensor connected and reading
- [ ] Power monitoring channels showing voltage/current

---

## 🎉 You're Ready!

If all checks pass, your ESP32 IoT Monitoring System is fully operational!

Access your dashboard at: **http://localhost:3000/dashboard.html**

Monitor in real-time, control remotely, and enjoy your intelligent power backup system! 🚀
