# 🌟 Light Intensity & Power Backup System

Complete IoT monitoring and control system for intelligent lighting and emergency power management using ESP32, INA3221 sensors, and real-time MQTT communication.

---

## 📋 Project Overview

This system provides:
- **Real-time Light Intensity Monitoring** (3-bit binary input, 0-100%)
- **Dual Power Monitoring** (Battery backup and main power via INA3221)
- **Intelligent Emergency Light Control** (activates only when needed)
- **Automated Emergency Sequence** during power cuts
- **Web Dashboard** for monitoring and control from anywhere
- **Power Cut History Tracking** with energy consumption data

---

## 🏗️ System Architecture

```
┌──────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│  Web Dashboard   │────────▶│  MQTT Broker     │────────▶│     ESP32       │
│  (Any Browser)   │         │ (HiveMQ Cloud)   │         │  + INA3221      │
│                  │◀────────│                  │◀────────│  + Sensors      │
└──────────────────┘         └──────────────────┘         └─────────────────┘
     Internet                     Internet                   Local Network
```

### Key Components

1. **ESP32 Dev Module** - Main controller with WiFi connectivity
2. **INA3221 Sensor** - Dual channel voltage/current monitoring
3. **Light Intensity Sensor** - 3-bit binary input (GPIO 32, 33, 35)
4. **Control Outputs** - GPIO 13 (System), GPIO 14 (Intensity), GPIO 27 (Emergency Light)
5. **Web Dashboard** - Real-time monitoring and control interface
6. **MQTT Broker** - Cloud-based message routing (HiveMQ)

---

## 🔧 Hardware Setup

### ESP32 Pin Configuration

| Pin | Function | Description |
|-----|----------|-------------|
| GPIO 2 | LED_PIN | Built-in LED indicator |
| GPIO 13 | LED2_PIN | System control (inverted logic) |
| GPIO 14 | LED4_PIN | Average intensity control (inverted logic) |
| GPIO 27 | POWER_STATUS_PIN | Emergency light control |
| GPIO 32 | INTENSITY_B0 | Light intensity bit 0 (LSB) |
| GPIO 33 | INTENSITY_B1 | Light intensity bit 1 |
| GPIO 35 | INTENSITY_B2 | Light intensity bit 2 (MSB) |
| GPIO 21 | SDA | I2C data for INA3221 |
| GPIO 22 | SCL | I2C clock for INA3221 |

### INA3221 Sensor Channels

- **Channel 1**: Battery backup monitoring (voltage & current)
- **Channel 2**: Main power monitoring (voltage & current)

### Light Intensity Input

The system reads 3 binary inputs (B0, B1, B2) to determine light intensity:
- **000** (0) → 0%
- **001** (1) → 14.3%
- **010** (2) → 28.6%
- **011** (3) → 42.9%
- **100** (4) → 57.1%
- **101** (5) → 71.4%
- **110** (6) → 85.7%
- **111** (7) → 100%

---

## ⚡ Emergency Power Cut Sequence

When main power voltage drops below 9.0V:

### Timeline:

1. **t = 0ms** - Power cut detected
   - GPIO13 (System) turns ON immediately
   - Emergency mode activated
   - Start tracking power cut history

2. **t = 200ms** - Intensity control activated
   - GPIO14 sends pulse to toggle average intensity

3. **t = 60 seconds** - Emergency evaluation
   - GPIO13 turns OFF
   - GPIO14 turns OFF
   - **Check light intensity:**
     - If intensity < 40% → GPIO27 (Emergency Light) turns ON
     - If intensity ≥ 40% → Emergency light stays OFF (sufficient ambient light)

4. **Power restored** - Normal operation resumes
   - All emergency controls turn OFF
   - Log power cut history (duration, voltage drop, energy consumed)

---

## 🌐 Network Configuration

### WiFi Settings
- **SSID**: Chamix
- **Password**: 12345678

### MQTT Broker
- **Host**: broker.hivemq.com
- **Port**: 1883 (TCP for ESP32)
- **WebSocket**: 8884 (WSS for web)

### MQTT Topics

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `esp32/led/control` | Web → ESP32 | Built-in LED control |
| `esp32/led/status` | ESP32 → Web | Built-in LED status |
| `esp32/led2/control` | Web → ESP32 | System control (GPIO13) |
| `esp32/led2/status` | ESP32 → Web | System status |
| `esp32/led4/control` | Web → ESP32 | Intensity control (GPIO14) |
| `esp32/led4/status` | ESP32 → Web | Intensity status |
| `esp32/emergency/control` | Web → ESP32 | Manual emergency light control |
| `esp32/emergency/status` | ESP32 → Web | Emergency light status |
| `esp32/sensor/voltage` | ESP32 → Web | Battery voltage (Channel 1) |
| `esp32/sensor/current` | ESP32 → Web | Battery current (Channel 1) |
| `esp32/sensor2/voltage` | ESP32 → Web | Main power voltage (Channel 2) |
| `esp32/sensor2/current` | ESP32 → Web | Main power current (Channel 2) |
| `esp32/light/intensity` | ESP32 → Web | Light intensity percentage |
| `esp32/powercut/status` | ESP32 → Web | Power cut alerts |
| `esp32/command/status` | ESP32 → Web | System command logs |
| `esp32/history/powercut` | ESP32 → Web | Power cut history data |

---

## 📁 Project Structure

```
MQTT Connection/
├── README.md                          # This file
├── PROJECT_DOCUMENTATION.md           # Detailed documentation
├── BLUETOOTH_OTA_GUIDE.md            # OTA update guide
├── MQTT Connection.code-workspace    # VS Code workspace
├── web/                              # Web dashboard
│   ├── README.md                     # Web app documentation
│   ├── server.js                     # Node.js server
│   ├── dashboard.html                # Main dashboard
│   ├── history.html                  # Power cut history
│   ├── global-control.html           # Remote control interface
│   ├── package.json                  # Dependencies
│   └── public/
│       └── index.html                # Landing page
└── test2/
    └── test2.ino                     # Arduino test sketch

Mqtt connection/                       # PlatformIO project
├── README.md                          # Firmware documentation
├── platformio.ini                     # PlatformIO config
├── src/
│   └── main.cpp                      # Main ESP32 firmware
├── include/                          # Header files
├── lib/                              # Libraries
└── test/                             # Test files
```

---

## 🚀 Quick Start

### 1. ESP32 Firmware Setup

```bash
cd "C:\Users\chami\OneDrive\Documents\PlatformIO\Projects\Mqtt connection"
pio run -e esp32dev --target upload
pio device monitor
```

### 2. Web Dashboard Setup

```bash
cd "C:\Users\chami\OneDrive\Desktop\Projects\IOT\MQTT Connection\web"
npm install
npm start
```

### 3. Access Dashboard

Open browser: http://localhost:3000/dashboard.html

---

## 📊 Features

### Real-Time Monitoring
- ✅ Light intensity percentage (0-100%)
- ✅ Battery voltage and current
- ✅ Main power voltage and current
- ✅ Power calculations (V × A = W)
- ✅ System status indicators
- ✅ Connection status

### Control Features
- ✅ Manual system control (GPIO13)
- ✅ Manual intensity control (GPIO14)
- ✅ Manual emergency light control (GPIO27)
- ✅ Automatic emergency sequence
- ✅ Toggle switches with real-time feedback

### Data Logging
- ✅ Power cut history with timestamps
- ✅ Duration tracking
- ✅ Voltage drop measurements
- ✅ Energy consumption during outages
- ✅ Browser local storage persistence

### User Interface
- ✅ Dark/Light theme toggle
- ✅ Responsive design (mobile-friendly)
- ✅ Real-time charts (Chart.js)
- ✅ Browser notifications
- ✅ Alert sounds
- ✅ Command log viewer

---

## 🔐 Safety Features

- **Low Voltage Warning**: Alert when battery < 10V
- **Automatic Power Cut Detection**: Threshold at 9.0V
- **Intelligent Light Control**: Emergency light only activates when needed (< 40% ambient light)
- **Energy Tracking**: Monitor battery consumption during outages
- **Reconnection Handling**: Auto-reconnect for WiFi and MQTT
- **Manual Override**: Emergency light can be controlled manually via web interface

---

## 📖 Documentation

- **[Firmware Documentation](../Mqtt%20connection/README.md)** - ESP32 code details
- **[Web Dashboard Documentation](web/README.md)** - Web interface guide
- **[Project Documentation](PROJECT_DOCUMENTATION.md)** - Complete system guide
- **[OTA Update Guide](BLUETOOTH_OTA_GUIDE.md)** - Wireless updates

---

## 🛠️ Development Tools

- **PlatformIO** - Firmware development
- **VS Code** - Code editor
- **Node.js** - Web server
- **Chart.js** - Data visualization
- **MQTT.js** - Browser MQTT client

---

## 📝 License

This project is for educational and personal use.

---

## 👤 Author

**Chami**
- Project: Light Intensity & Power Backup System
- Date: December 2025

---

## 🔗 Resources

- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [PlatformIO](https://platformio.org/)
- [HiveMQ MQTT Broker](https://www.hivemq.com/)
- [INA3221 Datasheet](https://www.ti.com/product/INA3221)
