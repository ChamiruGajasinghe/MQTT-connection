# ESP32 Light Intensity & Power Backup Web Dashboard

Real-time web dashboard for monitoring and controlling the ESP32 IoT system with power management and light intensity monitoring.

---

## 📋 Overview

A comprehensive Node.js web application with real-time MQTT communication for:
- Light intensity monitoring (0-100%)
- Dual power source monitoring (battery + main power)
- Emergency power cut management
- Manual system controls
- Power cut history tracking
- Live data visualization

---

## 🚀 Quick Start

### Prerequisites

- Node.js (v14 or higher)
- npm (comes with Node.js)
- Running ESP32 with firmware

### Installation

1. Navigate to web directory:
```bash
cd "C:\Users\chami\OneDrive\Desktop\Projects\IOT\MQTT Connection\web"
```

2. Install dependencies:
```bash
npm install
```

3. Start the server:
```bash
npm start
```

4. Access the dashboard:
```
http://localhost:3000
```

---

## 📁 File Structure

```
web/
├── README.md                 # This file
├── package.json             # Dependencies and scripts
├── server.js                # Node.js Express server
├── dashboard.html           # Main monitoring dashboard
├── history.html             # Power cut history viewer
├── global-control.html      # Remote control interface
└── public/
    └── index.html          # Landing page
```

---

## 🌐 Available Pages

### Main Dashboard (`/dashboard.html`)
- Real-time light intensity gauge (0-100%)
- Battery voltage and current monitoring
- Main power voltage and current monitoring
- Power calculations (V × A = W)
- System status controls
- Emergency light control
- Live charts (Chart.js)
- Power cut alerts
- Command log viewer
- Dark/Light theme toggle

### History Page (`/history.html`)
- Power cut history table
- Event timestamps
- Duration tracking
- Voltage drop analysis
- Energy consumption per event
- Export to CSV
- Local storage persistence

### Global Control (`/global-control.html`)
- Simplified remote control interface
- Toggle switches for all outputs
- Status indicators
- Minimal design for quick access

---

## 📡 MQTT Integration

### Broker Configuration

```javascript
const MQTT_BROKER = 'wss://broker.hivemq.com:8884/mqtt';
```

### Subscribed Topics

The dashboard automatically subscribes to:
- `esp32/led/status` - Built-in LED status
- `esp32/led2/status` - System control status
- `esp32/led4/status` - Intensity control status
- `esp32/emergency/status` - Emergency light status
- `esp32/sensor/voltage` - Battery voltage
- `esp32/sensor/current` - Battery current
- `esp32/sensor2/voltage` - Main power voltage
- `esp32/sensor2/current` - Main power current
- `esp32/light/intensity` - Light intensity percentage
- `esp32/powercut/status` - Power cut alerts
- `esp32/command/status` - Command logs
- `esp32/history/powercut` - History data

### Published Topics

User actions publish to:
- `esp32/led/control` - Control built-in LED
- `esp32/led2/control` - Control system (GPIO13)
- `esp32/led4/control` - Control intensity (GPIO14)
- `esp32/emergency/control` - Control emergency light

---

## 🎨 Features

### Real-Time Monitoring
- ✅ Light intensity percentage with gauge
- ✅ Voltage and current displays
- ✅ Power calculations
- ✅ Connection status indicator
- ✅ Live updating charts

### Control Interface
- ✅ Toggle switches for all outputs
- ✅ Manual ON/OFF controls
- ✅ Emergency light override
- ✅ Real-time feedback
- ✅ Status indicators (colored dots)

### Alerts & Notifications
- ✅ Power cut detection alerts
- ✅ Browser notifications
- ✅ Alert sound (Web Audio API)
- ✅ Low voltage warnings (< 10V)
- ✅ Visual warning banners

### Data Visualization
- ✅ Real-time line charts (Chart.js)
- ✅ Dual voltage comparison
- ✅ Time-series data (last 20 points)
- ✅ Auto-scrolling charts
- ✅ Responsive design

### User Experience
- ✅ Dark/Light theme toggle
- ✅ Responsive mobile design
- ✅ Command log with timestamps
- ✅ Clear log button
- ✅ Test alert system
- ✅ Auto-reconnection

---

## 🛠️ Dependencies

### package.json

```json
{
  "name": "esp32-mqtt-led-control",
  "version": "1.0.0",
  "dependencies": {
    "express": "^4.18.2",
    "mqtt": "^4.3.7"
  },
  "scripts": {
    "start": "node server.js"
  }
}
```

### External Libraries (CDN)

- **Chart.js** - Data visualization
- **MQTT.js** - Browser MQTT client
- **Font Awesome** - Icons

---

## 🎛️ Server Configuration

### server.js

```javascript
const express = require('express');
const app = express();
const PORT = 3000;

app.use(express.static(__dirname));
app.use('/public', express.static('public'));

app.listen(PORT, () => {
    console.log('Server running on http://localhost:3000');
});
```

### Network Access

The server provides both local and network access:
- **Local**: http://localhost:3000
- **Network**: http://[YOUR_IP]:3000
- **Dashboard**: http://localhost:3000/dashboard.html
- **History**: http://localhost:3000/history.html
- **Control**: http://localhost:3000/global-control.html

---

## 📊 Dashboard Components

### Light Intensity Gauge
- Displays 0-100% light level
- Updates every 2 seconds
- Large easy-to-read display
- Unit changed from Lux to %

### Power Monitoring Cards
- **Backup Power (Channel 1)**
  - Voltage display (V)
  - Current display (mA)
  - Power calculation (W)
  
- **Main Power (Channel 2)**
  - Voltage display (V)
  - Current display (mA)
  - Power calculation (W)
  - Power cut detection

### Control Switches
- **System Control** (GPIO13)
  - Toggle ON/OFF
  - Real-time status
  - Color indicators

- **Average Intensity** (GPIO14)
  - Toggle ON/OFF
  - Real-time status
  - Color indicators

- **Emergency Light** (GPIO27)
  - Manual override: ON/OFF/AUTO
  - Status indicators
  - Auto mode respects light intensity

### Charts
- Dual-line chart showing:
  - Battery voltage (blue line)
  - Main power voltage (red line)
  - Time-series data
  - Auto-scrolling (last 20 points)

---

## 🔔 Alert System

### Power Cut Alert
- Visual banner (red background)
- Browser notification
- Alert sound (beep)
- Automatic display when voltage2 < 9V

### Low Voltage Warning
- Triggers when battery < 10V
- Warning message
- Helps prevent battery damage

### Test Alert Button
- Manually test notification system
- Requests browser permission
- Plays test sound

---

## 💾 Data Persistence

### LocalStorage Usage
- Power cut history stored in browser
- Survives page refresh
- Accessible from history page
- JSON format storage

### History Data Structure
```javascript
{
  timestamp: "12/23/2025, 10:30:45 AM",
  duration: 120000,      // milliseconds
  startV: 12.5,         // volts
  endV: 11.8,           // volts
  drop: 0.7,            // volts
  energy: 245.3         // mWh
}
```

---

## 🎨 Theme System

### Dark Mode
- Dark background (#1A1D2E)
- Light text (#ECF0F1)
- Reduced eye strain

### Light Mode
- Light background (#F5F7FA)
- Dark text (#2C3E50)
- Better for bright environments

### Toggle
- Icon changes (moon/sun)
- Smooth transitions
- Preference saved in browser

---

## 🔧 Troubleshooting

### Server Won't Start
```bash
# Check if port 3000 is in use
netstat -ano | findstr :3000

# Kill process if needed
taskkill /PID <PID> /F

# Try alternative port
# Edit server.js: const PORT = 3001;
```

### MQTT Connection Failed
- Check internet connection
- Verify broker URL: wss://broker.hivemq.com:8884/mqtt
- Check browser console for errors
- Try different browser

### Dashboard Not Updating
- Verify ESP32 is connected
- Check MQTT topics in browser console
- Ensure ESP32 is publishing data
- Check WiFi connection on ESP32

### Charts Not Showing
- Check Chart.js CDN link
- Verify browser JavaScript is enabled
- Check browser console for errors
- Try clearing browser cache

---

## 🌍 Remote Access Options

### 1. Port Forwarding
1. Get public IP: https://whatismyipaddress.com
2. Forward port 3000 in router
3. Access via: http://[PUBLIC_IP]:3000

### 2. Tunneling Services
- **ngrok**: `ngrok http 3000`
- **localtunnel**: `lt --port 3000`
- **serveo.net**: `ssh -R 80:localhost:3000 serveo.net`

### 3. Cloud Deployment
- Deploy to Heroku, Railway, or Render
- Configure WebSocket support
- Set environment variables

---

## 📱 Mobile Responsiveness

The dashboard is fully responsive:
- ✅ Works on smartphones
- ✅ Works on tablets
- ✅ Touch-friendly controls
- ✅ Adaptive layouts
- ✅ Optimized for small screens

---

## 🔐 Security Notes

### Current Setup (Development)
- No authentication required
- Public MQTT broker
- Local network access only

### Production Recommendations
- Add user authentication
- Use private MQTT broker
- Enable HTTPS
- Add API rate limiting
- Implement CORS policies

---

## 🚀 Future Enhancements

- [ ] User authentication system
- [ ] Database integration (MongoDB/PostgreSQL)
- [ ] Advanced analytics dashboard
- [ ] Email/SMS notifications
- [ ] Mobile app (React Native/Flutter)
- [ ] Multi-device support
- [ ] Data export (CSV/PDF reports)
- [ ] Scheduling system
- [ ] Energy cost calculator

---

## 📖 API Reference

### MQTT Message Formats

**Control Messages** (Web → ESP32):
```javascript
// ON/OFF commands
"ON" or "OFF"

// Pulse command
"PULSE"

// Emergency modes
"AUTO" or "MANUAL"
```

**Status Messages** (ESP32 → Web):
```javascript
// LED status
"ON" or "OFF"

// Sensor values
"12.345"  // Float as string

// Power cut status
"POWER_CUT" or "NORMAL"

// History data (JSON)
"{\"duration\":120000,\"startV\":12.5,...}"
```

---

## 🐛 Known Issues

1. **Chart Performance**: May slow down with many data points
2. **Browser Notifications**: Requires user permission
3. **Mobile Audio**: Alert sounds may not work on iOS
4. **LocalStorage Limit**: 5-10MB history storage limit

---

## 📝 License

Educational and personal use.

---

## 👤 Author

**Chami**  
Web Dashboard Development  
December 2025

---

## 🔗 Related Documentation

- [Main Project README](../README.md)
- [ESP32 Firmware README](../../Mqtt%20connection/README.md)
- [Complete Project Documentation](../PROJECT_DOCUMENTATION.md)
3. Click "Turn OFF" button to turn off ESP32 LED
4. LED indicator on webpage shows current status

## MQTT Topics

- **Control Topic**: `esp32/led/control`
  - Publish "ON" or "OFF" to control LED
  
- **Status Topic**: `esp32/led/status`
  - ESP32 publishes current LED state

## MQTT Broker

Using public broker: **broker.hivemq.com:1883**

For production, consider using:
- Your own MQTT broker (Mosquitto)
- Cloud MQTT service (HiveMQ Cloud, AWS IoT, etc.)

## Architecture

```
Web Browser (Frontend)
    ↓ HTTP
Web Server (Node.js + Express)
    ↓ MQTT (publish to esp32/led/control)
MQTT Broker (broker.hivemq.com)
    ↓ MQTT (subscribe to esp32/led/control)
ESP32 Dev Module
    → Controls GPIO2 LED
    → Publishes status to esp32/led/status
```

## Troubleshooting

### ESP32 not connecting to MQTT
- Check WiFi credentials in esp32_main.cpp
- Verify internet connection
- Check serial monitor for error messages

### Web interface not working
- Ensure server is running (npm start)
- Check console for errors (F12 in browser)
- Verify MQTT broker is accessible

### LED not responding
- Check ESP32 serial monitor for received messages
- Verify MQTT topic names match
- Ensure ESP32 is still connected to MQTT broker
