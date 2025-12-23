# 🌍 ESP32 Global LED Control System

## 📋 Project Overview

A complete IoT system that allows you to control an ESP32's onboard LED from **anywhere in the world** using a web browser. No port forwarding, no VPN, no local network required!

### Key Features
✅ Control LED from any device (phone, tablet, computer)  
✅ Works from any network (home WiFi, mobile data, office)  
✅ Real-time bidirectional communication via MQTT  
✅ Beautiful responsive web interface  
✅ Connection status indicators  
✅ Automatic reconnection handling  

---

## 🏗️ System Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────┐
│  Web Browser    │────────▶│  MQTT Broker     │────────▶│   ESP32     │
│  (Anywhere)     │         │ (HiveMQ Cloud)   │         │ (Your Home) │
│                 │◀────────│                  │◀────────│             │
└─────────────────┘         └──────────────────┘         └─────────────┘
    Internet                     Internet                   WiFi: Dialog 4G
    Any Network                  Public Cloud               LED: GPIO 2
```

### Communication Flow

1. **User Action**: Click "Turn ON" button in web browser
2. **Web → Broker**: Browser publishes "ON" to `chami/esp32/led/control` topic
3. **Broker → ESP32**: MQTT broker forwards message to ESP32
4. **ESP32**: Receives "ON" command, turns LED on
5. **ESP32 → Broker**: ESP32 publishes "ON" to `chami/esp32/led/status` topic
6. **Broker → Web**: Browser receives status update and shows green indicator

---

## 🔧 Hardware Requirements

### ESP32 Dev Module
- **Model**: ESP32 Dev Module (38-pin board)
- **Chip**: ESP32-D0WD-V3
- **LED**: GPIO 2 (built-in LED)
- **MAC Address**: F4:65:0B:55:31:C0
- **USB Port**: COM4

### Specifications
- CPU: Dual-core 240MHz
- RAM: 320KB
- Flash: 4MB
- WiFi: 802.11 b/g/n (2.4GHz)
- Bluetooth: Classic + BLE (not used in this project)

---

## 🌐 Network Configuration

### WiFi Network
- **SSID**: Dialog 4G 858
- **Password**: 04588A9D
- **Security**: WPA/WPA2
- **Channel**: 6

### MQTT Broker
- **Host**: broker.hivemq.com
- **Port**: 1883 (TCP for ESP32)
- **WebSocket Port**: 8884 (WSS for web browser)
- **Type**: Public cloud broker (no authentication required)

### MQTT Topics
| Topic | Direction | Purpose |
|-------|-----------|---------|
| `chami/esp32/led/control` | Web → ESP32 | Send ON/OFF commands |
| `chami/esp32/led/status` | ESP32 → Web | Receive LED status updates |

---

## 📁 Project Structure

```
MQTT Connection/
├── platformio.ini                      # PlatformIO configuration
├── src/
│   ├── esp32_webserver.cpp            # ✅ ACTIVE: Main MQTT control code
│   ├── esp32_main.cpp                 # ❌ Old MQTT version (not used)
│   ├── esp32c3_main.cpp               # ❌ ESP32-C3 test (WiFi failed)
│   ├── esp32_espnow_gateway.cpp       # ❌ ESP-NOW experiment (abandoned)
│   └── esp32c3_espnow_sender.cpp      # ❌ ESP-NOW experiment (abandoned)
├── web/
│   ├── global-control.html            # ✅ ACTIVE: Web control interface
│   ├── server.js                      # ❌ Old Node.js server (not needed)
│   └── package.json                   # ❌ Old dependencies (not needed)
└── PROJECT_DOCUMENTATION.md           # This file
```

---

## 💻 ESP32 Code Explanation

### File: `esp32_webserver.cpp`

#### Key Components

**1. WiFi Connection**
```cpp
void connectWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);  // Wait until connected
    }
}
```
- Connects ESP32 to home WiFi network
- Blocks until connection successful
- Required for MQTT communication

**2. MQTT Connection**
```cpp
void connectMQTT() {
    String clientId = "ESP32_LED_" + String(random(0xffff), HEX);
    if (mqtt.connect(clientId.c_str())) {
        mqtt.subscribe(mqtt_topic_control);  // Listen for commands
        mqtt.publish(mqtt_topic_status, "OFF");  // Send initial status
    }
}
```
- Generates unique client ID
- Connects to cloud MQTT broker
- Subscribes to control topic
- Publishes initial status

**3. Message Handler (Callback)**
```cpp
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    if (message == "ON") {
        digitalWrite(LED_PIN, HIGH);  // Turn LED on
        mqtt.publish(mqtt_topic_status, "ON");
    } else if (message == "OFF") {
        digitalWrite(LED_PIN, LOW);   // Turn LED off
        mqtt.publish(mqtt_topic_status, "OFF");
    }
}
```
- Automatically called when message arrives
- Converts byte array to string
- Controls LED based on command
- Publishes status update

**4. Main Loop**
```cpp
void loop() {
    if (!mqtt.connected()) {
        connectMQTT();  // Reconnect if disconnected
    }
    mqtt.loop();  // Process MQTT messages
    
    // Send heartbeat every 30 seconds
    if (millis() - lastMsg > 30000) {
        mqtt.publish(mqtt_topic_status, ledState.c_str());
        lastMsg = millis();
    }
}
```
- Maintains MQTT connection
- Processes incoming messages
- Sends periodic status updates

---

## 🌐 Web Interface Explanation

### File: `global-control.html`

#### Architecture
- **HTML5**: Structure
- **CSS3**: Beautiful gradient design with animations
- **JavaScript**: MQTT communication
- **MQTT.js**: WebSocket MQTT library (from CDN)

#### Key Functions

**1. MQTT Connection**
```javascript
function initMQTT() {
    mqttClient = mqtt.connect('wss://broker.hivemq.com:8884/mqtt', {
        clientId: 'WebClient_' + Math.random().toString(16).substr(2, 8),
        clean: true,
        reconnectPeriod: 1000,
    });
}
```
- Connects to MQTT broker via secure WebSocket
- Generates unique client ID
- Auto-reconnects if connection lost

**2. Button Actions**
```javascript
function turnOn() {
    mqttClient.publish('chami/esp32/led/control', 'ON');
}

function turnOff() {
    mqttClient.publish('chami/esp32/led/control', 'OFF');
}
```
- Publishes commands to control topic
- ESP32 receives and executes command

**3. Status Updates**
```javascript
mqttClient.on('message', function(topic, message) {
    if (topic === 'chami/esp32/led/status') {
        updateLEDStatus(message.toString());
    }
});
```
- Listens for status updates from ESP32
- Updates LED indicator and text in real-time

#### UI Elements

| Element | Purpose | States |
|---------|---------|--------|
| Connection Badge | Shows MQTT connection status | 🟢 Connected, 🟡 Connecting, 🔴 Disconnected |
| LED Indicator | Visual LED representation | 🟢 Green glow (ON), 🔴 Red (OFF) |
| Status Text | Text status | "✓ LED is ON" / "LED is OFF" |
| ON Button | Turn LED on | Green gradient, disabled when offline |
| OFF Button | Turn LED off | Red gradient, disabled when offline |

---

## 🚀 How to Use

### Setup (One-time)

1. **Upload Code to ESP32**
   ```bash
   # Open PlatformIO terminal
   pio run -e esp32dev --target upload
   ```

2. **Verify ESP32 Connection**
   ```bash
   # Open serial monitor
   pio device monitor -p COM4 -b 115200
   ```
   
   Expected output:
   ```
   === ESP32 Global MQTT Controller ===
   Connecting to WiFi...
   ✓ WiFi Connected!
   IP Address: 192.168.8.110
   Connecting to MQTT broker... ✓ Connected!
   Subscribed to: chami/esp32/led/control
   ✓ Ready for global control!
   ```

3. **Access Web Interface**
   - **Option A**: Open `global-control.html` file directly
   - **Option B**: Upload to CodePen/GitHub Pages
   - **Option C**: Host on any web server

### Daily Use

1. Make sure ESP32 is powered and connected to WiFi
2. Open web interface from any device
3. Click "Turn ON" or "Turn OFF" buttons
4. Watch LED indicator update in real-time!

---

## 🔍 Troubleshooting

### ESP32 Issues

**Problem**: ESP32 won't connect to WiFi
- ✅ Check WiFi credentials in code
- ✅ Verify router is on and broadcasting
- ✅ Check WiFi is 2.4GHz (ESP32 doesn't support 5GHz)
- ✅ Move ESP32 closer to router

**Problem**: ESP32 won't connect to MQTT broker
- ✅ Check WiFi connection first
- ✅ Verify internet connectivity
- ✅ Check serial monitor for error codes
- ✅ Try restarting ESP32

**Problem**: LED doesn't turn on/off
- ✅ Check GPIO 2 is correct pin (varies by board)
- ✅ Some boards use inverted logic (LOW=ON, HIGH=OFF)
- ✅ Check MQTT messages in serial monitor
- ✅ Verify callback function is being triggered

### Web Interface Issues

**Problem**: "Disconnected" status (red badge)
- ✅ Check internet connection
- ✅ Check browser console for errors (F12)
- ✅ Verify MQTT broker is accessible (broker.hivemq.com)
- ✅ Try refreshing page

**Problem**: Buttons don't work
- ✅ Check connection status is "Connected" (green)
- ✅ Check browser console for JavaScript errors
- ✅ Verify ESP32 is powered and online

**Problem**: LED indicator doesn't update
- ✅ Check ESP32 is publishing status updates
- ✅ Verify subscription to status topic succeeded
- ✅ Check browser console for incoming messages

### Network Issues

**Problem**: Can't control from outside home network
- ✅ Verify using cloud MQTT broker (not local)
- ✅ Check ESP32 has internet access (not just LAN)
- ✅ Confirm web page can reach broker.hivemq.com

---

## 📊 MQTT Message Examples

### Control Messages (Web → ESP32)
```
Topic: chami/esp32/led/control
Payload: "ON"   → Turn LED on
Payload: "OFF"  → Turn LED off
```

### Status Messages (ESP32 → Web)
```
Topic: chami/esp32/led/status
Payload: "ON"   → LED is currently on
Payload: "OFF"  → LED is currently off
```

### Message Flow Example
```
1. User clicks "Turn ON"
2. Web: Publish "ON" to chami/esp32/led/control
3. ESP32: Receive "ON" from chami/esp32/led/control
4. ESP32: digitalWrite(LED_PIN, HIGH)
5. ESP32: Publish "ON" to chami/esp32/led/status
6. Web: Receive "ON" from chami/esp32/led/status
7. Web: Update indicator to green
```

---

## 🔒 Security Considerations

### Current Setup (Public Broker)
⚠️ Using public MQTT broker without authentication:
- Anyone who knows your topic can control your LED
- Messages are not encrypted (except WebSocket SSL)
- Suitable for learning and non-critical projects

### Recommended for Production

1. **Use Private MQTT Broker**
   - Self-hosted Mosquitto
   - AWS IoT Core
   - Azure IoT Hub

2. **Add Authentication**
   ```cpp
   mqtt.connect(clientId, "username", "password");
   ```

3. **Use SSL/TLS**
   ```cpp
   WiFiClientSecure espClient;
   espClient.setCACert(ca_cert);
   ```

4. **Use Unique Topic Names**
   - Replace "chami" with random UUID
   - Keep topic secret

---

## 🎯 Future Enhancements

### Potential Improvements

1. **Multiple LEDs**
   - Control different GPIO pins
   - Separate topics for each LED

2. **Dimming Control**
   - Use PWM for brightness control
   - Slider in web interface

3. **Scheduling**
   - Turn LED on/off at specific times
   - Use ESP32 RTC or cloud scheduling

4. **Status Monitoring**
   - Add temperature sensor
   - Report WiFi signal strength
   - Battery voltage monitoring

5. **Security**
   - Add authentication
   - Use private MQTT broker
   - Implement access control

6. **Multiple Devices**
   - Control multiple ESP32s
   - Device selection in web interface

---

## 📚 Libraries Used

### ESP32 Side
| Library | Version | Purpose |
|---------|---------|---------|
| Arduino.h | Built-in | Core Arduino functions |
| WiFi.h | Built-in | WiFi connectivity |
| PubSubClient | 2.8 | MQTT client |

### Web Side
| Library | Version | Source | Purpose |
|---------|---------|--------|---------|
| MQTT.js | Latest | CDN | WebSocket MQTT client |

---

## 🛠️ Development Tools

### Software
- **PlatformIO**: ESP32 development environment
- **VS Code**: Code editor
- **Arduino Framework**: ESP32 programming framework
- **Git**: Version control (optional)

### Hardware Tools
- **USB Cable**: Micro-USB for ESP32 Dev Module
- **Serial Monitor**: Debugging and status viewing
- **Multimeter**: Testing voltages (optional)

---

## 📖 References

### Documentation
- [PlatformIO Docs](https://docs.platformio.org/)
- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/)
- [PubSubClient Library](https://pubsubclient.knolleary.net/)
- [MQTT.js Documentation](https://github.com/mqttjs/MQTT.js)
- [HiveMQ Public Broker](https://www.hivemq.com/mqtt/public-mqtt-broker/)

### MQTT Protocol
- [MQTT Specification](https://mqtt.org/)
- [MQTT Topics Guide](https://www.hivemq.com/blog/mqtt-essentials-part-5-mqtt-topics-best-practices/)

---

## 📝 License

This project is for educational purposes. Feel free to use, modify, and distribute.

---

## 👤 Author

**Chami**
- ESP32 Dev Module: F4:65:0B:55:31:C0
- WiFi Network: Dialog 4G 858
- Project Date: December 2025

---

## 🎓 Learning Outcomes

By completing this project, you learned:
✅ ESP32 WiFi connectivity  
✅ MQTT publish/subscribe protocol  
✅ Cloud IoT communication  
✅ Web-based device control  
✅ Real-time bidirectional messaging  
✅ Asynchronous programming  
✅ PlatformIO development  
✅ HTML/CSS/JavaScript integration  

---

**🌟 Congratulations on building a global IoT control system! 🌟**
