#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "INA3221.h"

// ESP32 Dev Module - MQTT LED Control
#define LED_PIN 2  // Built-in LED on GPIO2
#define LED2_PIN 13  // External LED on GPIO13
#define LED4_PIN 14  // External LED on GPIO14
#define POWER_STATUS_PIN 27  // GPIO27 for power status (HIGH=normal, LOW=power cut)

// Light intensity input pins (3-bit binary)
#define INTENSITY_B0_PIN 32  // Bit 0 (LSB)
#define INTENSITY_B1_PIN 33  // Bit 1
#define INTENSITY_B2_PIN 35  // Bit 2 (MSB)

// WiFi credentials
const char* ssid = "Chamix";
const char* password = "12345678";

// MQTT Broker settings
const char* mqtt_broker = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/led/control";
const char* mqtt_status_topic = "esp32/led/status";
const char* mqtt_led2_topic = "esp32/led2/control";
const char* mqtt_led2_status_topic = "esp32/led2/status";
const char* mqtt_led4_topic = "esp32/led4/control";
const char* mqtt_led4_status_topic = "esp32/led4/status";
const char* mqtt_sensor_voltage_topic = "esp32/sensor/voltage";
const char* mqtt_sensor_current_topic = "esp32/sensor/current";
const char* mqtt_sensor2_voltage_topic = "esp32/sensor2/voltage";
const char* mqtt_sensor2_current_topic = "esp32/sensor2/current";
const char* mqtt_powercut_topic = "esp32/powercut/status";
const char* mqtt_command_status_topic = "esp32/command/status";
const char* mqtt_emergency_light_topic = "esp32/emergency/control";
const char* mqtt_emergency_light_status_topic = "esp32/emergency/status";
const char* mqtt_powercut_history_topic = "esp32/history/powercut";
const char* mqtt_light_intensity_topic = "esp32/light/intensity";
const char* mqtt_client_id = "ESP32_LED_Controller";

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

// INA3221 Sensor - Initialize at address 0x40
INA3221 INA(0x40);

// Power cut detection variables
bool powerCutDetected = false;
const float POWER_CUT_THRESHOLD = 9.0;  // Voltage threshold for power cut detection

// Emergency mode variables
bool emergencyModeActive = false;
unsigned long emergencyStartTime = 0;
unsigned long gpio14ActivationTime = 0;
const unsigned long EMERGENCY_DURATION = 60000;  // 1 minute in milliseconds
const unsigned long GPIO14_DELAY = 200;  // 200ms delay for GPIO14
bool gpio14Activated = false;

// Manual emergency light control
bool manualEmergencyControl = false;

// Light intensity tracking
float currentLightIntensity = 100.0;  // Default to 100% (bright)

// Power cut history tracking
unsigned long powerCutStartTime = 0;
float totalEnergyConsumed = 0;
float startVoltage = 0;
float endVoltage = 0;
unsigned long lastEnergyCalcTime = 0;

// Function to publish command status updates
void publishCommandStatus(const char* message) {
  mqtt_client.publish(mqtt_command_status_topic, message);
  Serial.println(message);
}

void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    Serial.print(".");
  }
  
  Serial.println("\n✓ WiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Message: ");
  Serial.println(message);
  
  if (String(topic) == mqtt_topic) {
    if (message == "ON" || message == "1") {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED turned ON");
      mqtt_client.publish(mqtt_status_topic, "ON");
    } 
    else if (message == "OFF" || message == "0") {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED turned OFF");
      mqtt_client.publish(mqtt_status_topic, "OFF");
    }
  }
  else if (String(topic) == mqtt_led2_topic) {
    Serial.println(">>> LED2 topic matched!");
    Serial.print(">>> Command: ");
    Serial.println(message);
    
    if (message == "ON" || message == "1") {
      digitalWrite(LED2_PIN, LOW);
      Serial.println(">>> LED2 (GPIO13) turned ON (LOW)");
      Serial.print(">>> GPIO13 state: ");
      Serial.println(digitalRead(LED2_PIN));
    } 
    else if (message == "OFF" || message == "0") {
      digitalWrite(LED2_PIN, HIGH);
      Serial.println(">>> LED2 (GPIO13) turned OFF (HIGH)");
      Serial.print(">>> GPIO13 state: ");
      Serial.println(digitalRead(LED2_PIN));
      
      // Send pulse(s) to GPIO14 after 1 second delay
      delay(1000);
      
      // Check if GPIO14 is already OFF (HIGH state)
      if (digitalRead(LED4_PIN) == HIGH) {
        Serial.println(">>> Average Intensity is OFF - Sending TWO pulses...");
        
        // First pulse
        Serial.println(">>> Sending GPIO14 pulse #1...");
        digitalWrite(LED4_PIN, LOW);
        delayMicroseconds(10);
        digitalWrite(LED4_PIN, HIGH);
        Serial.println(">>> GPIO14 pulse #1 completed");
        
        // Gap between pulses
        delay(100);
        
        // Second pulse
        Serial.println(">>> Sending GPIO14 pulse #2...");
        digitalWrite(LED4_PIN, LOW);
        delayMicroseconds(10);
        digitalWrite(LED4_PIN, HIGH);
        Serial.println(">>> GPIO14 pulse #2 completed");
      } else {
        Serial.println(">>> Average Intensity is ON - Sending single pulse...");
        digitalWrite(LED4_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(LED4_PIN, LOW);
        Serial.println(">>> GPIO14 pulse completed");
      }
    }
    else if (message == "PULSE") {
      Serial.println(">>> LED2 (GPIO13) PULSE command received");
      digitalWrite(LED2_PIN, HIGH);
      Serial.println(">>> GPIO13 HIGH (pulse started)");
      delay(2000);
      digitalWrite(LED2_PIN, LOW);
      Serial.println(">>> GPIO13 LOW (pulse ended)");
    }
  }
  else if (String(topic) == mqtt_led4_topic) {
    Serial.println(">>> LED4 topic matched!");
    Serial.print(">>> Command: ");
    Serial.println(message);
    
    if (message == "ON" || message == "1" || message == "OFF" || message == "0") {
      // Send pulse for both ON and OFF commands
      Serial.println(">>> LED4 (GPIO14) sending pulse...");
      digitalWrite(LED4_PIN, LOW);   // Inverted: LOW = active pulse
      Serial.println(">>> GPIO14 LOW (pulse started)");
      delayMicroseconds(10);  // Very short pulse - 10 microseconds
      digitalWrite(LED4_PIN, HIGH);  // Return to HIGH (inactive)
      Serial.println(">>> GPIO14 HIGH (pulse ended)");
      Serial.print(">>> GPIO14 final state: ");
      Serial.println(digitalRead(LED4_PIN));
    }
    else if (message == "PULSE") {
      Serial.println(">>> LED4 (GPIO14) PULSE command received");
      digitalWrite(LED4_PIN, HIGH);
      Serial.println(">>> GPIO14 HIGH (pulse started)");
      delay(2000);
      digitalWrite(LED4_PIN, LOW);
      Serial.println(">>> GPIO14 LOW (pulse ended)");
    }
  }
  else if (String(topic) == mqtt_emergency_light_topic) {
    Serial.println(">>> Emergency Light topic matched!");
    Serial.print(">>> Command: ");
    Serial.println(message);
    
    manualEmergencyControl = true;  // Enable manual control
    
    if (message == "ON" || message == "1") {
      digitalWrite(POWER_STATUS_PIN, LOW);  // LOW = Light ON
      mqtt_client.publish(mqtt_emergency_light_status_topic, "ON");
      Serial.println(">>> Emergency Light (GPIO27) turned ON manually (LOW)");
      Serial.print(">>> GPIO27 state: ");
      Serial.println(digitalRead(POWER_STATUS_PIN));
    } 
    else if (message == "OFF" || message == "0") {
      digitalWrite(POWER_STATUS_PIN, HIGH);  // HIGH = Light OFF
      mqtt_client.publish(mqtt_emergency_light_status_topic, "OFF");
      Serial.println(">>> Emergency Light (GPIO27) turned OFF manually (HIGH)");
      Serial.print(">>> GPIO27 state: ");
      Serial.println(digitalRead(POWER_STATUS_PIN));
    }
    else if (message == "AUTO") {
      manualEmergencyControl = false;  // Return to automatic control
      Serial.println(">>> Emergency Light returned to AUTO mode");
      mqtt_client.publish(mqtt_emergency_light_status_topic, "AUTO");
    }
  }
}

void connectMQTT() {
  while (!mqtt_client.connected()) {
    Serial.print("Connecting to MQTT broker: ");
    Serial.println(mqtt_broker);
    
    if (mqtt_client.connect(mqtt_client_id)) {
      Serial.println("✓ MQTT Connected!");
      
      mqtt_client.subscribe(mqtt_topic);
      Serial.print("Subscribed to: ");
      Serial.println(mqtt_topic);
      
      mqtt_client.subscribe(mqtt_led2_topic);
      Serial.print("Subscribed to: ");
      Serial.println(mqtt_led2_topic);
      
      mqtt_client.subscribe(mqtt_led4_topic);
      Serial.print("Subscribed to: ");
      Serial.println(mqtt_led4_topic);
      
      mqtt_client.subscribe(mqtt_emergency_light_topic);
      Serial.print("Subscribed to: ");
      Serial.println(mqtt_emergency_light_topic);
      
      mqtt_client.publish(mqtt_status_topic, "OFF");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32 MQTT LED Controller ===");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED2_PIN, HIGH);  // System starts OFF
  
  pinMode(LED4_PIN, OUTPUT);
  digitalWrite(LED4_PIN, HIGH);  // Average Intensity starts OFF
  
  pinMode(POWER_STATUS_PIN, OUTPUT);
  digitalWrite(POWER_STATUS_PIN, HIGH);  // Normal state: HIGH (no power cut)
  // Configure light intensity input pins
  pinMode(INTENSITY_B0_PIN, INPUT);
  pinMode(INTENSITY_B1_PIN, INPUT);
  pinMode(INTENSITY_B2_PIN, INPUT);

  
  // Initialize I2C for INA3221 sensor (ESP32 Default SDA=21, SCL=22)
  Wire.begin();
  
  Serial.println("Initializing INA3221 Sensor...");
  if (!INA.begin()) {
    Serial.println("ERROR: Could not find INA3221. Check wiring/address!");
  } else {
    Serial.println("✓ INA3221 Sensor Connected!");
    // Set shunt resistor (R100 = 0.1 Ohm)
    INA.setShuntR(0, 0.1);
    INA.setShuntR(1, 0.1);
    INA.setShuntR(2, 0.1);
  }
  
  connectWiFi();
  
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  
  connectMQTT();
  
  Serial.println("============================");
  Serial.println("Ready to receive commands!");
  Serial.println("============================\n");
}

void loop() {
  if (!mqtt_client.connected()) {
    connectMQTT();
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  
  mqtt_client.loop();
  
  // Read and publish light intensity every 2 seconds
  static unsigned long lastIntensityRead = 0;
  if (millis() - lastIntensityRead >= 2000) {
    lastIntensityRead = millis();
    
    // Read 3-bit binary input (B0, B1, B2)
    int b0 = digitalRead(INTENSITY_B0_PIN);
    int b1 = digitalRead(INTENSITY_B1_PIN);
    int b2 = digitalRead(INTENSITY_B2_PIN);
    
    // Calculate value (0-7) from 3 bits
    int intensityValue = (b2 << 2) | (b1 << 1) | b0;
    
    // Convert to percentage (0-7 maps to 0-100%)
    // Using formula: percentage = (value / 7) * 100
    float intensityPercentage = (intensityValue / 7.0) * 100.0;
    
    // Update global light intensity
    currentLightIntensity = intensityPercentage;
    
    // Publish light intensity percentage
    char intensityStr[10];
    dtostrf(intensityPercentage, 5, 1, intensityStr);
    mqtt_client.publish(mqtt_light_intensity_topic, intensityStr);
    
    Serial.printf("Light Intensity: B2=%d B1=%d B0=%d -> Value=%d -> %.1f%%\n", 
                  b2, b1, b0, intensityValue, intensityPercentage);
  }
  
  // Read and display INA3221 sensor data every 2 seconds
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead >= 2000) {
    lastSensorRead = millis();
    
    Serial.println("\n--- Channel Measurements ---");
    
    // Read Channel 1 data
    float voltage1 = INA.getBusVoltage(0);
    float current1 = INA.getCurrent_mA(0);
    
    Serial.printf("CH1: %.3f V | %.2f mA\n", voltage1, current1);
    
    // Publish Channel 1 to MQTT topics
    char voltageStr[10];
    char currentStr[10];
    dtostrf(voltage1, 5, 3, voltageStr);
    dtostrf(current1, 6, 2, currentStr);
    
    mqtt_client.publish(mqtt_sensor_voltage_topic, voltageStr);
    mqtt_client.publish(mqtt_sensor_current_topic, currentStr);
    
    // Read Channel 2 data (Main Power)
    float voltage2 = INA.getBusVoltage(1);
    float current2 = INA.getCurrent_mA(1);
    
    Serial.printf("CH2: %.3f V | %.2f mA\n", voltage2, current2);
    
    // Publish Channel 2 to MQTT topics
    char voltage2Str[10];
    char current2Str[10];
    dtostrf(voltage2, 5, 3, voltage2Str);
    dtostrf(current2, 6, 2, current2Str);
    
    mqtt_client.publish(mqtt_sensor2_voltage_topic, voltage2Str);
    mqtt_client.publish(mqtt_sensor2_current_topic, current2Str);
    
    // Accumulate energy consumption during power cut (from battery - Channel 1)
    if (powerCutDetected) {
      unsigned long currentTime = millis();
      float timeDelta = (currentTime - lastEnergyCalcTime) / 1000.0; // Convert to seconds
      float power = voltage1 * (current1 / 1000.0); // Power in watts (current in mA -> A)
      float energyIncrement = power * (timeDelta / 3600.0); // Energy in Wh
      totalEnergyConsumed += energyIncrement * 1000.0; // Convert to mWh
      lastEnergyCalcTime = currentTime;
    }
    
    // Power cut detection logic
    if (voltage2 < POWER_CUT_THRESHOLD) {
      // Power cut detected
      if (!powerCutDetected) {
        powerCutDetected = true;
        emergencyModeActive = true;
        emergencyStartTime = millis();
        gpio14Activated = false;
        
        // Track power cut history
        powerCutStartTime = millis();
        lastEnergyCalcTime = millis();
        startVoltage = voltage1;
        totalEnergyConsumed = 0;
        
        // GPIO27 will be controlled after 1 minute based on light intensity
        mqtt_client.publish(mqtt_powercut_topic, "POWER_CUT");
        publishCommandStatus("⚠️ POWER CUT DETECTED! Starting emergency sequence...");
        
        // Immediately turn ON GPIO13 (System) - set LOW
        digitalWrite(LED2_PIN, LOW);
        mqtt_client.publish(mqtt_led2_status_topic, "ON");
        publishCommandStatus("✓ Step 1: GPIO13 (System) turned ON");
        
        // Schedule GPIO14 activation
        gpio14ActivationTime = millis() + GPIO14_DELAY;
        
        Serial.println("⚠️ POWER CUT DETECTED! Emergency mode activated.");
      }
    } else {
      // Normal power
      if (powerCutDetected) {
        powerCutDetected = false;
        
        // Calculate power cut history
        unsigned long duration = millis() - powerCutStartTime;
        endVoltage = voltage1;
        float voltageDrop = startVoltage - endVoltage;
        
        // Create history data JSON string
        char historyData[200];
        snprintf(historyData, sizeof(historyData), 
                 "{\"duration\":%lu,\"startV\":%.2f,\"endV\":%.2f,\"drop\":%.2f,\"energy\":%.2f}",
                 duration, startVoltage, endVoltage, voltageDrop, totalEnergyConsumed);
        
        mqtt_client.publish(mqtt_powercut_history_topic, historyData);
        Serial.print("Power cut history: ");
        Serial.println(historyData);
        
        // Only control GPIO27 if not in manual mode
        if (!manualEmergencyControl) {
          digitalWrite(POWER_STATUS_PIN, HIGH);  // Set GPIO27 to HIGH
          mqtt_client.publish(mqtt_emergency_light_status_topic, "OFF");
        }
        mqtt_client.publish(mqtt_powercut_topic, "NORMAL");
        
        // Clear the command log on webpage
        mqtt_client.publish(mqtt_command_status_topic, "CLEAR_LOG");
        delay(50);  // Small delay to ensure CLEAR_LOG is processed first
        
        publishCommandStatus("✓ Power restored. Voltage normal.");
        Serial.println("✓ Power restored. Voltage normal.");
        
        // If emergency mode is still active, turn off all systems
        if (emergencyModeActive) {
          digitalWrite(LED2_PIN, HIGH);  // Inverted: HIGH = OFF
          digitalWrite(LED4_PIN, HIGH);  // Inverted: HIGH = OFF
          
          mqtt_client.publish(mqtt_led2_status_topic, "OFF");
          mqtt_client.publish(mqtt_led4_status_topic, "OFF");
          
          publishCommandStatus("✓ Power restored early - GPIO13 turned OFF");
          publishCommandStatus("✓ GPIO14 turned OFF");
          publishCommandStatus("🔴 Emergency sequence interrupted. All systems OFF.");
          
          emergencyModeActive = false;
          Serial.println("Emergency mode interrupted by power restoration. All systems OFF.");
        }
      }
    }
  }
  
  // Emergency mode management
  if (emergencyModeActive) {
    unsigned long currentTime = millis();
    
    // Activate GPIO14 after 200ms delay
    if (!gpio14Activated && (currentTime >= gpio14ActivationTime)) {
      // Send pulse to GPIO14 (toggle)
      digitalWrite(LED4_PIN, LOW);  // Pulse start
      delayMicroseconds(10);  // Very short pulse
      digitalWrite(LED4_PIN, HIGH);  // Pulse end
      mqtt_client.publish(mqtt_led4_status_topic, "PULSE");
      publishCommandStatus("✓ Step 2: GPIO14 (Average Intensity) pulse sent (200ms delay)");
      gpio14Activated = true;
    }
    
    // Check if 1 minute has elapsed
    if (currentTime - emergencyStartTime >= EMERGENCY_DURATION) {
      // Check light intensity and control GPIO27 (emergency light)
      if (currentLightIntensity < 40.0 && !manualEmergencyControl) {
        digitalWrite(POWER_STATUS_PIN, LOW);  // Turn ON emergency light (LOW = ON)
        mqtt_client.publish(mqtt_emergency_light_status_topic, "ON");
        publishCommandStatus("💡 Light intensity < 40% - Emergency light turned ON");
        Serial.printf("Emergency light ON (Intensity: %.1f%%)\n", currentLightIntensity);
      } else if (!manualEmergencyControl) {
        publishCommandStatus("☀️ Light intensity sufficient (%.1f%%) - Emergency light not needed");
        Serial.printf("Emergency light not needed (Intensity: %.1f%%)\n", currentLightIntensity);
      }
      
      // Turn off both GPIO13 and GPIO14 - both set HIGH
      digitalWrite(LED2_PIN, HIGH);   // Inverted: HIGH = OFF
      digitalWrite(LED4_PIN, HIGH);   // Inverted: HIGH = OFF
      
      mqtt_client.publish(mqtt_led2_status_topic, "OFF");
      mqtt_client.publish(mqtt_led4_status_topic, "OFF");
      
      publishCommandStatus("✓ Step 3: 1 minute elapsed - GPIO13 turned OFF");
      publishCommandStatus("✓ Step 4: GPIO14 turned OFF");
      publishCommandStatus("🔴 Emergency sequence completed. Systems powered OFF.");
      
      emergencyModeActive = false;
      Serial.println("Emergency mode ended. All systems OFF.");
    }
  }
}

