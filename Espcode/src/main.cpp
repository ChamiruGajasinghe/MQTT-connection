#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "INA3221.h"

#define LED_PIN 2           
#define LED2_PIN 13         
#define LED4_PIN 14         
#define POWER_STATUS_PIN 27 

#define INTENSITY_B0_PIN 32  
#define INTENSITY_B1_PIN 33  
#define INTENSITY_B2_PIN 35  

const char* ssid = "Chamix";
const char* password = "12345678";
const char* mqtt_broker = "broker.hivemq.com";
const int mqtt_port = 1883;

// --- MQTT TOPIC DEFINITIONS ---
const char* mqtt_topic = "esp32/led/control";
const char* mqtt_status_topic = "esp32/led/status";
const char* mqtt_led2_topic = "esp32/led2/control";
const char* mqtt_led2_status_topic = "esp32/led2/status";
const char* mqtt_led4_topic = "esp32/led4/control";
const char* mqtt_led4_status_topic = "esp32/led4/status";

// --- SENSOR TOPICS (Keep only one copy of these!) ---
const char* mqtt_sensor_voltage_topic = "esp32/sensor/voltage";
const char* mqtt_sensor2_voltage_topic = "esp32/sensor2/voltage";

// --- NEW TOPICS FOR CURRENT & POWER ---
const char* mqtt_sensor_current_topic = "esp32/sensor/current";  // 5V System Current
const char* mqtt_sensor_power_topic   = "esp32/sensor/power";    // 5V System Power
const char* mqtt_sensor2_current_topic = "esp32/sensor2/current"; // 12V System Current
const char* mqtt_sensor2_power_topic   = "esp32/sensor2/power";   // 12V System Power
// --------------------------------------

const char* mqtt_powercut_topic = "esp32/powercut/status";
const char* mqtt_command_status_topic = "esp32/command/status";
const char* mqtt_emergency_light_topic = "esp32/emergency/control";
const char* mqtt_emergency_light_status_topic = "esp32/emergency/status";
const char* mqtt_powercut_history_topic = "esp32/history/powercut";
const char* mqtt_light_intensity_topic = "esp32/light/intensity";
const char* mqtt_client_id = "ESP32_Full_Fixed_v2";
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
INA3221 INA(0x40);

bool powerCutDetected = false;
const float POWER_CUT_THRESHOLD = 9.0;
bool emergencyModeActive = false;
unsigned long gpio14ActivationTime = 0;
const unsigned long GPIO14_DELAY = 200;
bool gpio14Activated = false;
bool manualEmergencyControl = false;
float currentLightIntensity = 100.0;
unsigned long powerCutStartTime = 0;
float startVoltage = 0;

// Timer variables
unsigned long emergencyStartTime = 0; // Tracks the start of the 1-minute sequence
unsigned long lightDimStartTime = 0;
bool isWaitingToTurnOff = false;
const unsigned long OFF_TIMEOUT = 30000; 

void publishCommandStatus(const char* message) {
  mqtt_client.publish(mqtt_command_status_topic, message);
  Serial.println(message);
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(250);
  }
  digitalWrite(LED_PIN, LOW);
}

float intensitySamples[30]; 
int sampleIndex = 0;
bool arrayFull = false;

float getAverageIntensity() {
  int count = arrayFull ? 30 : sampleIndex;
  if (count == 0) return currentLightIntensity;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += intensitySamples[i];
  return sum / count;
}
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) message += (char)payload[i];
  
  if (String(topic) == mqtt_topic) {
    digitalWrite(LED_PIN, (message == "ON") ? HIGH : LOW);
    mqtt_client.publish(mqtt_status_topic, message.c_str());
  }
  else if (String(topic) == mqtt_led2_topic) {
    digitalWrite(LED2_PIN, (message == "ON") ? LOW : HIGH);
    mqtt_client.publish(mqtt_led2_status_topic, message.c_str());
  }
  else if (String(topic) == mqtt_led4_topic) {
    digitalWrite(LED4_PIN, LOW);
    delayMicroseconds(15);
    digitalWrite(LED4_PIN, HIGH);
    mqtt_client.publish(mqtt_led4_status_topic, message.c_str());
    publishCommandStatus("✓ Manual Pulse triggered from Dashboard");
  }
  else if (String(topic) == mqtt_emergency_light_topic) {
    manualEmergencyControl = (message != "AUTO");
    if (message == "ON") digitalWrite(POWER_STATUS_PIN, LOW);
    else if (message == "OFF") digitalWrite(POWER_STATUS_PIN, HIGH);
    mqtt_client.publish(mqtt_emergency_light_status_topic, message.c_str());
  }
}

void connectMQTT() {
  while (!mqtt_client.connected()) {
    if (mqtt_client.connect(mqtt_client_id)) {
      mqtt_client.subscribe(mqtt_topic);
      mqtt_client.subscribe(mqtt_led2_topic);
      mqtt_client.subscribe(mqtt_led4_topic);
      mqtt_client.subscribe(mqtt_emergency_light_topic);
    } else {
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(POWER_STATUS_PIN, OUTPUT);
  
  digitalWrite(LED2_PIN, HIGH); 
  digitalWrite(LED4_PIN, HIGH); 
  digitalWrite(POWER_STATUS_PIN, HIGH); 

  pinMode(INTENSITY_B0_PIN, INPUT);
  pinMode(INTENSITY_B1_PIN, INPUT);
  pinMode(INTENSITY_B2_PIN, INPUT);

  Wire.begin();
  if (INA.begin()) {
    INA.setShuntR(0, 0.1); INA.setShuntR(1, 0.1); INA.setShuntR(2, 0.1);
  }
  connectWiFi();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
}

void loop() {
  if (!mqtt_client.connected()) connectMQTT();
  mqtt_client.loop();

  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead >= 2000) {
    lastSensorRead = millis();
    
    // 1. Read Voltage (V)
    float voltage1 = INA.getBusVoltage(0); 
    float voltage2 = INA.getBusVoltage(1); 

    // 2. Read Current (Amps) and convert to mA
    // Note: If using BeastDevices library, getCurrent() returns Amps.
    float current1 = INA.getCurrent(0) * 1000.0; 
    float current2 = INA.getCurrent(1) * 1000.0;

    // 3. Calculate Power (mW): P = V * I
    float power1 = voltage1 * current1;
    float power2 = voltage2 * current2;

    // 4. Publish All Data
    mqtt_client.publish(mqtt_sensor_voltage_topic, String(voltage1).c_str());
    mqtt_client.publish(mqtt_sensor_current_topic, String(current1).c_str()); // New
    mqtt_client.publish(mqtt_sensor_power_topic, String(power1).c_str());     // New

    mqtt_client.publish(mqtt_sensor2_voltage_topic, String(voltage2).c_str());
    mqtt_client.publish(mqtt_sensor2_current_topic, String(current2).c_str()); // New
    mqtt_client.publish(mqtt_sensor2_power_topic, String(power2).c_str());     // New

    // --- Intensity Calculation ---
    int intensityValue = (digitalRead(INTENSITY_B2_PIN) << 2) | (digitalRead(INTENSITY_B1_PIN) << 1) | digitalRead(INTENSITY_B0_PIN);
    currentLightIntensity = (intensityValue / 7.0) * 100.0;
    mqtt_client.publish(mqtt_light_intensity_topic, String(currentLightIntensity, 1).c_str());

    // --- Sampling ---
    intensitySamples[sampleIndex] = currentLightIntensity;
    sampleIndex++;
    if (sampleIndex >= 30) { 
      sampleIndex = 0; 
      arrayFull = true; 
    }

    // --- Power Cut Logic ---
    if (voltage2 < POWER_CUT_THRESHOLD) {
      if (!powerCutDetected) {
        powerCutDetected = true;
        emergencyModeActive = true;
        emergencyStartTime = millis(); 
        gpio14Activated = false;
        isWaitingToTurnOff = false; 
        
        if (currentLightIntensity < 40.0) {
          digitalWrite(POWER_STATUS_PIN, LOW); 
          mqtt_client.publish(mqtt_emergency_light_status_topic, "ON");
          publishCommandStatus("🌙 Low light detected (<40%): Emergency Light ON");
        } else {
          digitalWrite(POWER_STATUS_PIN, HIGH); 
          mqtt_client.publish(mqtt_emergency_light_status_topic, "OFF");
          publishCommandStatus("☀️ Sufficient light: Emergency Light remains OFF");
        }

        powerCutStartTime = millis();
        startVoltage = voltage1;
        mqtt_client.publish(mqtt_powercut_topic, "POWER_CUT");
        
        digitalWrite(LED2_PIN, LOW); 
        mqtt_client.publish(mqtt_led2_status_topic, "ON");
        
        gpio14ActivationTime = millis() + GPIO14_DELAY;
        Serial.println("⚠️ Power Cut Detected: Initial intensity check completed.");
      }
    
    } else {
      if (powerCutDetected) {
        powerCutDetected = false;
        emergencyModeActive = false;
        isWaitingToTurnOff = false;
        if (!manualEmergencyControl) {
          digitalWrite(POWER_STATUS_PIN, HIGH); 
          mqtt_client.publish(mqtt_emergency_light_status_topic, "OFF");
        }
        digitalWrite(LED2_PIN, HIGH); 
        digitalWrite(LED4_PIN, HIGH); 
        mqtt_client.publish(mqtt_led2_status_topic, "OFF");
        mqtt_client.publish(mqtt_powercut_topic, "NORMAL");
        publishCommandStatus("✓ Power restored.");
      }
    }
  }

  // --- Emergency Mode Management ---
  if (emergencyModeActive) {
    unsigned long elapsed = millis() - emergencyStartTime;

    if (!gpio14Activated && (millis() >= gpio14ActivationTime)) {
      digitalWrite(LED4_PIN, LOW);
      delayMicroseconds(10);
      digitalWrite(LED4_PIN, HIGH);
      gpio14Activated = true;
    }

    float avg = getAverageIntensity();
    if (avg >= 40.0 && digitalRead(POWER_STATUS_PIN) == LOW) {
        digitalWrite(POWER_STATUS_PIN, HIGH); 
        mqtt_client.publish(mqtt_emergency_light_status_topic, "OFF");
        publishCommandStatus("☀️ 1-min Average > 40%: Light turned OFF");
    } else if (avg < 40.0 && digitalRead(POWER_STATUS_PIN) == HIGH && elapsed < 60000) {
        digitalWrite(POWER_STATUS_PIN, LOW); 
        mqtt_client.publish(mqtt_emergency_light_status_topic, "ON");
    }

    if (elapsed >= 60000) {
      digitalWrite(LED2_PIN, HIGH); 
      mqtt_client.publish(mqtt_led2_status_topic, "OFF");
      
      if (getAverageIntensity() >= 40.0) {
        digitalWrite(POWER_STATUS_PIN, HIGH);
        mqtt_client.publish(mqtt_emergency_light_status_topic, "OFF");
      }
      
      publishCommandStatus("⏹️ 1 Minute Elapsed: System Shutting Down");
      emergencyModeActive = false; 
    }
  }
}