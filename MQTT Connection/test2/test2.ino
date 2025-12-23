#include <WiFi.h>

// ===== Replace with your WiFi credentials =====
const char* ssid = "Dialog 4G 858";
const char* password = "04588A9D";

void setup() {
  Serial.begin(115200);
  delay(3000); // Allow Serial Monitor to start

  Serial.println("\n===== ESP32-C3 WiFi Debug Test =====");

  // Show MAC address
  Serial.print("ESP32-C3 MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Ensure station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);  // Clear previous WiFi config
  delay(1000);

  Serial.print("Connecting to SSID: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
}

void loop() {
  // WiFi status codes: 0=IDLE, 1=NO_SSID, 2=SCAN_DONE, 3=CONNECTED, 4=CONNECT_FAILED, 5=CONNECTION_LOST, 6=WRONG_PASSWORD
  wl_status_t status = WiFi.status();

  switch (status) {
    case WL_IDLE_STATUS:
      Serial.println("Status: WL_IDLE_STATUS (0)");
      break;
    case WL_NO_SSID_AVAIL:
      Serial.println("Status: WL_NO_SSID_AVAIL (1)");
      break;
    case WL_SCAN_COMPLETED:
      Serial.println("Status: WL_SCAN_COMPLETED (2)");
      break;
    case WL_CONNECTED:
      Serial.println("Status: WL_CONNECTED (3) ✅");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      delay(5000); // Wait 5 seconds before repeating
      return;      // Connected — skip rest
    case WL_CONNECT_FAILED:
      Serial.println("Status: WL_CONNECT_FAILED (4)");
      break;
    case WL_CONNECTION_LOST:
      Serial.println("Status: WL_CONNECTION_LOST (5)");
      break;
    case WL_WRONG_PASSWORD:
      Serial.println("Status: WL_WRONG_PASSWORD (6) ❌");
      break;
    default:
      Serial.println("Status: UNKNOWN");
      break;
  }

  delay(1000); // Update status every second
}
