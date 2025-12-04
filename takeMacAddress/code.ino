#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);   // REQUIRED for ESP-NOW MAC
  delay(100);            // Give WiFi driver time to start

  Serial.println("MAC Address:");
  Serial.println(WiFi.macAddress());
}

void loop() {}
