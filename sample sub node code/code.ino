#include <WiFi.h>
#include <esp_now.h>
#include "DHT.h"

// ----------------------------
// CONFIG
// ----------------------------
#define DHTPIN 4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

// Main ESP32 MAC (replace with your own)
uint8_t mainNodeMAC[] = {0xCC, 0xDB, 0xA7, 0x53, 0x46, 0x80};

// ----------------------------
// PACKET STRUCT
// ----------------------------
typedef struct {
  uint8_t nodeID;
  float t1;
  float t2;
  float v1;
  float v2;
  float rpm;
  float voltage;
} SensorPacket;

SensorPacket pkt;

// ----------------------------
// SEND CALLBACK (NEW FORMAT)
// ----------------------------
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ----------------------------
// SETUP
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("SUB NODE ESP32 - DHT22 Sender");

  dht.begin();

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    return;
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mainNodeMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }
}

// ----------------------------
// LOOP
// ----------------------------
void loop() {
  float temp = dht.readTemperature();
  float humd = dht.readHumidity();

  if (isnan(temp) || isnan(humd)) {
    Serial.println("DHT read failed!");
    delay(1000);
    return;
  }

  pkt.nodeID = 1;
  pkt.t1 = temp;
  pkt.t2 = humd;
  pkt.v1 = 0;
  pkt.v2 = 0;
  pkt.rpm = 0;
  pkt.voltage = 0;

  esp_now_send(mainNodeMAC, (uint8_t*)&pkt, sizeof(pkt));

  Serial.print("Sent Temp: ");
  Serial.print(temp);
  Serial.print("  Hum: ");
  Serial.println(humd);

  delay(1000);
}
