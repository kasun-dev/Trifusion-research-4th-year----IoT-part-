#include <esp_now.h>
#include <WiFi.h>

#define NODE_TIMEOUT 5000

typedef struct {
  uint8_t nodeID;
  float t1;
  float t2;
  float v1;
  float v2;
  float rpm;
  float voltage;
} SensorPacket;

struct NodeStatus {
  bool online;
  unsigned long lastSeen;
  SensorPacket lastPkt;
};

NodeStatus nodes[10];

void onReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(SensorPacket)) {
    Serial.println("⚠ Wrong packet size!");
    return;
  }

  SensorPacket pkt;
  memcpy(&pkt, incomingData, sizeof(pkt));

  const uint8_t *mac = info->src_addr;

  Serial.println("----- PACKET RECEIVED -----");
  Serial.print("From MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  Serial.print("Node ID: "); Serial.println(pkt.nodeID);
  Serial.print("T1: "); Serial.println(pkt.t1);
  Serial.print("T2: "); Serial.println(pkt.t2);
  Serial.print("V1: "); Serial.println(pkt.v1);
  Serial.print("V2: "); Serial.println(pkt.v2);
  Serial.print("RPM: "); Serial.println(pkt.rpm);
  Serial.print("Voltage: "); Serial.println(pkt.voltage);
  Serial.println("---------------------------");

  nodes[pkt.nodeID].online = true;
  nodes[pkt.nodeID].lastSeen = millis();
  nodes[pkt.nodeID].lastPkt = pkt;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("MAIN ESP32 - ESP-NOW RECEIVER");

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed!");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  Serial.println("ESP-NOW Ready, Listening...");
}

void loop() {
  unsigned long now = millis();

  for (int id = 1; id < 10; id++) {
    if (nodes[id].online && now - nodes[id].lastSeen > NODE_TIMEOUT) {
      nodes[id].online = false;
      Serial.print("⚠ Node ");
      Serial.print(id);
      Serial.println(" OFFLINE");
    }
  }

  delay(200);
}
