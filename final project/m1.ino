// ===== M1: COLLECTOR (ESP-NOW RX -> UART BATCH) =====
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

#define MAX_NODES 10
#define NODE_TIMEOUT_MS 3000
#define BATCH_INTERVAL_MS 1000

// UART to M2
HardwareSerial uartLink(1);
#define LINK_TX 17
#define LINK_RX 16   // not used in 1-way, but ok
#define LINK_BAUD 921600

// ---- Node packet (ESP-NOW payload) ----
typedef struct __attribute__((packed)) {
  uint8_t  nodeId;     // 1..10
  uint16_t seq;        // increments per send
  float    v[6];       // generic values, unused = NAN
} NodePacket;

struct NodeStatus {
  bool online;
  uint32_t lastSeenMs;
  NodePacket last;
};

NodeStatus nodes[MAX_NODES + 1]; // index by nodeId

// ---- CRC16 CCITT-FALSE ----
static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/*
UART Frame format (M1->M2):
[0xAA][0x55][TYPE=0x01][LEN_L][LEN_H][PAYLOAD...][CRC_L][CRC_H]

PAYLOAD for batch:
uint32_t batchTsMs
uint8_t  count
NodePacket packets[count]
*/
static void sendBatchUART() {
  // Build payload in a buffer
  uint8_t payload[1024];
  size_t pos = 0;

  const uint32_t ts = millis();
  memcpy(payload + pos, &ts, sizeof(ts)); pos += sizeof(ts);

  // count placeholder
  uint8_t count = 0;
  payload[pos++] = 0;

  // append active node packets
  for (int id = 1; id <= MAX_NODES; id++) {
    if (nodes[id].lastSeenMs == 0) continue;
    bool active = (ts - nodes[id].lastSeenMs) <= NODE_TIMEOUT_MS;
    if (!active) continue;

    memcpy(payload + pos, &nodes[id].last, sizeof(NodePacket));
    pos += sizeof(NodePacket);
    count++;
  }

  // write actual count
  payload[sizeof(ts)] = count;

  // Frame header
  const uint8_t h1 = 0xAA, h2 = 0x55, type = 0x01;
  const uint16_t len = (uint16_t)pos;

  uint16_t crc = crc16_ccitt(payload, len);

  uartLink.write(h1);
  uartLink.write(h2);
  uartLink.write(type);
  uartLink.write((uint8_t)(len & 0xFF));
  uartLink.write((uint8_t)((len >> 8) & 0xFF));
  uartLink.write(payload, len);
  uartLink.write((uint8_t)(crc & 0xFF));
  uartLink.write((uint8_t)((crc >> 8) & 0xFF));

  Serial.printf("➡️ Sent batch: %u nodes\n", count);
}

void onReceive(const esp_now_recv_info* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(NodePacket)) {
    Serial.println("⚠ ESP-NOW wrong size");
    return;
  }
  NodePacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.nodeId < 1 || pkt.nodeId > MAX_NODES) return;

  nodes[pkt.nodeId].online = true;
  nodes[pkt.nodeId].lastSeenMs = millis();
  nodes[pkt.nodeId].last = pkt;

  Serial.printf("📥 Node %u seq %u\n", pkt.nodeId, pkt.seq);
}

uint32_t lastBatch = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  uartLink.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);
  Serial.println("✅ UART ready (M1->M2)");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onReceive);
  Serial.println("📡 ESP-NOW RX ready");
}

void loop() {
  uint32_t now = millis();

  // mark offline
  for (int id = 1; id <= MAX_NODES; id++) {
    if (nodes[id].lastSeenMs == 0) continue;
    if (nodes[id].online && (now - nodes[id].lastSeenMs > NODE_TIMEOUT_MS)) {
      nodes[id].online = false;
      Serial.printf("⚠ Node %d OFFLINE\n", id);
    }
  }

  // batch to M2 every 1s
  if (now - lastBatch >= BATCH_INTERVAL_MS) {
    lastBatch = now;
    sendBatchUART();
  }
}
