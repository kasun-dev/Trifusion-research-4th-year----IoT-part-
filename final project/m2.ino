// ===== M2: PUBLISHER (UART RX -> AWS MQTT + SIM800L TIME + SD LOGGER) =====
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>

// ---------- WIFI ----------
#define WIFI_SSID "Iphone"
#define WIFI_PASS "0760853136"

// ---------- AWS ----------
#define AWS_ENDPOINT "a2mlrmhn7n3wcj-ats.iot.eu-north-1.amazonaws.com"
#define AWS_PORT 8883
#define AWS_TOPIC "heycarb/vacum_blower/001"

// ---------- UART from M1 ----------
HardwareSerial uartLink(1);
#define LINK_RX 16
#define LINK_TX 17
#define LINK_BAUD 921600

// ---------- SIM800L ----------
HardwareSerial sim800(2);
#define SIM_RX 26
#define SIM_TX 25
#define SIM_BAUD 9600

// ---------- SD ----------
#define SD_CS 5
// SPI pins are default on ESP32: SCK 18, MISO 19, MOSI 23

// ---------- SYSTEM ----------
#define MAX_NODES 10
#define NODE_TIMEOUT_MS 3000
#define PUBLISH_INTERVAL_MS 1000
#define QUEUE_FILE "/queue.log"

// ---- Certs (paste your real ones) ----
static const char AWS_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

static const char AWS_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIURh9bRfXbX9M0d79LMIo1btdtJSEwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI2MDEwMzIzNDA0
N1oXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAK0DwcouG1MXc61UrVzn
EEM8BsZ6dGmCmfkWF+iryHWEeoiTp00KVGN9FUds2BB7S+4gN2VwGrTvmNA0V0L9
cqbshmPE9ycvs28qKY73//TEfGn8VhTBroR2Q516uR0MDyAH8ICrqMrkNISJJxwI
z13qsTN7PS8EtGYFGMOlpYM2jdVux5d7i5q1Q/MiHtyHsmkcEQsUYwlaIeONj8eC
v5rYCrZSUduDYDiKRRKEFyLssExIyq903HdoEpb8/JnjbBkXRp/yQbme00jASHuZ
icdC/hknWQc7frEtUnZw8IcCsmiVQ/d7h902aXHzXCyXUWsYRsxlMiiaV2Yk9EwZ
fuMCAwEAAaNgMF4wHwYDVR0jBBgwFoAUTM5/QHDAKImgN3UDLMQ3d2yW7AswHQYD
VR0OBBYEFPN7WKxBYLypvpaS5wwbJhs9UJYDMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQA3Orj4fGnqEgm/udrdGEPeLZEu
MZjfZCw9yhaDEqm3aK8vO7M1FCFTmtVak5MBdJgbY475Xrdy5zX1gxIevTGUp632
YWP+UdI5w/8GBB7P9jB15eI+sqyK1zxeuhqkYVwgJl5Cut6JshUCE3C6QU2ATRlD
7j2y39YEjY1QLA4gqrjonz6JRg1Ng42utRcF374UasYbsY6vTzupIUC9mh8VLfv7
O6NQLw2levh/ylmUfmZIIqccWGBfmScW9cgzXy/RVqzE6bgDgTh/A8hSqi6bYzs6
1M0MTuS4jZQnOgU5wZ0z965sZQKWV9jO080ftuyAJYUfBpYPG9cEHicHxQz4
-----END CERTIFICATE-----
)EOF";

static const char AWS_PRIVATE_KEY[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEArQPByi4bUxdzrVStXOcQQzwGxnp0aYKZ+RYX6KvIdYR6iJOn
TQpUY30VR2zYEHtL7iA3ZXAatO+Y0DRXQv1ypuyGY8T3Jy+zbyopjvf/9MR8afxW
FMGuhHZDnXq5HQwPIAfwgKuoyuQ0hIknHAjPXeqxM3s9LwS0ZgUYw6WlgzaN1W7H
l3uLmrVD8yIe3IeyaRwRCxRjCVoh442Px4K/mtgKtlJR24NgOIpFEoQXIuywTEjK
r3Tcd2gSlvz8meNsGRdGn/JBuZ7TSMBIe5mJx0L+GSdZBzt+sS1SdnDwhwKyaJVD
93uH3TZpcfNcLJdRaxhGzGUyKJpXZiT0TBl+4wIDAQABAoIBACFf0igQENFMetH6
vZ5oLCjsEVqEEKSxvsXfzyjgykBxF7n00Zb44SJ35HzssBktz43VxRnaMCyq801m
a1bO0SkGAz6Hpi129CQDfBTKHiO3K1C+jlResC15Kr3cCI1j4B8LqQcJFfxdXQwb
8OFliarSNPB7W2gQfkQXw5kqAO14bU1001YAr/yM9xayeXYVFBr/XdtcZBeKtkAq
/n8Nkh114X3MLnzox54CZU3N3Opjqc1jx7AOmvEvJjtLj8sq3K6qUTOFyeGwAw7V
NoVzZX9DflLz3sEpNH1KhFFZLV1F+HCg9D6r2og7MFrrdSyJr3lXDSq0w9WUwPRN
vACKHIECgYEA2v6SLxktXXeJugfsrPIHDFrQUQw1Yh8DVHInqoyZsC4yxbCpE0mU
tlDo2nOZLam7/UCoBHnvyomsNIKRb9EnMEsgXr352WbeY81WJrY+Ah0Sqsj3p+3n
KQaf5CsXkGH/1sGeG9CBG28X61AX5jjT7CqACdPrNLzgXksDwQSYCSECgYEAykAo
rdHv7+eI8FmUdGAQGHdxwxlvHQYQs4uyoRu5XkHBmaxn6YnjO2ew5r+9S2WoVRB0
uqEfRgQMJnGyFCkcoGAbw2+eeCD6vYIhz+g6yWxdpzhT8fV84dOO+k4nYlzqHk2N
zIIumoRwshdpffild0EHj+tbHjf/A7xtSyDTc4MCgYBMML+JVVgcKaeoKnOkY/wh
x0Ksv/OetK2C5sh4JLyfuCL/9ouMY1Ay8glhX1COu3vlC2apUAcmTymzhy20Wm8o
9SpI7A2OHqUG0fzEMSl3sMe61XqcWT/QXTapunhTSlUpUWmBwdP5SHho7Q+zkFfi
1ZKAWNN/IKtrAuxGbiO7IQKBgDVzT0zgtrCIWEBs7Db1TEurBX2yMxNQjwlkWgkJ
8qteZXPfhHbL4inI9Y+GDNjoPNx+RNstyb4PQ8bFNXLuioo33B6CWTcWQC3lPlpb
3W1uHjIbSNQhNKfZ6WdtUCtGsvjfNiJeJULgzYfDeDW6iMBDh2QZpzMNSXALVDcO
rdNRAoGBAIZmwtonMBBABbT5CbSdt2Doro3Ygrwj3G7CDYQE8oepJfHAEJJ0QIht
rxW6j2nIIX0U0MwqOp8++ZyZcPUSyHZENmAECfMDPHO7/IILcdBIyTHiw1ybcJI5
atvXilM+hsDBIDcao3tZ7+7WRXJnWRnDrG/RoJRcmJ/12Mhp5XFW
-----END RSA PRIVATE KEY-----
)EOF";

// ---- Shared packet types ----
typedef struct __attribute__((packed)) {
  uint8_t  nodeId;
  uint16_t seq;
  float    v[6];
} NodePacket;

struct NodeStatus {
  bool online;
  uint32_t lastSeenMs;
  NodePacket last;
};

NodeStatus nodes[MAX_NODES + 1];

// ---------- MQTT ----------
WiFiClientSecure net;
PubSubClient mqtt(net);

// ---------- CRC16 ----------
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

// ---------- Wi-Fi + AWS ----------
static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");
}

static void connectAWS() {
  net.setCACert(AWS_ROOT_CA);
  net.setCertificate(AWS_CERT);
  net.setPrivateKey(AWS_PRIVATE_KEY);

  mqtt.setServer(AWS_ENDPOINT, AWS_PORT);

  Serial.print("AWS connecting");
  while (!mqtt.connected()) {
    if (mqtt.connect("M2_PUBLISHER")) {
      Serial.println("\n✅ AWS connected");
    } else {
      Serial.print(".");
      delay(1000);
    }
  }
}

// ---------- SIM800L helpers ----------
static bool simSendCmd(const char* cmd, const char* expect, uint32_t timeoutMs = 1200) {
  while (sim800.available()) sim800.read();
  sim800.println(cmd);

  uint32_t start = millis();
  String buf;
  while (millis() - start < timeoutMs) {
    while (sim800.available()) {
      char c = (char)sim800.read();
      buf += c;
      if (expect && buf.indexOf(expect) >= 0) return true;
    }
  }
  return false;
}

// Returns epoch seconds if available; else 0.
// NOTE: SIM time needs network + correct format; keep it simple for now.
static uint32_t getEpochFromSIM800L() {
  // Ensure time zone auto update (sometimes helps)
  simSendCmd("AT+CLTS=1", "OK", 1500);
  simSendCmd("AT&W", "OK", 1500);

  // Request clock
  while (sim800.available()) sim800.read();
  sim800.println("AT+CCLK?");
  uint32_t start = millis();
  String s;
  while (millis() - start < 1500) {
    while (sim800.available()) s += (char)sim800.read();
  }

  // Example response: +CCLK: "26/02/13,01:23:45+22"
  int q1 = s.indexOf('"');
  int q2 = s.indexOf('"', q1 + 1);
  if (q1 < 0 || q2 < 0) return 0;

  String t = s.substring(q1 + 1, q2); // yy/MM/dd,hh:mm:ss+zz
  if (t.length() < 17) return 0;

  int yy = t.substring(0, 2).toInt();
  int MM = t.substring(3, 5).toInt();
  int dd = t.substring(6, 8).toInt();
  int hh = t.substring(9, 11).toInt();
  int mm = t.substring(12, 14).toInt();
  int ss = t.substring(15, 17).toInt();

  // Convert naive to epoch (UTC-ish). For project logging, this is fine.
  // If you want exact TZ handling, we can extend later.
  struct tm tmv {};
  tmv.tm_year = 2000 + yy - 1900;
  tmv.tm_mon  = MM - 1;
  tmv.tm_mday = dd;
  tmv.tm_hour = hh;
  tmv.tm_min  = mm;
  tmv.tm_sec  = ss;

  time_t epoch = mktime(&tmv);
  if (epoch < 0) return 0;
  return (uint32_t)epoch;
}

// Maintain an epoch baseline (epochAtSync + millisAtSync)
uint32_t epochAtSync = 0;
uint32_t millisAtSync = 0;

static uint32_t nowEpoch() {
  if (epochAtSync == 0) return 0;
  uint32_t delta = (millis() - millisAtSync) / 1000;
  return epochAtSync + delta;
}

// ---------- SD queue ----------
static bool sdReady = false;

static void queueToSD(const String& line) {
  if (!sdReady) return;
  File f = SD.open(QUEUE_FILE, FILE_APPEND);
  if (!f) return;
  f.println(line);
  f.close();
}

static void replayQueueIfAny() {
  if (!sdReady) return;
  if (!mqtt.connected()) return;

  File f = SD.open(QUEUE_FILE, FILE_READ);
  if (!f) return;

  // Copy to a temp file after successful publishes
  File tmp = SD.open("/queue.tmp", FILE_WRITE);
  if (!tmp) { f.close(); return; }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (mqtt.publish(AWS_TOPIC, line.c_str())) {
      // published OK -> drop it
    } else {
      // failed again -> keep it
      tmp.println(line);
      break; // stop replay to avoid long blocking
    }
    mqtt.loop();
    delay(10);
  }

  // Copy remaining lines as well (if any)
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length()) tmp.println(line);
  }

  f.close();
  tmp.close();

  SD.remove(QUEUE_FILE);
  SD.rename("/queue.tmp", QUEUE_FILE);
}

// ---------- UART batch parser ----------
/*
Frame:
AA 55 TYPE LEN_L LEN_H PAYLOAD CRC_L CRC_H
TYPE=0x01 batch
PAYLOAD: uint32_t batchTsMs, uint8_t count, NodePacket[count]
*/
enum RxState { WAIT_AA, WAIT_55, WAIT_TYPE, WAIT_LEN1, WAIT_LEN2, WAIT_PAYLOAD, WAIT_CRC1, WAIT_CRC2 };
RxState rxState = WAIT_AA;

uint8_t rxType = 0;
uint16_t rxLen = 0;
uint16_t rxCrcRead = 0;
uint16_t payloadPos = 0;
uint8_t payloadBuf[1100];

static void handleBatch(const uint8_t* payload, uint16_t len, uint16_t crcRead) {
  uint16_t crc = crc16_ccitt(payload, len);
  if (crc != crcRead) {
    Serial.println("❌ UART CRC mismatch");
    return;
  }

  if (len < 5) return;
  uint32_t batchTsMs = 0;
  memcpy(&batchTsMs, payload, 4);
  uint8_t count = payload[4];

  size_t expected = 5 + (size_t)count * sizeof(NodePacket);
  if (len < expected) return;

  const uint8_t* p = payload + 5;
  uint32_t now = millis();

  for (uint8_t i = 0; i < count; i++) {
    NodePacket pkt;
    memcpy(&pkt, p, sizeof(pkt));
    p += sizeof(pkt);

    if (pkt.nodeId < 1 || pkt.nodeId > MAX_NODES) continue;

    nodes[pkt.nodeId].online = true;
    nodes[pkt.nodeId].lastSeenMs = now;
    nodes[pkt.nodeId].last = pkt;
  }

  Serial.printf("📥 Batch received (%u nodes)\n", count);
}

static void uartReadLoop() {
  while (uartLink.available()) {
    uint8_t b = (uint8_t)uartLink.read();

    switch (rxState) {
      case WAIT_AA:   rxState = (b == 0xAA) ? WAIT_55 : WAIT_AA; break;
      case WAIT_55:   rxState = (b == 0x55) ? WAIT_TYPE : WAIT_AA; break;
      case WAIT_TYPE: rxType = b; rxState = WAIT_LEN1; break;
      case WAIT_LEN1: rxLen = b; rxState = WAIT_LEN2; break;
      case WAIT_LEN2:
        rxLen |= (uint16_t)b << 8;
        if (rxLen > sizeof(payloadBuf)) { rxState = WAIT_AA; break; }
        payloadPos = 0;
        rxState = WAIT_PAYLOAD;
        break;
      case WAIT_PAYLOAD:
        payloadBuf[payloadPos++] = b;
        if (payloadPos >= rxLen) rxState = WAIT_CRC1;
        break;
      case WAIT_CRC1: rxCrcRead = b; rxState = WAIT_CRC2; break;
      case WAIT_CRC2:
        rxCrcRead |= (uint16_t)b << 8;
        if (rxType == 0x01) handleBatch(payloadBuf, rxLen, rxCrcRead);
        rxState = WAIT_AA;
        break;
    }
  }
}

// ---------- Publish combined JSON ----------
static String floatToJson(float x) {
  if (isnan(x)) return "null";
  // keep small
  return String(x, 4);
}

static String buildJson() {
  uint32_t ts = nowEpoch(); // 0 if SIM time not ready
  uint32_t now = millis();

  String s = "{";
  s += "\"gateway\":\"M2\",";
  if (ts != 0) s += "\"timestamp\":" + String(ts) + ",";
  s += "\"nodes\":[";

  bool first = true;
  for (int id = 1; id <= MAX_NODES; id++) {
    if (nodes[id].lastSeenMs == 0) continue;
    bool active = (now - nodes[id].lastSeenMs) <= NODE_TIMEOUT_MS;

    if (!first) s += ",";
    first = false;

    s += "{";
    s += "\"id\":" + String(id) + ",";
    s += "\"seq\":" + String(nodes[id].last.seq) + ",";
    s += "\"online\":" + String(active ? "true" : "false") + ",";
    s += "\"v\":[";
    for (int k = 0; k < 6; k++) {
      if (k) s += ",";
      s += floatToJson(nodes[id].last.v[k]);
    }
    s += "]";
    s += "}";
  }

  s += "]}";
  return s;
}

uint32_t lastPublish = 0;

void setup() {
  Serial.begin(115200);
  delay(600);

  // UART from M1
  uartLink.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);
  Serial.println("✅ UART ready (from M1)");

  // SIM800L
  sim800.begin(SIM_BAUD, SERIAL_8N1, SIM_RX, SIM_TX);
  Serial.println("✅ SIM800L UART ready");

  // SD
  if (SD.begin(SD_CS)) {
    sdReady = true;
    Serial.println("✅ SD ready");
  } else {
    Serial.println("⚠ SD not ready (offline queue disabled)");
  }

  // Wi-Fi + AWS
  connectWiFi();
  connectAWS();

  // Sync time from SIM (best-effort)
  epochAtSync = getEpochFromSIM800L();
  millisAtSync = millis();
  if (epochAtSync) Serial.printf("✅ SIM time synced: %u\n", epochAtSync);
  else Serial.println("⚠ SIM time not available yet (timestamp=0)");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    // try reconnect quickly
    connectWiFi();
  }
  if (!mqtt.connected()) {
    connectAWS();
  }
  mqtt.loop();

  // Read data from M1
  uartReadLoop();

  // Mark offline
  uint32_t now = millis();
  for (int id = 1; id <= MAX_NODES; id++) {
    if (nodes[id].lastSeenMs == 0) continue;
    if (nodes[id].online && (now - nodes[id].lastSeenMs > NODE_TIMEOUT_MS)) {
      nodes[id].online = false;
    }
  }

  // Publish every 1 second
  if (now - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = now;

    // refresh SIM time occasionally (optional)
    if (epochAtSync == 0 || (now - millisAtSync) > 60000) {
      uint32_t e = getEpochFromSIM800L();
      if (e) { epochAtSync = e; millisAtSync = now; }
    }

    String payload = buildJson();

    bool ok = mqtt.publish(AWS_TOPIC, payload.c_str());
    if (ok) {
      Serial.println("☁️ Published batch");
      replayQueueIfAny(); // try to drain backlog
    } else {
      Serial.println("❌ Publish failed -> queued to SD");
      queueToSD(payload);
    }
  }
}
