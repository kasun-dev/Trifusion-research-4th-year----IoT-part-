#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include <esp_wifi.h>   // ⭐ FIX: needed for channel control

/* ================= WIFI ================= */
#define WIFI_SSID "Iphone"
#define WIFI_PASS "0760853136"

/* ================= AWS ================= */
#define AWS_ENDPOINT "a2mlrmhn7n3wcj-ats.iot.eu-north-1.amazonaws.com"
#define AWS_PORT 8883
#define AWS_TOPIC "esp32/test"

/* ================= NODE CONFIG ================= */
#define MAX_NODES 10
#define NODE_TIMEOUT 5000
#define PUBLISH_INTERVAL 5000
#define ESPNOW_CHANNEL 6   // ⭐ FIX: must match sender

/* ================= CERTIFICATES ================= */
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

/* ================= STRUCTS ================= */
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

/* ================= GLOBALS ================= */
WiFiClientSecure net;
PubSubClient client(net);
NodeStatus nodes[MAX_NODES];
unsigned long lastPublish = 0;

/* ================= WIFI ================= */
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi Connected");
}

/* ================= AWS ================= */
void connectAWS() {
  net.setCACert(AWS_ROOT_CA);
  net.setCertificate(AWS_CERT);
  net.setPrivateKey(AWS_PRIVATE_KEY);

  client.setServer(AWS_ENDPOINT, AWS_PORT);

  Serial.print("Connecting to AWS IoT");
  while (!client.connected()) {
    if (client.connect("ESP32_MAIN_GATEWAY")) {
      Serial.println("\n✅ AWS Connected");
    } else {
      Serial.print(".");
      delay(1000);
    }
  }
}

/* ================= ESP-NOW RECEIVE ================= */
void onReceive(const esp_now_recv_info *info,
               const uint8_t *incomingData,
               int len) {

  if (len != sizeof(SensorPacket)) return;

  SensorPacket pkt;
  memcpy(&pkt, incomingData, sizeof(pkt));

  if (pkt.nodeID >= MAX_NODES) return;

  nodes[pkt.nodeID].online = true;
  nodes[pkt.nodeID].lastSeen = millis();
  nodes[pkt.nodeID].lastPkt = pkt;

  Serial.printf("📥 Node %d updated\n", pkt.nodeID);
}

/* ================= PUBLISH TO AWS ================= */
void publishNode(uint8_t id) {
  char payload[256];
  SensorPacket &p = nodes[id].lastPkt;

  snprintf(payload, sizeof(payload),
    "{"
    "\"device\":\"esp32-main\","
    "\"nodeID\":%d,"
    "\"t1\":%.2f,"
    "\"t2\":%.2f,"
    "\"v1\":%.2f,"
    "\"v2\":%.2f,"
    "\"rpm\":%.1f,"
    "\"voltage\":%.2f,"
    "\"online\":%s"
    "}",
    id,
    p.t1, p.t2, p.v1, p.v2, p.rpm, p.voltage,
    nodes[id].online ? "true" : "false"
  );

  client.publish(AWS_TOPIC, payload);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  /* ⭐ FIX 1: WiFi STA only, NO INTERNET yet */
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  /* ⭐ FIX 2: Lock WiFi channel for ESP-NOW */
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  /* ⭐ FIX 3: Init ESP-NOW FIRST */
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);
  Serial.println("📡 ESP-NOW Receiver Ready");

  /* ⭐ FIX 4: Now connect WiFi + AWS */
  connectWiFi();
  connectAWS();
}

/* ================= LOOP ================= */
void loop() {
  if (!client.connected()) connectAWS();
  client.loop();

  unsigned long now = millis();

  /* Node timeout */
  for (int i = 1; i < MAX_NODES; i++) {
    if (nodes[i].online && now - nodes[i].lastSeen > NODE_TIMEOUT) {
      nodes[i].online = false;
      Serial.printf("⚠ Node %d OFFLINE\n", i);
    }
  }

  /* Periodic publish */
  if (now - lastPublish > PUBLISH_INTERVAL) {
    lastPublish = now;
    for (int i = 1; i < MAX_NODES; i++) {
      if (nodes[i].lastSeen > 0) publishNode(i);
    }
  }
}
