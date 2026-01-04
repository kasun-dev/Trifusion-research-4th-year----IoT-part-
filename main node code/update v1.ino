// Main ESP32: ESP-NOW receiver + WiFiManager portal + Azure IoT Hub sender
// Install libraries: WiFiManager, Azure IoT Hub (and AzureIoTProtocol_MQTT), ArduinoJson

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>                // https://github.com/tzapu/WiFiManager
#include <esp_now.h>
#include <AzureIoTHub.h>
#include <AzureIoTProtocol_MQTT.h>
#include <ArduinoJson.h>

// ----------------- CONFIG: paste your IoT device connection string here -----------------
static const char* connectionString = "HostName=relatime-iot-hub-01.azure-devices.net;DeviceId=main-node-esp32;SharedAccessKey=xbE+8l6USF0EzGLzRwy/IP577ReqF66Ov8PnXdoBos8=";
// -----------------------------------------------------------------------------------------

#define NODE_TIMEOUT 5000
#define MAX_NODES 10               // support up to 9 nodes (index by nodeID)
#define AGGREGATE_INTERVAL_MS 1000 // send aggregated JSON every 1 second

// Matches your sub-node struct
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

NodeStatus nodes[MAX_NODES];

// Azure IoT Hub client handle (LL-layer)
static IOTHUB_DEVICE_CLIENT_LL_HANDLE deviceHandle = NULL;

// ----------------------------------------------------------------
// ESP-NOW receive callback (uses new signature)
void onReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(SensorPacket)) {
    Serial.print("⚠ Wrong packet size! got="); Serial.print(len);
    Serial.print(" expected="); Serial.println(sizeof(SensorPacket));
    return;
  }

  SensorPacket pkt;
  memcpy(&pkt, incomingData, sizeof(pkt));

  // Protect nodeID
  uint8_t idx = pkt.nodeID;
  if (idx >= MAX_NODES) {
    Serial.print("⚠ Invalid nodeID: "); Serial.println(idx);
    return;
  }

  // Update node status
  nodes[idx].online = true;
  nodes[idx].lastSeen = millis();
  nodes[idx].lastPkt = pkt;

  // Print (optional)
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
}

// ----------------------------------------------------------------
// Initialize ESP-NOW and register callback
void initEspNowReceiver() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed!");
    return;
  }
  // register receiver callback (new API)
  esp_now_register_recv_cb(onReceive);
  Serial.println("ESP-NOW initialized (receiver)");
}

// ----------------------------------------------------------------
// Safe JSON helper for float (ensures numeric and not NaN/INF)
float safeVal(float v) {
  if (isnan(v) || isinf(v)) return 0.0f;
  return v;
}

// ----------------------------------------------------------------
// Azure IoT Hub init from connection string (LL client)
bool initAzureClient() {
  if (deviceHandle != NULL) return true;
  deviceHandle = IoTHubDeviceClient_LL_CreateFromConnectionString(connectionString, MQTT_Protocol);
  if (deviceHandle == NULL) {
    Serial.println("❌ Failed to create Azure IoT client from connection string");
    return false;
  }
  // optional: set some options (timeouts, logging) here
  Serial.println("✅ Azure IoT client created");
  return true;
}

// send JSON string to Azure IoT Hub
bool sendTelemetryToAzure(const char *jsonStr) {
  if (!initAzureClient()) return false;
  IOTHUB_MESSAGE_HANDLE msgHandle = IoTHubMessage_CreateFromString(jsonStr);
  if (msgHandle == NULL) {
    Serial.println("❌ IoTHubMessage_CreateFromString failed");
    return false;
  }
  // Send async (LL layer). No callback required, we'll call DoWork() in loop.
  if (IoTHubDeviceClient_LL_SendEventAsync(deviceHandle, msgHandle, NULL, NULL) != IOTHUB_CLIENT_OK) {
    Serial.println("❌ IoTHubDeviceClient_LL_SendEventAsync failed");
    IoTHubMessage_Destroy(msgHandle);
    return false;
  }
  IoTHubMessage_Destroy(msgHandle);
  return true;
}

// ----------------------------------------------------------------
// Build aggregated JSON for all nodes and send to Azure
void aggregateAndSend() {
  // Build JSON: { "hub": "main-node-esp32", "ts": 12345, "nodes":[{...},{...}] }
  StaticJsonDocument<1024> doc;
  doc["hub"] = "main-node-esp32";
  doc["timestamp_ms"] = (uint64_t)millis();

  JsonArray arr = doc.createNestedArray("nodes");

  for (int i = 0; i < MAX_NODES; ++i) {
    JsonObject n = arr.createNestedObject();
    n["nodeID"] = i;
    n["online"] = nodes[i].online ? 1 : 0;
    n["lastSeen_ms"] = nodes[i].lastSeen;

    // Expand last packet fields (safeVal)
    n["t1"] = safeVal(nodes[i].lastPkt.t1);
    n["t2"] = safeVal(nodes[i].lastPkt.t2);
    n["v1"] = safeVal(nodes[i].lastPkt.v1);
    n["v2"] = safeVal(nodes[i].lastPkt.v2);
    n["rpm"] = safeVal(nodes[i].lastPkt.rpm);
    n["voltage"] = safeVal(nodes[i].lastPkt.voltage);
  }

  char buffer[1024];
  size_t n = serializeJson(doc, buffer, sizeof(buffer));
  Serial.print("Sending aggregated JSON ("); Serial.print(n); Serial.println(" bytes)");

  // Send to Azure
  if (!sendTelemetryToAzure(buffer)) {
    Serial.println("⚠ Failed to queue telemetry to Azure");
  } else {
    Serial.println("✔ Telemetry queued to Azure");
  }
}

// ----------------------------------------------------------------
// WiFiManager captive portal setup
void startWifiManagerPortal() {
  WiFiManager wm;
  // Optionally set an AP name
  String apName = "ESP32-GW-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  wm.autoConnect(apName.c_str()); // blocks until connected
  Serial.println("WiFi connected via WiFiManager");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

// ----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("MAIN ESP32 - ESP-NOW -> Azure IoT Gateway");

  // initialize nodes array
  for (int i = 0; i < MAX_NODES; ++i) {
    nodes[i].online = false;
    nodes[i].lastSeen = 0;
    memset(&nodes[i].lastPkt, 0, sizeof(SensorPacket));
  }

  // init ESP-NOW receiver
  initEspNowReceiver();

  // start WiFi & Captive portal to get SSID/password dynamically
  startWifiManagerPortal();

  // initialize Azure client (after WiFi is available)
  if (!initAzureClient()) {
    Serial.println("Warning: Azure client not ready yet");
  }
}

// ----------------------------------------------------------------
unsigned long lastAggregate = 0;

void loop() {
  unsigned long now = millis();

  // 1) maintain IoT Hub LL client state machine
  if (deviceHandle != NULL) {
    IoTHubDeviceClient_LL_DoWork(deviceHandle);
  }

  // 2) check node timeouts
  for (int id = 0; id < MAX_NODES; ++id) {
    if (nodes[id].online && (now - nodes[id].lastSeen > NODE_TIMEOUT)) {
      nodes[id].online = false;
      Serial.print("⚠ Node "); Serial.print(id); Serial.println(" OFFLINE");
    }
  }

  // 3) aggregate & send every AGGREGATE_INTERVAL_MS
  if (now - lastAggregate >= AGGREGATE_INTERVAL_MS) {
    lastAggregate = now;
    aggregateAndSend();
  }

  // 4) small delay and call DoWork again for azure reliability
  if (deviceHandle != NULL) IoTHubDeviceClient_LL_DoWork(deviceHandle);
  delay(10);
}
