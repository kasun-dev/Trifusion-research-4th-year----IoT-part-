#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>     // ⭐ REQUIRED 
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <max6675.h>
#include <DHT.h>

// ============================================================
// ESP-NOW CHANNEL (MUST MATCH GATEWAY)
// ============================================================
#define ESPNOW_CHANNEL 6

// ============================================================
// MAIN ESP32 (GATEWAY) MAC ADDRESS
// ============================================================
uint8_t mainMAC[] = {0xCC, 0xDB, 0xA7, 0x53, 0x46, 0x80};

// ============================================================
// PACKET FORMAT
// ============================================================
typedef struct {
  uint8_t nodeID;
  float t1;
  float t2;
  float v1;
  float v2;
  float rpm;
  float voltage;
} SensorPacket;

SensorPacket packet;

// ============================================================
// SAFE VALUE HANDLER
// ============================================================
float safeValue(float v) {
  if (isnan(v) || isinf(v)) return 0;
  return v;
}

// ============================================================
// ESP-NOW SEND CALLBACK
// ============================================================
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ============================================================
// ESP-NOW INIT (FIXED)
// ============================================================
void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();   // ⭐ IMPORTANT

  // ⭐ LOCK CHANNEL
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed!");
    ESP.restart();
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mainMAC, 6);
  peer.channel = ESPNOW_CHANNEL;   // ⭐ FIXED
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("❌ Failed to add peer!");
  }
}

// ============================================================
// ADXL345
// ============================================================
Adafruit_ADXL345_Unified accel1(1);
Adafruit_ADXL345_Unified accel2(2);

// ============================================================
// MAX6675
// ============================================================
int thermoSO = 19;
int thermoSCK = 18;
int thermoCS1 = 5;
int thermoCS2 = 4;

MAX6675 therm1(thermoSCK, thermoCS1, thermoSO);
MAX6675 therm2(thermoSCK, thermoCS2, thermoSO);

// ============================================================
// DHT22
// ============================================================
#define DHTPIN 25
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ============================================================
// RMS SETTINGS
// ============================================================
const int SAMPLE_RATE = 200;
const int WINDOW_MS = 1000;

unsigned long lastSample = 0;
unsigned long lastRms = 0;

float sumSq1 = 0, sumSq2 = 0;
int count1 = 0, count2 = 0;

// ============================================================
// SEND PACKET
// ============================================================
void sendToMain(float rms1, float rms2, float t1, float t2, float et, float eh) {

  packet.nodeID = 1;
  packet.v1 = safeValue(rms1);
  packet.v2 = safeValue(rms2);
  packet.t1 = safeValue(t1);
  packet.t2 = safeValue(t2);
  packet.rpm = safeValue(et);
  packet.voltage = safeValue(eh);

  esp_now_send(mainMAC, (uint8_t*)&packet, sizeof(packet));
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  initEspNow();
  dht.begin();

  if (!accel1.begin(0x53)) Serial.println("⚠ ADXL #1 missing!");
  if (!accel2.begin(0x1D)) Serial.println("⚠ ADXL #2 missing!");

  accel1.setRange(ADXL345_RANGE_16_G);
  accel2.setRange(ADXL345_RANGE_16_G);
  accel1.setDataRate(ADXL345_DATARATE_400_HZ);
  accel2.setDataRate(ADXL345_DATARATE_400_HZ);

  Serial.println("✅ FAN NODE READY");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  if (now - lastSample >= (1000 / SAMPLE_RATE)) {
    lastSample = now;

    sensors_event_t e1, e2;
    accel1.getEvent(&e1);
    accel2.getEvent(&e2);

    float mag1 = safeValue(sqrt(
      e1.acceleration.x * e1.acceleration.x +
      e1.acceleration.y * e1.acceleration.y +
      e1.acceleration.z * e1.acceleration.z));

    float mag2 = safeValue(sqrt(
      e2.acceleration.x * e2.acceleration.x +
      e2.acceleration.y * e2.acceleration.y +
      e2.acceleration.z * e2.acceleration.z));

    sumSq1 += mag1 * mag1;
    sumSq2 += mag2 * mag2;
    count1++;
    count2++;
  }

  if (now - lastRms >= WINDOW_MS) {
    lastRms = now;

    float rms1 = count1 ? sqrt(sumSq1 / count1) : 0;
    float rms2 = count2 ? sqrt(sumSq2 / count2) : 0;

    float t1 = safeValue(therm1.readCelsius());
    float t2 = safeValue(therm2.readCelsius());
    float et = safeValue(dht.readTemperature());
    float eh = safeValue(dht.readHumidity());

    Serial.printf(
      "RMS1: %.3f | RMS2: %.3f | T1: %.1f | T2: %.1f | ET: %.1f | EH: %.1f\n",
      rms1, rms2, t1, t2, et, eh
    );

    sendToMain(rms1, rms2, t1, t2, et, eh);

    sumSq1 = sumSq2 = 0;
    count1 = count2 = 0;
  }
}
