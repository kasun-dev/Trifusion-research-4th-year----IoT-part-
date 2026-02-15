#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <max6675.h>
#include <DHT.h>
#include <math.h>

/* ===================== CONFIG ===================== */
// IMPORTANT: Set this to M1 (Collector ESP32) MAC address
uint8_t m1CollectorMAC[] = {0xCC, 0xDB, 0xA7, 0x53, 0x46, 0x80}; // <-- change

static const uint8_t NODE_ID = 1;     // 1..10
static const uint32_t SEND_INTERVAL_MS = 1000;
static const uint32_t SLOT_MS = 100;  // TDMA slot size
static const uint32_t SAMPLE_RATE = 200;
static const uint32_t WINDOW_MS = 1000;

/* ===================== PACKET ===================== */
typedef struct __attribute__((packed)) {
  uint8_t  nodeId;
  uint16_t seq;
  float    v[6];   // generic values, unused = NAN
} NodePacket;

NodePacket pkt;
uint16_t seqNo = 0;

/* ===================== SENSORS ===================== */
Adafruit_ADXL345_Unified accel1(1);  // 0x53
Adafruit_ADXL345_Unified accel2(2);  // 0x1D

// MAX6675 (Thermocouples)
int thermoSO  = 19;
int thermoSCK = 18;
int thermoCS1 = 5;
int thermoCS2 = 4;

MAX6675 therm1(thermoSCK, thermoCS1, thermoSO);
MAX6675 therm2(thermoSCK, thermoCS2, thermoSO);

// DHT22
#define DHTPIN  25
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

/* ===================== RMS STATE ===================== */
unsigned long lastSample = 0;
unsigned long lastWindow = 0;

float sumSq1 = 0, sumSq2 = 0;
int count1 = 0, count2 = 0;

/* ===================== HELPERS ===================== */
static float safeFloat(float x) {
  if (isnan(x) || isinf(x)) return NAN; // keep as NAN (do not force 0)
  return x;
}

static float magFromAccel(const sensors_event_t &e) {
  float x = e.acceleration.x;
  float y = e.acceleration.y;
  float z = e.acceleration.z;
  float m = sqrtf(x*x + y*y + z*z);
  return safeFloat(m);
}

/* ===================== ESP-NOW ===================== */
void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

static void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed!");
    ESP.restart();
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, m1CollectorMAC, 6);
  peer.channel = 0;      // use current channel; M1 will be fixed on router channel if needed
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("❌ Failed to add peer!");
  } else {
    Serial.println("✅ Peer added (M1 Collector)");
  }
}

/* ===================== SEND PACKET ===================== */
static void sendPacket(float t1, float t2, float rms1, float rms2, float envT, float envH) {
  pkt.nodeId = NODE_ID;
  pkt.seq = seqNo++;

  // Fill all 6 values (unused would be NAN, but Node1 uses all)
  pkt.v[0] = safeFloat(t1);
  pkt.v[1] = safeFloat(t2);
  pkt.v[2] = safeFloat(rms1);
  pkt.v[3] = safeFloat(rms2);
  pkt.v[4] = safeFloat(envT);
  pkt.v[5] = safeFloat(envH);

  esp_now_send(m1CollectorMAC, (uint8_t*)&pkt, sizeof(pkt));
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(21, 22);

  initEspNow();
  dht.begin();

  // ADXL init (do not stop if missing)
  if (!accel1.begin(0x53)) Serial.println("⚠ WARNING: ADXL #1 missing!");
  if (!accel2.begin(0x1D)) Serial.println("⚠ WARNING: ADXL #2 missing!");

  accel1.setRange(ADXL345_RANGE_16_G);
  accel2.setRange(ADXL345_RANGE_16_G);
  accel1.setDataRate(ADXL345_DATARATE_400_HZ);
  accel2.setDataRate(ADXL345_DATARATE_400_HZ);

  // init timestamps
  lastSample = millis();
  lastWindow = millis();

  Serial.println("✅ NODE 1 READY (New Protocol: nodeId, seq, v[6])");
}

/* ===================== LOOP ===================== */
void loop() {
  unsigned long now = millis();

  // -------- Vibration sample at ~200 Hz --------
  if (now - lastSample >= (1000 / SAMPLE_RATE)) {
    lastSample = now;

    sensors_event_t e1, e2;
    accel1.getEvent(&e1);
    accel2.getEvent(&e2);

    float mag1 = magFromAccel(e1);
    float mag2 = magFromAccel(e2);

    if (!isnan(mag1)) { sumSq1 += mag1 * mag1; count1++; }
    if (!isnan(mag2)) { sumSq2 += mag2 * mag2; count2++; }
  }

  // -------- Every 1 second window: compute RMS and send --------
  if (now - lastWindow >= WINDOW_MS) {
    lastWindow += WINDOW_MS; // keeps cadence stable

    float rms1 = (count1 > 0) ? sqrtf(sumSq1 / count1) : NAN;
    float rms2 = (count2 > 0) ? sqrtf(sumSq2 / count2) : NAN;

    float t1 = safeFloat(therm1.readCelsius());
    float t2 = safeFloat(therm2.readCelsius());
    float envT = safeFloat(dht.readTemperature());
    float envH = safeFloat(dht.readHumidity());

    // --- Simple TDMA offset (avoid collisions) ---
    // nodeId=1 -> 100ms, nodeId=2 -> 200ms, ...
    uint32_t slotDelay = (uint32_t)NODE_ID * SLOT_MS;
    delay(slotDelay);

    Serial.printf(
      "Node%u seq%u | v=[T1 %.1f, T2 %.1f, RMS1 %.3f, RMS2 %.3f, ET %.1f, EH %.1f]\n",
      NODE_ID, (unsigned)seqNo, t1, t2, rms1, rms2, envT, envH
    );

    sendPacket(t1, t2, rms1, rms2, envT, envH);

    // reset window
    sumSq1 = sumSq2 = 0;
    count1 = count2 = 0;
  }
}
