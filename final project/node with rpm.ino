#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <max6675.h>
#include <math.h>

/* ================= CONFIG ================= */
uint8_t m1CollectorMAC[] = {0xCC, 0xDB, 0xA7, 0x53, 0x46, 0x80};

static const uint8_t NODE_ID = 2;
static const uint32_t SLOT_MS = 100;
static const uint32_t SAMPLE_RATE = 200;
static const uint32_t WINDOW_MS = 1000;

/* ================= PACKET ================= */
typedef struct __attribute__((packed)) {
  uint8_t nodeId;
  uint16_t seq;
  float v[6];
} NodePacket;

NodePacket pkt;
uint16_t seqNo = 0;

/* ================= ADXL345 ================= */
Adafruit_ADXL345_Unified accel1(1);
Adafruit_ADXL345_Unified accel2(2);

/* ================= MAX6675 ================= */
int thermoSO = 19;
int thermoSCK = 18;
int thermoCS1 = 5;
int thermoCS2 = 4;

MAX6675 therm1(thermoSCK, thermoCS1, thermoSO);
MAX6675 therm2(thermoSCK, thermoCS2, thermoSO);

/* ================= RPM SENSOR ================= */
#define RPM_PIN 27

volatile uint32_t pulseCount = 0;

/* ================= RMS STATE ================= */
unsigned long lastSample = 0;
unsigned long lastWindow = 0;

float sumSq1 = 0, sumSq2 = 0;
int count1 = 0, count2 = 0;

/* ================= HELPERS ================= */
static float safeFloat(float x) {
  if (isnan(x) || isinf(x)) return NAN;
  return x;
}

static float magFromAccel(const sensors_event_t &e) {
  float m = sqrtf(e.acceleration.x * e.acceleration.x +
                  e.acceleration.y * e.acceleration.y +
                  e.acceleration.z * e.acceleration.z);
  return safeFloat(m);
}

/* ================= RPM INTERRUPT ================= */
void IRAM_ATTR pulseISR() {
  pulseCount++;
}

/* ================= ESP-NOW ================= */
void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send OK" : "Send FAIL");
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);

  esp_now_init();
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, m1CollectorMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

/* ================= SEND ================= */
void sendPacket(float t1, float t2, float rms1, float rms2, float rpm) {

  pkt.nodeId = NODE_ID;
  pkt.seq = seqNo++;

  pkt.v[0] = safeFloat(t1);
  pkt.v[1] = safeFloat(t2);
  pkt.v[2] = safeFloat(rms1);
  pkt.v[3] = safeFloat(rms2);
  pkt.v[4] = safeFloat(rpm);
  pkt.v[5] = NAN;

  esp_now_send(m1CollectorMAC, (uint8_t*)&pkt, sizeof(pkt));
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  initEspNow();

  accel1.begin(0x53);
  accel2.begin(0x1D);

  accel1.setRange(ADXL345_RANGE_16_G);
  accel2.setRange(ADXL345_RANGE_16_G);

  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), pulseISR, FALLING);

  lastSample = millis();
  lastWindow = millis();

  Serial.println("NODE 2 READY");
}

/* ================= LOOP ================= */
void loop() {
  unsigned long now = millis();

  // vibration sampling
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

  // every 1 sec window
  if (now - lastWindow >= WINDOW_MS) {
    lastWindow += WINDOW_MS;

    float rms1 = count1 ? sqrtf(sumSq1 / count1) : NAN;
    float rms2 = count2 ? sqrtf(sumSq2 / count2) : NAN;

    float t1 = therm1.readCelsius();
    float t2 = therm2.readCelsius();

    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float rpm = pulses * 60.0;  // adjust if multiple teeth

    delay(NODE_ID * SLOT_MS);

    Serial.printf("RPM: %.1f\n", rpm);

    sendPacket(t1, t2, rms1, rms2, rpm);

    sumSq1 = sumSq2 = 0;
    count1 = count2 = 0;
  }
}
