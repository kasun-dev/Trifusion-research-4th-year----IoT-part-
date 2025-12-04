#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <max6675.h>
#include <DHT.h>

// ============================================================
// MAIN ESP32 (MASTER) MAC ADDRESS
// ============================================================
uint8_t mainMAC[] = {0xCC, 0xDB, 0xA7, 0x53, 0x46, 0x80};

// ============================================================
// STANDARD PACKET (MATCHES MAIN BOARD)
// ============================================================
typedef struct {
  uint8_t nodeID;   // unique per node (1 = shaft)
  float t1;         // MAX6675 temp1
  float t2;         // MAX6675 temp2
  float v1;         // RMS1
  float v2;         // RMS2
  float rpm;        // repurposed → DHT temp (et)
  float voltage;    // repurposed → DHT humidity (eh)
} SensorPacket;

SensorPacket packet;

// ============================================================
// ESP-NOW SEND CALLBACK (IDF 5.x)
// ============================================================
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ============================================================
// ESP-NOW INIT
// ============================================================
void initEspNow() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    ESP.restart();
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mainMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer!");
  }
}

// ============================================================
// ADXL345 SENSORS
// ============================================================
Adafruit_ADXL345_Unified accel1(1);  // 0x53
Adafruit_ADXL345_Unified accel2(2);  // 0x1D

// ============================================================
// MAX6675 (thermocouples)
// ============================================================
int thermoSO = 19;
int thermoSCK = 18;
int thermoCS1 = 5;
int thermoCS2 = 4;

MAX6675 therm1(thermoSCK, thermoCS1, thermoSO);
MAX6675 therm2(thermoSCK, thermoCS2, thermoSO);

// ============================================================
// DHT22 SENSOR
// ============================================================
#define DHTPIN  25       // your chosen pin
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

// ============================================================
// RMS SETTINGS
// ============================================================
const int SAMPLE_RATE = 200;
const int WINDOW_MS   = 1000;

unsigned long lastSample = 0;
unsigned long lastRms = 0;

float sumSq1 = 0, sumSq2 = 0;
int count1 = 0, count2 = 0;

// ============================================================
// SEND PACKET TO MAIN NODE
// ============================================================
void sendToMain(float rms1, float rms2, float t1, float t2, float et, float eh) {
  packet.nodeID = 1;   // SH AFT NODE ID

  packet.v1 = rms1;
  packet.v2 = rms2;

  packet.t1 = t1;
  packet.t2 = t2;

  packet.rpm = et;       // repurposed for DHT TEMP
  packet.voltage = eh;   // repurposed for DHT HUMIDITY

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

  if (!accel1.begin(0x53)) { Serial.println("ADXL #1 missing!"); while(1); }
  if (!accel2.begin(0x1D)) { Serial.println("ADXL #2 missing!"); while(1); }

  accel1.setRange(ADXL345_RANGE_16_G);
  accel2.setRange(ADXL345_RANGE_16_G);
  accel1.setDataRate(ADXL345_DATARATE_400_HZ);
  accel2.setDataRate(ADXL345_DATARATE_400_HZ);

  Serial.println("SHAFT NODE READY (ADXL x2, MAX6675 x2, DHT22)");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // ---- VIBRATION SAMPLING ----
  if (now - lastSample >= (1000 / SAMPLE_RATE)) {
    lastSample = now;

    sensors_event_t e1, e2;
    accel1.getEvent(&e1);
    accel2.getEvent(&e2);

    float mag1 = sqrt(e1.acceleration.x * e1.acceleration.x +
                      e1.acceleration.y * e1.acceleration.y +
                      e1.acceleration.z * e1.acceleration.z);

    float mag2 = sqrt(e2.acceleration.x * e2.acceleration.x +
                      e2.acceleration.y * e2.acceleration.y +
                      e2.acceleration.z * e2.acceleration.z);

    sumSq1 += mag1 * mag1;
    sumSq2 += mag2 * mag2;

    count1++;
    count2++;
  }

  // ---- EVERY 1s SEND RMS + TEMPS ----
  if (now - lastRms >= WINDOW_MS) {
    lastRms = now;

    float rms1 = count1 ? sqrt(sumSq1 / count1) : -1;
    float rms2 = count2 ? sqrt(sumSq2 / count2) : -1;

    float t1 = therm1.readCelsius();
    float t2 = therm2.readCelsius();

    float et = dht.readTemperature();
    float eh = dht.readHumidity();

    Serial.printf("RMS1: %.3f | RMS2: %.3f | T1: %.1f | T2: %.1f | ET: %.1f | EH: %.1f\n",
                  rms1, rms2, t1, t2, et, eh);

    sendToMain(rms1, rms2, t1, t2, et, eh);

    sumSq1 = sumSq2 = 0;
    count1 = count2 = 0;
  }
}
