#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Sampling settings
const int SAMPLE_RATE = 200;          // 200 samples per second
const int WINDOW_MS = 1000;           // 1-second RMS window

unsigned long lastSampleTime = 0;
unsigned long lastRmsTime = 0;

float sumSquare = 0;
int sampleCount = 0;

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);  // ESP32 I2C

  if (!accel.begin()) {
    Serial.println("ADXL345 NOT FOUND!");
    while (1);
  }

  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_400_HZ); // stable sampling

  Serial.println("ADXL345 RMS Vibration Test Running...");
}

void loop() {
  unsigned long now = millis();

  // ---- Sampling block ----
  if (now - lastSampleTime >= (1000 / SAMPLE_RATE)) {
    lastSampleTime = now;

    sensors_event_t e;
    accel.getEvent(&e);

    // Calculate magnitude
    float x = e.acceleration.x;
    float y = e.acceleration.y;
    float z = e.acceleration.z;

    float mag = sqrt(x*x + y*y + z*z);

    // Add squared magnitude
    sumSquare += (mag * mag);
    sampleCount++;
  }

  // ---- RMS every 1 second ----
  if (now - lastRmsTime >= WINDOW_MS) {
    lastRmsTime = now;

    if (sampleCount > 0) {
      float rms = sqrt(sumSquare / sampleCount);

      // Output (or send via ESP-NOW)
      Serial.print("RMS: ");
      Serial.println(rms, 4);

      // If sending:
      // sendToMainNode(rms);

      // Reset window
      sumSquare = 0;
      sampleCount = 0;
    }
  }
}
