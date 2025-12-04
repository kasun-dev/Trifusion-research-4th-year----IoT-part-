#include <U8g2lib.h>
#include <Wire.h>

// U8G2 Constructor (SSD1306 128x64 I2C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ----------------------
//  CONFIGURABLE VALUES
// ----------------------
bool emergencyMode = true;     // true = E (blinks), false = N
bool wifiConnected = true;      // WiFi ✓ or ✗
int nodeCount = 3;              // Example: 3 nodes
bool blinkState = false;        // Used for blinking

unsigned long lastBlink = 0;

void setup() {
  Wire.begin(21, 22);   // your SDA, SCL
  u8g2.begin();

  Serial.begin(115200);
}

void loop() {

  // Toggle blink state every 500ms
  if (millis() - lastBlink > 500) {
    blinkState = !blinkState;
    lastBlink = millis();
  }

  drawScreen();
}

void drawScreen() {
  u8g2.clearBuffer();

  // -------------------
  // MODE DISPLAY (Size 2)
  // -------------------
  u8g2.setFont(u8g2_font_logisoso16_tf); // Clean industrial font
  u8g2.setCursor(5, 20);

  if (emergencyMode) {
    if (blinkState) {
      u8g2.print("E");      // blinking E
    }
  } else {
    u8g2.print("N");        // normal mode, solid N
  }

  // Label text
  u8g2.setFont(u8g2_font_6x13_tf);  
  u8g2.setCursor(30, 18);
  u8g2.print("MODE");

  // -------------------
  // WIFI STATUS (✓ or ✗)
  // -------------------
  u8g2.setCursor(5, 35);
  u8g2.print("WiFi:");

  if (wifiConnected) {
    u8g2.print(" ✓");
  } else {
    u8g2.print(" ✗");
  }

  // -------------------
  // NODE COUNT
  // -------------------
  u8g2.setCursor(70, 35);
  u8g2.print("Nodes: ");
  u8g2.print(nodeCount);

  // -------------------
  // BLINKING DATA DOT
  // -------------------
  if (blinkState) {
    u8g2.setFont(u8g2_font_squeezed_r6_tr);
    u8g2.drawDisc(64, 55, 3); // center bottom
  }

  u8g2.sendBuffer();
}

