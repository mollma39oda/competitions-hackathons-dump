#include <Arduino.h>
#include <Wire.h>
#include <QTRSensors.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// QTR sensor configuration
const uint8_t SensorCount = 8;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

// OLED configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  // QTR sensor pins (update for your wiring)
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){14, 27, 26, 25, 33, 32, 35, 34}, SensorCount);
  qtr.setEmitterPin(13);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED failed");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("QTR RAW VIEW");
  display.display();
  delay(1000);
}

void loop() {
  qtr.read(sensorValues);

  // Prepare and show values
  display.clearDisplay();
  display.setTextSize(1);

  // Show as 4 lines, 3 values per line, left to right, sensors 0..7
  // Layout:
  // 0,1,2
  // 3,4,5
  // 6,7
  // But you want 3 per line, so: 0,1,2 | 3,4,5 | 6,7, -
  // To get a 4x3 grid, pad last with '-'
  // To fit on 4 lines: 0,1,2 | 3,4,5 | 6,7,-

  // Actually, as you have 8 sensors, let's do:
  // Line 1: 0,1,2
  // Line 2: 3,4,5
  // Line 3: 6,7,-
  // Line 4: (blank or repeat 0,1,2 for clarity)

  // Line 1: 0,1,2
  display.setCursor(0, 0);
  display.printf("%4u,%4u,%4u", sensorValues[0], sensorValues[1], sensorValues[2]);
  // Line 2: 3,4,5
  display.setCursor(0, 8);
  display.printf("%4u,%4u,%4u", sensorValues[3], sensorValues[4], sensorValues[5]);
  // Line 3: 6,7,-
  display.setCursor(0, 16);
  display.printf("%4u,%4u,   -", sensorValues[6], sensorValues[7]);
  // Line 4: (optional: empty or repeat first 3)
  display.setCursor(0, 24);
  display.print("QTR RAW");

  display.display();

  // Also print to Serial for convenience
  Serial.print("QTR: ");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    if (i < SensorCount - 1) Serial.print(",");
  }
  Serial.println();

  delay(150);
}