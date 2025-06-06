#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>

// Pin definitions
#define RIGHT_PIN 0
#define RT_PIN    1
#define LEFT_PIN  2
#define BUTTON_PIN D4  // Push button

// MAC Address of receiver (robot)
uint8_t receiverMac[] = {0xEC, 0xE3, 0x34, 0xBF, 0x5D, 0x84};

// Data structure sent to robot
typedef struct {
  int right;
  int rt;
  int left;
  uint8_t button; // 1 = pressed, 0 = not pressed
} ControlData;

ControlData controlData, lastControlData;
unsigned long lastSentTime = 0;

// Button debouncing
volatile bool buttonPressed = false;
volatile unsigned long lastInterruptTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long CLICK_TIMEOUT = 500;
const int CLICKS_FOR_OTA = 5;
int buttonPressCount = 0;
unsigned long firstClickTime = 0;
bool buttonReleased = true;

// Analog smoothing
#define ANALOG_SAMPLES 10
int rightBuffer[ANALOG_SAMPLES] = {0};
int rtBuffer[ANALOG_SAMPLES] = {0};
int leftBuffer[ANALOG_SAMPLES] = {0};
int analogIndex = 0;

// Button ISR
void ARDUINO_ISR_ATTR buttonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > DEBOUNCE_DELAY) {
    buttonPressed = true;
    lastInterruptTime = currentTime;
  }
}

// ESP-NOW callback
void onDataSent(const uint8_t *, esp_now_send_status_t) {}

void setup() {
  Serial.begin(115200);

  pinMode(RIGHT_PIN, INPUT);
  pinMode(RT_PIN, INPUT);
  pinMode(LEFT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);

  WiFi.mode(WIFI_STA);

  // ESP-NOW init and peer
  esp_now_init();
  esp_now_register_send_cb(onDataSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  controlData = {0, 0, 0, 0};
  lastControlData = controlData;
  sendData();
}

void loop() {
  // OTA mode activation by 5 rapid clicks
  if (buttonPressed) {
    handleButton();
    buttonPressed = false;
  }

  readInputs();
  if (millis() - lastSentTime > 50 ||
      controlData.right != lastControlData.right ||
      controlData.left != lastControlData.left ||
      controlData.rt != lastControlData.rt ||
      controlData.button != lastControlData.button) {
    sendData();
    lastControlData = controlData;
    lastSentTime = millis();
  }
}

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading == LOW && buttonReleased) {
    buttonReleased = false;
    controlData.button = 1;
  } else if (reading == HIGH && !buttonReleased) {
    buttonReleased = true;
    controlData.button = 0;
    if (buttonPressCount == 0) firstClickTime = millis();
    buttonPressCount++;
    if (buttonPressCount >= CLICKS_FOR_OTA &&
        (millis() - firstClickTime < CLICKS_FOR_OTA * CLICK_TIMEOUT)) {
      activateOTAMode();
      buttonPressCount = 0;
    }
  }
}

void readInputs() {
  rightBuffer[analogIndex] = analogRead(RIGHT_PIN);
  rtBuffer[analogIndex] = analogRead(RT_PIN);
  leftBuffer[analogIndex] = analogRead(LEFT_PIN);
  analogIndex = (analogIndex + 1) % ANALOG_SAMPLES;

  int rawRight = averageArray(rightBuffer, ANALOG_SAMPLES);
  int rawRT = averageArray(rtBuffer, ANALOG_SAMPLES);
  int rawLeft = averageArray(leftBuffer, ANALOG_SAMPLES);

  controlData.right = rawRight;
  controlData.rt = rawRT;
  controlData.left = rawLeft;
}

int averageArray(int* arr, int size) {
  long sum = 0;
  for (int i = 0; i < size; i++) sum += arr[i];
  return sum / size;
}

void sendData() {
  esp_now_send(receiverMac, (uint8_t*)&controlData, sizeof(controlData));
  // Blink LED for feedback
  digitalWrite(LED_BUILTIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(LED_BUILTIN, LOW);
}

void activateOTAMode() {
  Serial.println("Entering OTA mode...");
  esp_now_deinit();
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32_Controller", "12345678");
  ArduinoOTA.setHostname("ESP32-Controller");
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.begin();
  Serial.println("OTA ready. Connect to WiFi AP and upload new program.");
  // Blink LED in OTA mode
  while (true) {
    ArduinoOTA.handle();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
}