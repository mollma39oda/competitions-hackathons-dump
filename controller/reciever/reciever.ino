#include <WiFi.h>
#include <esp_now.h>

// Data structure must match the sender!
typedef struct {
  int right;
  int rt;
  int left;
  uint8_t button;
} ControlData;

ControlData incomingData;

// Calibration values (update as needed)
int rightMin = 550, rightMax = 2600, rightCenter = 1600;
int leftMin = 650, leftMax = 3000, leftCenter = 1750;
int rtMin = 1400, rtMax = 2300;

const int SPEED_MAX = 255;
const int SPEED_MIN_RUN = SPEED_MAX * 30 / 100; // 30% threshold

// Calibration mode
bool calibrationActive = true;
unsigned long calibrationStartTime;
const unsigned long CALIBRATION_DURATION = 10000; // 10 seconds

// For printing
bool dataReady = false;
unsigned long lastPrintTime = 0;

// Utility to get stick direction and speed, clamping input to min/max
enum StickDir { CENTER, FORWARD, BACKWARD, LEFT, RIGHT };

struct StickInfo {
  StickDir direction;
  int speed;
};

// Clamp a value between min and max
int clamp(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

StickInfo getStickInfo(int raw, int minVal, int maxVal, int centerVal, bool limit80 = false) {
  StickInfo info;
  int deadzone = 5; // adjust as needed

  // Clamp raw value between min and max
  raw = clamp(raw, minVal, maxVal);

  if (raw > centerVal + deadzone) {
    info.direction = FORWARD; // or RIGHT for right stick
    info.speed = (raw - centerVal) * SPEED_MAX / (maxVal - centerVal);
    if (limit80 && info.speed > SPEED_MAX * 80 / 100) info.speed = SPEED_MAX * 80 / 100;
    if (info.speed < SPEED_MIN_RUN) info.speed = 0;
  } else if (raw < centerVal - deadzone) {
    info.direction = BACKWARD; // or LEFT for right stick
    info.speed = (centerVal - raw) * SPEED_MAX / (centerVal - minVal);
    if (limit80 && info.speed > SPEED_MAX * 80 / 100) info.speed = SPEED_MAX * 80 / 100;
    if (info.speed < SPEED_MIN_RUN) info.speed = 0;
  } else {
    info.direction = CENTER;
    info.speed = 0;
  }
  return info;
}

// RT logic: only enables movement when "full throttle"
bool isRTFull(int raw) {
  raw = clamp(raw, rtMin, rtMax);
  int threshold = rtMax - (rtMax - rtMin) * 10 / 100; // top 10%
  return (raw >= threshold);
}

const char* dirToStr(StickDir dir, bool isRightStick = false) {
  if (isRightStick) {
    switch(dir) {
      case FORWARD: return "RIGHT";
      case BACKWARD: return "LEFT";
      case CENTER: return "CENTER";
      default: return "";
    }
  } else {
    switch(dir) {
      case FORWARD: return "FRONT";
      case BACKWARD: return "BACK";
      case CENTER: return "CENTER";
      default: return "";
    }
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(ControlData));
  dataReady = true;

  // During calibration, update min/max
  if (calibrationActive) {
    if (incomingData.right > rightMax) rightMax = incomingData.right;
    if (incomingData.right < rightMin) rightMin = incomingData.right;

    if (incomingData.left > leftMax) leftMax = incomingData.left;
    if (incomingData.left < leftMin) leftMin = incomingData.left;

    if (incomingData.rt > rtMax) rtMax = incomingData.rt;
    if (incomingData.rt < rtMin) rtMin = incomingData.rt;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("==== Receiver Calibration Mode ====");
  Serial.println("Move all sticks and trigger on the controller to all extremes for 10 seconds...");
  calibrationStartTime = millis();
}

void loop() {
  // Calibration timing
  if (calibrationActive && millis() - calibrationStartTime > CALIBRATION_DURATION) {
    calibrationActive = false;
    rightCenter = (rightMax + rightMin) / 2;
    leftCenter  = (leftMax + leftMin) / 2;

    Serial.println("\n==== Calibration COMPLETE! ====");
    Serial.printf("RIGHT_MIN: %d\n", rightMin);
    Serial.printf("RIGHT_MAX: %d\n", rightMax);
    Serial.printf("RIGHT_CENTER: %d\n", rightCenter);

    Serial.printf("LEFT_MIN: %d\n", leftMin);
    Serial.printf("LEFT_MAX: %d\n", leftMax);
    Serial.printf("LEFT_CENTER: %d\n", leftCenter);

    Serial.printf("RT_MIN: %d\n", rtMin);
    Serial.printf("RT_MAX: %d\n", rtMax);

    Serial.println("Copy these values to your code if needed!");
  }

  // Print every 500ms after calibration
  unsigned long printInterval = calibrationActive ? 200 : 500;
  if (dataReady && millis() - lastPrintTime >= printInterval) {
    if (!calibrationActive) {
      StickInfo left = getStickInfo(incomingData.left, leftMin, leftMax, leftCenter, true);
      StickInfo right = getStickInfo(incomingData.right, rightMin, rightMax, rightCenter, true);

      bool rt_full = isRTFull(incomingData.rt);

      Serial.printf(
        "LEFT: %s, speed: %d | RIGHT: %s, speed: %d | RT: %s\n",
        dirToStr(left.direction, false),
        left.speed,
        dirToStr(right.direction, true),
        right.speed,
        rt_full ? "FULL" : "NOT FULL"
      );
    }
    lastPrintTime = millis();
    dataReady = false;
  }
}