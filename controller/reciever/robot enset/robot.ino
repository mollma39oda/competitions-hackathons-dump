#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <QTRSensors.h>
#include <driver/ledc.h>
#include <PID_v1.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
// ==== OBSTACLE AVOIDANCE TIMING ====
#define OBST_STOP_MS     300

// Multitasking support
TaskHandle_t backgroundTask;

// Watchdog timeout config
#define WDT_TIMEOUT_SECONDS 3

// QTR config
const uint8_t SensorCount = 8;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
bool qtrBinaryValues[SensorCount];
const uint16_t QTR_THRESHOLD = 3000;
bool qtrInitialized = false;

// Motor pins
#define L_UPPER_IN1 16
#define L_UPPER_IN2 17
#define L_LOWER_IN1 15
#define L_LOWER_IN2 4
#define R_UPPER_IN1 5
#define R_UPPER_IN2 18
#define R_LOWER_IN1 19
#define R_LOWER_IN2 23
int motorPins[] = {L_UPPER_IN1, L_UPPER_IN2, L_LOWER_IN1, L_LOWER_IN2, R_UPPER_IN1, R_UPPER_IN2, R_LOWER_IN1, R_LOWER_IN2};
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8 // 0-255

#define MAX_TOTAL_PWM 900
#define MIN_PWM 0

// --- VOLTAGE-ADAPTED PWM LIMITS ---
#define MAX_PWM_NORMAL 191   // 9V out of 12V supply (normal speed)
#define MAX_PWM_BOOST  234   // 11V out of 12V supply (boost speed)
#define SPEED_MIN_RUN_NORMAL (MAX_PWM_NORMAL * 30 / 100)
#define SPEED_MIN_RUN_BOOST  (MAX_PWM_BOOST * 30 / 100)

// Stick/trigger mapping (fixed, no calibration)
const int rightMin = 550, rightMax = 2600, rightCenter = 1600;
const int leftMin = 650, leftMax = 3000, leftCenter = 1750;
const int rtMin = 1400, rtMax = 2300;

// --- PID for line follower ---
double setpoint = 0;
double input = 0, output = 0;
double Kp = 50, Ki = 0.5, Kd = 10;
PID linePID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ESP-NOW data structure
typedef struct {
  int right;
  int rt;
  int left;
  uint8_t button; // 1=pressed, 0=not pressed
} ControlData;

ControlData incomingData = {0, 0, 0, 0};
unsigned long lastDataReceived = 0;
const unsigned long dataTimeout = 3000; // 3 seconds timeout

// --- MODES ---
enum RobotMode { MODE_CONTROLLER, MODE_LINEFOLLOWER };
RobotMode currentMode = MODE_CONTROLLER;
uint8_t lastButtonState = 0;

// --- DIR ENUM FOR SMOOTH DRIVE ---
enum DriveDir { DIR_STOP=0, DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT };
DriveDir lastDriveDir = DIR_STOP;

// --- OBSTACLE SEQUENCE LOGIC (for linefollower ONLY) ---
enum ObstacleState { OBST_NONE, OBST_SEQUENCE };
ObstacleState obstState = OBST_NONE;
static int obstStep = 0;
static unsigned long obstStepStart = 0;
const int OBST_SEQ_DUR[] = {400, 400, 400, 400}; // ms for RIGHT, FWD, LEFT, FWD (tweak if needed)
const int OBST_SEQ_NUM = 4;

// --- ESP-NOW callback
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(ControlData));
  lastDataReceived = millis();
}
void stopWiFi() {
  WiFi.disconnect(true);      // Disconnect STA/AP
  WiFi.mode(WIFI_OFF);        // Turn off WiFi mode
  esp_wifi_stop();            // Stop the WiFi driver
  esp_wifi_deinit();          // Deinitialize the WiFi driver
  delay(100);                 // Give hardware time to shut down
}

// === MOTOR MOVEMENTS ===
void Stop() {
  for (int i = 0; i < 8; i++) ledcWriteChannel(i, 0);
}
void RIGHT(int speed) {
    int absSpeed = constrain(abs(speed), MIN_PWM, MAX_PWM_BOOST);
    ledcWriteChannel(0, absSpeed); ledcWriteChannel(1, 0);
    ledcWriteChannel(2, absSpeed); ledcWriteChannel(3, 0);
    ledcWriteChannel(4, 0); ledcWriteChannel(5, absSpeed);
    ledcWriteChannel(6, 0); ledcWriteChannel(7, absSpeed);
}
void LEFT(int speed) {
    int absSpeed = constrain(abs(speed), MIN_PWM, MAX_PWM_BOOST);
    ledcWriteChannel(0, 0); ledcWriteChannel(1, absSpeed);
    ledcWriteChannel(2, 0); ledcWriteChannel(3, absSpeed);
    ledcWriteChannel(4, absSpeed); ledcWriteChannel(5, 0);
    ledcWriteChannel(6, absSpeed); ledcWriteChannel(7, 0);
}
void FORWARD(int speed) {
    int absSpeed = constrain(abs(speed), MIN_PWM, MAX_PWM_BOOST);
    ledcWriteChannel(0, absSpeed); ledcWriteChannel(1, 0);
    ledcWriteChannel(2, absSpeed); ledcWriteChannel(3, 0);
    ledcWriteChannel(4, absSpeed); ledcWriteChannel(5, 0);
    ledcWriteChannel(6, absSpeed); ledcWriteChannel(7, 0);
}
void BACKWARD(int speed) {
    int absSpeed = constrain(abs(speed), MIN_PWM, MAX_PWM_BOOST);
    ledcWriteChannel(0, 0); ledcWriteChannel(1, absSpeed);
    ledcWriteChannel(2, 0); ledcWriteChannel(3, absSpeed);
    ledcWriteChannel(4, 0); ledcWriteChannel(5, absSpeed);
    ledcWriteChannel(6, 0); ledcWriteChannel(7, absSpeed);
}

void safeDrive(int lSpeed, int rSpeed) {
  lSpeed = constrain(lSpeed, MIN_PWM, MAX_PWM_BOOST);
  rSpeed = constrain(rSpeed, MIN_PWM, MAX_PWM_BOOST);
  int total = 2 * lSpeed + 2 * rSpeed;
  if (total > MAX_TOTAL_PWM) {
    float scale = (float)MAX_TOTAL_PWM / (float)total;
    lSpeed = lSpeed * scale;
    rSpeed = rSpeed * scale;
  }
  ledcWriteChannel(0, lSpeed); ledcWriteChannel(1, 0);
  ledcWriteChannel(2, lSpeed); ledcWriteChannel(3, 0);
  ledcWriteChannel(4, rSpeed); ledcWriteChannel(5, 0);
  ledcWriteChannel(6, rSpeed); ledcWriteChannel(7, 0);
}

// --- Smooth drive (no timed stop) ---
// Modified: Add 100ms stop before changing direction, unless STOP or same direction
void smoothDrive(DriveDir newDir, int speed) {
  int cappedSpeed = constrain(abs(speed), MIN_PWM, MAX_PWM_BOOST);

  // Only stop if changing to a different direction (not STOP)
  if (newDir != lastDriveDir && lastDriveDir != DIR_STOP && newDir != DIR_STOP) {
    Stop();
    delay(100); // Stop for 100ms before changing direction
  }

  switch (newDir) {
    case DIR_FORWARD:  FORWARD(cappedSpeed); break;
    case DIR_BACKWARD: BACKWARD(cappedSpeed); break;
    case DIR_LEFT:     LEFT(cappedSpeed); break;
    case DIR_RIGHT:    RIGHT(cappedSpeed); break;
    default:           Stop(); break;
  }
  lastDriveDir = newDir;
}

// --- BUTTON MENU LOGIC ---
void menuDisplay() {
  // Display logic removed since OLED (I2C) is not used.
  Serial.println("Select Mode:");
  Serial.println(currentMode == MODE_CONTROLLER ? "> CONTROLLER" : "  CONTROLLER");
  Serial.println(currentMode == MODE_LINEFOLLOWER ? "> LINE FOLLOWER" : "  LINE FOLLOWER");
}

// --- QTR SETUP AND CALIBRATION ---
void setupQTR() {
  if (qtrInitialized) return;
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){14, 27, 26, 25, 33, 32, 35, 34}, SensorCount);
  qtr.setEmitterPin(13);
  qtrInitialized = true;
}

void updateQTRBinary() {
  qtr.read(sensorValues);
  for (int i = 0; i < SensorCount; i++) {
    qtrBinaryValues[i] = (sensorValues[i] > QTR_THRESHOLD);
  }
}

// --- WIFI/ESP-NOW MANAGEMENT ---
void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int attempts = 0;
  while (attempts < 3) {
    if (esp_now_init() == ESP_OK) {
      esp_now_register_recv_cb(onDataRecv);
      Serial.println("ESP-NOW initialized successfully");
      break;
    } else {
      attempts++;
      Serial.println("ESP-NOW init failed, retrying...");
      delay(200);
    }
  }
  if (attempts >= 3) {
    Serial.println("ESP-NOW init failed after 3 attempts");
    ESP.restart();
  }
}
void stopESPNow() {
  esp_now_unregister_recv_cb();
  esp_now_deinit();
}

// --- STICK LOGIC ---
int clamp(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}
int joyToSpeed(int raw, int minVal, int maxVal, int centerVal, int speedCap, int minRun) {
  int deadzone = 5;
  raw = clamp(raw, minVal, maxVal);
  int s = 0;
  if (raw > centerVal + deadzone) {
    s = (raw - centerVal) * speedCap / (maxVal - centerVal);
    if (s < minRun) s = 0;
  } else if (raw < centerVal - deadzone) {
    s = -(centerVal - raw) * speedCap / (centerVal - minVal);
    if (abs(s) < minRun) s = 0;
  }
  return s;
}
const char* dirToStr(int s, bool isRightStick = false) {
  if (isRightStick) {
    if (s > 0) return "RIGHT";
    if (s < 0) return "LEFT";
    return "CENTER";
  } else {
    if (s > 0) return "FRONT";
    if (s < 0) return "BACK";
    return "CENTER";
  }
}
void Brake() {
    for (int i = 0; i < 8; i++) {
        ledcWriteChannel(i, 180);
    }
    delay(5);
    Stop();
}
void driveTank_smart(ControlData data) {
    bool boost = (data.rt >= rtMax - (rtMax - rtMin) * 10 / 100);
    int speedCap = boost ? MAX_PWM_BOOST : MAX_PWM_NORMAL;
    int minRun = boost ? SPEED_MIN_RUN_BOOST : SPEED_MIN_RUN_NORMAL;
    int leftSpeed = joyToSpeed(data.left, leftMin, leftMax, leftCenter, speedCap, minRun);
    int rightSpeed = joyToSpeed(data.right, rightMin, rightMax, rightCenter, speedCap, minRun);
    if (rightSpeed != 0) {
        if (rightSpeed > 0) {
            smoothDrive(DIR_RIGHT, rightSpeed);
        } else {
            smoothDrive(DIR_LEFT, -rightSpeed);
        }
    } else if (leftSpeed != 0) {
        if (leftSpeed > 0) {
            smoothDrive(DIR_FORWARD, leftSpeed);
        } else {
            smoothDrive(DIR_BACKWARD, -leftSpeed);
        }
    } else {
        Brake();
    }
}
void backgroundTaskCode(void * parameter) {
  esp_task_wdt_add(NULL);
  for(;;) {
    esp_task_wdt_reset();
    if (currentMode == MODE_LINEFOLLOWER && qtrInitialized) {
      updateQTRBinary();
    }
    delay(50);
  }
}

// --- OBSTACLE AVOIDANCE (with direct HC-SR04 trigger/echo) ---
#define SONAR_TRIG_PIN 12
#define SONAR_ECHO_PIN 2
#define SONAR_MAX_DIST_CM 200

void setupSonar() {
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  digitalWrite(SONAR_TRIG_PIN, LOW);
}

// Returns distance in cm, or 0 if no echo (out of range)
unsigned int getSonarDistanceCM() {
  // Trigger pulse
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  // Read echo
  long duration = pulseIn(SONAR_ECHO_PIN, HIGH, SONAR_MAX_DIST_CM * 58 * 2); // timeout
  if (duration == 0) return SONAR_MAX_DIST_CM; // No echo
  unsigned int distance = duration / 58;
  return distance;
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = (1 << 0),
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  for (int i = 0; i < 8; i++) {
    pinMode(motorPins[i], OUTPUT);
    ledcAttachChannel(motorPins[i], PWM_FREQUENCY, PWM_RESOLUTION, i);
  }
  Stop();
  setupESPNow();
  setupSonar();
  xTaskCreatePinnedToCore(
    backgroundTaskCode,
    "BackgroundTask",
    10000,
    NULL,
    1,
    &backgroundTask,
    0);
  menuDisplay();
  esp_task_wdt_reset();
}

// --- LOOP ---
void loop() {
  esp_task_wdt_reset();
  static unsigned long buttonPressStart = 0;
  static bool inMenu = true;
  static bool qtrCalibrated = false;
  if (inMenu) {
    menuDisplay();
    if (incomingData.button && !lastButtonState) {
      buttonPressStart = millis();
    } else if (!incomingData.button && lastButtonState) {
      unsigned long pressDuration = millis() - buttonPressStart;
      if (pressDuration < 1000) {
        currentMode = (RobotMode)((currentMode + 1) % 2);
      } else {
        inMenu = false;
        Serial.print("Starting ");
        if (currentMode == MODE_CONTROLLER) {
          Serial.println("Controller");
        } else if (currentMode == MODE_LINEFOLLOWER) {
          Serial.println("Line Follower");
          setupQTR();
          stopESPNow();
          delay(100);
          stopWiFi();
        }
        delay(800);
      }
    }
    lastButtonState = incomingData.button;
    delay(100);
    return;
  }
  if (currentMode == MODE_CONTROLLER) {
    if (millis() - lastDataReceived > dataTimeout) {
      Stop();
      Serial.println("CONNECTION LOST");
      Serial.println("Waiting for controller...");
      if (millis() - lastDataReceived > dataTimeout * 3) {
        stopESPNow();
        delay(200);
        setupESPNow();
        lastDataReceived = millis();
      }
      delay(100);
      return;
    }
    driveTank_smart(incomingData);
    bool boost = (incomingData.rt >= rtMax - (rtMax - rtMin) * 10 / 100);
    int speedCap = boost ? MAX_PWM_BOOST : MAX_PWM_NORMAL;
    int minRun = boost ? SPEED_MIN_RUN_BOOST : SPEED_MIN_RUN_NORMAL;
    int leftSpeed = joyToSpeed(incomingData.left, leftMin, leftMax, leftCenter, speedCap, minRun);
    int rightSpeed = joyToSpeed(incomingData.right, rightMin, rightMax, rightCenter, speedCap, minRun);
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 200) {
      Serial.printf("LEFT: %s %d\n", dirToStr(leftSpeed), abs(leftSpeed));
      Serial.printf("RIGHT: %s %d\n", dirToStr(rightSpeed, true), abs(rightSpeed));
      Serial.printf("RT: %s\n", boost ? "BOOST" : "NORMAL");
      Serial.printf("Button:%d\n", incomingData.button);
      lastDisplayUpdate = millis();
    }
    delay(20);
    return;
  }

  // --- LINE FOLLOWER MODE WITH OBSTACLE AVOIDANCE ---
  if (currentMode == MODE_LINEFOLLOWER) {
    static int lastErrorSign = 0;
    esp_task_wdt_reset();
    qtr.read(sensorValues);
    uint8_t digitalSensors[SensorCount];
    for (int i = 0; i < SensorCount; i++) {
      digitalSensors[i] = (sensorValues[i] < QTR_THRESHOLD) ? 1 : 0;
    }
    int weights[SensorCount] = {-3, -2, -1, 0, 0, 1, 2, 3};
    int sum = 0, count = 0;
    for (int i = 0; i < SensorCount; i++) {
      sum += digitalSensors[i] * weights[i];
      count += digitalSensors[i];
    }

    // --- OBSTACLE AVOIDANCE LOGIC, SEQUENCE: RIGHT, FWD, LEFT, FWD ---
    unsigned int obstDist = getSonarDistanceCM();
    if (obstDist == 0) obstDist = SONAR_MAX_DIST_CM;

    if (obstState == OBST_NONE) {
      if (obstDist < 20) {
        obstState = OBST_SEQUENCE;
        obstStep = 0;
        obstStepStart = millis();
        Stop();
        Serial.println("Obstacle!");
        return;
      }
    } else if (obstState == OBST_SEQUENCE) {
      // Sequence: RIGHT, FWD, LEFT, FWD
      if (obstStep == 0) {
        smoothDrive(DIR_RIGHT, MAX_PWM_NORMAL);
        Serial.println("Avoid: RIGHT");
      } else if (obstStep == 1) {
        smoothDrive(DIR_FORWARD, MAX_PWM_NORMAL);
        Serial.println("Avoid: FWD1");
      } else if (obstStep == 2) {
        smoothDrive(DIR_LEFT, MAX_PWM_NORMAL);
        Serial.println("Avoid: LEFT");
      } else if (obstStep == 3) {
        smoothDrive(DIR_FORWARD, MAX_PWM_NORMAL);
        Serial.println("Avoid: FWD2");
      }
      if (millis() - obstStepStart > OBST_SEQ_DUR[obstStep]) {
        obstStep++;
        obstStepStart = millis();
      }
      if (obstStep >= OBST_SEQ_NUM) {
        obstState = OBST_NONE;
        obstStep = 0;
        Stop();
        // Optionally: wait a moment or check for line reacquire
      }
      return;
    }

    // --- LOST LINE HANDLING (ALL BLACK/ALL WHITE) ---
    static int lostLineStep = 0;
    static unsigned long lostLineLastTime = 0;
    const unsigned long LOST_LINE_STEP_MS = 400; // How long to try each step before moving to next
    if (count == 0 || count == SensorCount) {
      // Lost line: alternate right, forward, left, forward
      if (millis() - lostLineLastTime > LOST_LINE_STEP_MS) {
        lostLineStep = (lostLineStep + 1) % 4;
        lostLineLastTime = millis();
      }
      switch (lostLineStep) {
        case 0: smoothDrive(DIR_RIGHT, MAX_PWM_NORMAL); break;
        case 1: smoothDrive(DIR_FORWARD, MAX_PWM_NORMAL); break;
        case 2: smoothDrive(DIR_LEFT, MAX_PWM_NORMAL); break;
        case 3: smoothDrive(DIR_FORWARD, MAX_PWM_NORMAL); break;
      }
      return;
    } else {
      lostLineStep = 0; // Reset on line reacquired
      input = (double)sum / count;
      if (input > 0) lastErrorSign = 1;
      else if (input < 0) lastErrorSign = -1;
    }
    linePID.Compute();
    int baseSpeed = MAX_PWM_NORMAL;
    int error = input;
    if (abs(error) <= 1) {
      int leftSpeed = baseSpeed - output;
      int rightSpeed = baseSpeed + output;
      safeDrive(leftSpeed, rightSpeed);
      lastDriveDir = DIR_STOP;
    } else if (error > 1) {
      smoothDrive(DIR_RIGHT, MAX_PWM_NORMAL);
    } else if (error < -1) {
      smoothDrive(DIR_LEFT, MAX_PWM_NORMAL);
    }
    Serial.print("D:");
    for (int i = 0; i < SensorCount; i++) Serial.print(digitalSensors[i]);
    Serial.printf(" Err:%d PID:%.1f\n", error, output);
    delay(10);
    return;
  }
}