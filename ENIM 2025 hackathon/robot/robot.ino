#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

// ===== Motor Driver Pins (L298N) =====
#define ENA  4
#define IN_1 17
#define IN_2 16
#define IN_3 5
#define IN_4 18
#define ENB 19
#define SERVO_PIN 13

// ===== Line Follower Config =====
const int MaxSpeed = 140; // For line follower mode

const int NUM_SENSORS = 5;
const int sensorPins[NUM_SENSORS] = {33, 25, 26, 27, 14};
const int sensorWeights[NUM_SENSORS] = {2, 1, 0, -1, -2};
int sensorValues[NUM_SENSORS];

// PID parameters for line follower
double Kp = 40.0, Ki = 1.0, Kd = 10000.0;
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
int lastError = 0;

// ===== Controller Mode Config =====
const int rightMin = 550, rightMax = 2600, rightCenter = 1600;
const int leftMin  = 650, leftMax  = 3000, leftCenter  = 1750;
const int rtMin = 1400, rtMax = 2300;
const int deadzone = 30;
int maxSpeed = 255;
Servo myServo;
int servoPos = 90; // Start at mid position

// ===== ESP-NOW =====
typedef struct {
  int right;
  int rt;
  int left;
  uint8_t button;
} ControlData;

volatile ControlData incomingData = {0, 0, 0, 0};
volatile unsigned long lastDataReceived = 0;

// ===== Mode Toggle =====
enum RobotMode { MODE_CONTROLLER, MODE_LINEFOLLOWER };
RobotMode currentMode = MODE_CONTROLLER;
uint8_t lastButtonState = 0;
bool buttonPressed = false;

// ===== Helper Functions =====
void setMotor(int enPin, int in1, int in2, int pwm, int lim) {
  pwm = constrain(pwm, -lim, lim);
  if (pwm > 0) {
    analogWrite(enPin, pwm);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwm < 0) {
    analogWrite(enPin, -pwm);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    analogWrite(enPin, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// For controller mode (full speed, tank drive)
void tankDrive(int fwdBack, int turn) {
  int leftMotor = fwdBack + turn;
  int rightMotor = fwdBack - turn;
  leftMotor = constrain(leftMotor, -maxSpeed, maxSpeed);
  rightMotor = constrain(rightMotor, -maxSpeed, maxSpeed);
  setMotor(ENA, IN_2, IN_1, leftMotor, maxSpeed);   // Left motor
  setMotor(ENB, IN_4, IN_3, rightMotor, maxSpeed);  // Right motor
}

void stopMotors() {
  setMotor(ENA, IN_2, IN_1, 0, maxSpeed);
  setMotor(ENB, IN_4, IN_3, 0, maxSpeed);
}

// For line follower mode (limited speed, PID)
void MoveMotors(int left, int right) {
  setMotor(ENA, IN_2, IN_1, left, MaxSpeed);
  setMotor(ENB, IN_4, IN_3, right, MaxSpeed);
}

void StopMotorsLF() {
  setMotor(ENA, IN_2, IN_1, 0, MaxSpeed);
  setMotor(ENB, IN_4, IN_3, 0, MaxSpeed);
}

int mapAnalog(int val, int minV, int maxV, int centerV, int maxOut) {
  if (val > centerV + deadzone) {
    return map(val, centerV + deadzone, maxV, 0, maxOut);
  } else if (val < centerV - deadzone) {
    return map(val, centerV - deadzone, minV, 0, -maxOut);
  } else {
    return 0;
  }
}

void updateServoFromAnalog(int analogVal) {
  int speed = mapAnalog(analogVal, leftMin, leftMax, leftCenter, 4); // max 4 deg per update
  if (speed != 0) {
    servoPos += speed;
    servoPos = constrain(servoPos, 0, 180);
    myServo.write(servoPos);
  }
}

// ===== Line Follower Logic (classic PID, no sharp turn block) =====
void doLineFollow() {
  int sum = 0;
  int count = 0;

  // Read sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
    if (sensorValues[i]) {
      sum += sensorWeights[i];
      count++;
    }
  }

  int error;

  // All sensors see the line (all black): force left turn
  if (count == NUM_SENSORS) {
    MoveMotors(-120, 120); // Turn left in place
    Serial.println("All black: forced left turn");
    delay(50);
    return;
  }

  // Some sensors see the line
  if (count > 0) {
    error = sum / count;
    lastError = error;
  }
  // No sensor sees the line: use last known error direction
  else {
    error = lastError;
    if (error < 0) {
      MoveMotors(-120, 120); // Last seen line was left, turn left to search
      Serial.println("Lost line: searching left");
    } else if (error > 0) {
      MoveMotors(120, -120); // Last seen line was right, turn right to search
      Serial.println("Lost line: searching right");
    } else {
      MoveMotors(120, -120); // Unknown, default to right
      Serial.println("Lost line: default searching right");
    }
    delay(50);
    return;
  }

  // PID control
  input = error;
  setpoint = 0;
  myPID.Compute();
  int correction = (int)output;

  int baseSpeed = MaxSpeed;
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, 0, MaxSpeed);
  rightSpeed = constrain(rightSpeed, 0, MaxSpeed);

  MoveMotors(leftSpeed, rightSpeed);

  // Debug output
  Serial.print("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) Serial.print(sensorValues[i]);
  Serial.printf(" | Error: %d | PID output: %d | L:%d R:%d\n", error, correction, leftSpeed, rightSpeed);

  delay(10);
}

// ===== ESP-NOW Receive Callback =====
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(ControlData)) {
    memcpy((void*)&incomingData, data, sizeof(ControlData));
    lastDataReceived = millis();
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN_1, OUTPUT); pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT); pinMode(IN_4, OUTPUT);
  stopMotors();

  // Sensor input pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // PID setup for line follower
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-MaxSpeed, MaxSpeed);
  myPID.SetSampleTime(10);

  myServo.attach(SERVO_PIN);
  myServo.write(servoPos);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("ESP-NOW initialized");
  } else {
    Serial.println("ESP-NOW init failed!");
  }
}

// ===== LOOP =====
void loop() {
  // Mode toggle: button (debounced)
  if (incomingData.button && !lastButtonState) {
    buttonPressed = true;
  } else if (!incomingData.button && lastButtonState && buttonPressed) {
    // Toggle mode
    if (currentMode == MODE_CONTROLLER) {
      currentMode = MODE_LINEFOLLOWER;
      StopMotorsLF();
      Serial.println("Switched to LINE FOLLOWER mode");
    } else {
      currentMode = MODE_CONTROLLER;
      stopMotors();
      Serial.println("Switched to CONTROLLER mode");
    }
    buttonPressed = false;
    delay(300); // debounce
  }
  lastButtonState = incomingData.button;

  if (currentMode == MODE_CONTROLLER) {
    bool rtHeld = (incomingData.rt >= rtMax - (rtMax - rtMin) * 10 / 100);
    if (rtHeld) {
      stopMotors();
      updateServoFromAnalog(incomingData.left);
    } else {
      int fwdBack = mapAnalog(incomingData.left, leftMin, leftMax, leftCenter, maxSpeed);
      int turn    = mapAnalog(incomingData.right, rightMin, rightMax, rightCenter, maxSpeed);
      tankDrive(fwdBack, turn);
    }
  } else if (currentMode == MODE_LINEFOLLOWER) {
    doLineFollow();
  }
  delay(10);
}