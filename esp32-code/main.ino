// Oil Skimmer Boat - ESP32 + Bluetooth Classic
// Libs: ESP32Servo (Library Manager). BluetoothSerial ships with the ESP32 core.
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#define IN1 13
#define IN2 26
#define IN3 14
#define IN4 27
#define MOTOR_PIN 16
#define SERVO_L 18
#define SERVO_R 19

// 1500 us (~9.7 RPM) max safe speed for 28BYJ-48 without missing steps under load
#define STEP_INTERVAL_US 1500

// Updated range & center point for 360-degree positional servos
#define RUDDER_MIN 0
#define RUDDER_MAX 360
#define RUDDER_NEUTRAL 180
#define FAILSAFE_STOP true   // stop prop motor if phone disconnects

#if ESP_ARDUINO_VERSION_MAJOR < 3
  #define MOTOR_CH 15
#endif

BluetoothSerial SerialBT;
Servo servoL, servoR;

const uint8_t stepTable[8][4] = {
  {1,0,0,0},{1,1,0,0},{0,1,0,0},{0,1,1,0},
  {0,0,1,0},{0,0,1,1},{0,0,0,1},{1,0,0,1}
};
const uint8_t stepPins[4] = {IN1, IN2, IN3, IN4};

bool stepperRunning = true;          // auto-start on power-up
uint8_t stepIndex = 0;
uint32_t lastStepUs = 0;

char rxBuf[16];
uint8_t rxLen = 0;
bool wasConnected = false;

void motorWrite(int duty) {
  duty = constrain(duty, 0, 255);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PIN, duty);
#else
  ledcWrite(MOTOR_CH, duty);
#endif
}

void coilsOff() { for (uint8_t i = 0; i < 4; i++) digitalWrite(stepPins[i], LOW); }

void setRudder(int a) {
  a = constrain(a, RUDDER_MIN, RUDDER_MAX);
  servoL.write(a);
  servoR.write(a);
}

void handleCommand(const char* s) {
  int v = atoi(s + 1);
  switch (s[0]) {
    case 'M': motorWrite(v); break;
    case 'R': setRudder(v); break;
    case 'S':
      stepperRunning = (v != 0);
      if (!stepperRunning) coilsOff();
      break;
  }
}

void setup() {
  // ---- Fast hardware init (< 5ms before BT stack) ----
  for (uint8_t i = 0; i < 4; i++) { 
    pinMode(stepPins[i], OUTPUT); 
    digitalWrite(stepPins[i], LOW); 
  }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(MOTOR_PIN, 5000, 8);
#else
  ledcSetup(MOTOR_CH, 5000, 8);
  ledcAttachPin(MOTOR_PIN, MOTOR_CH);
#endif
  motorWrite(0);

  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoL.setPeriodHertz(50);
  servoR.setPeriodHertz(50);
  
  // Extended pulse width range (500us to 2500us) for full 360° motion
  servoL.attach(SERVO_L, 500, 2500);
  servoR.attach(SERVO_R, 500, 2500);
  setRudder(RUDDER_NEUTRAL);

  stepperRunning = true;
  lastStepUs = micros();

  // ---- Bluetooth stack initialization ----
  SerialBT.begin("Skimmer-Boat");
}

void loop() {
  // Non-blocking stepper loop at 1500 us per step
  uint32_t now = micros();
  if (stepperRunning && (uint32_t)(now - lastStepUs) >= STEP_INTERVAL_US) {
    lastStepUs += STEP_INTERVAL_US;
    for (uint8_t i = 0; i < 4; i++) digitalWrite(stepPins[i], stepTable[stepIndex][i]);
    stepIndex = (stepIndex + 1) & 7;
  }

  // Non-blocking Bluetooth command parsing
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (rxLen > 0) { 
        rxBuf[rxLen] = 0; 
        handleCommand(rxBuf); 
      }
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0; // Buffer overflow protection
    }
  }

  // Connection loss failsafe
  if (FAILSAFE_STOP) {
    bool conn = SerialBT.hasClient();
    if (wasConnected && !conn) motorWrite(0);
    wasConnected = conn;
  }
}
