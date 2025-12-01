#include <Wire.h>
#include <LiquidCrystal.h>

// LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ------------------- Pins -------------------
#define IR_LEFT   A3
#define IR_RIGHT  0
#define MPU_SDA A1
#define MPU_SCL A2
#define ENC_LEFT  A5
#define ENC_RIGHT A4

#define L_PWM   11
#define R_PWM   3
#define L_IN1   1
#define L_IN2   2
#define R_IN1   12
#define R_IN2   13

// ------------------- Vehicle parameters -------------------
const float wheelCirc = 47.1;
const float encoderPPR = 40.0;
const float TRACK_WIDTH_CM = 13.2;

const int BASE_SPEED = 150;
const int TURN_BOOST = 100;

const float CLIMB_THRESHOLD = 10.0;
const float PEAK_ANGLE_THRESHOLD = 25.0;
const float NEAR_ZERO_THRESHOLD = 5.0;
const unsigned long STOP_4S_MS = 4000;

// ------------------- Globals -------------------
volatile long leftCount = 0;
volatile long rightCount = 0;

volatile bool lastA4 = HIGH;
volatile bool lastA5 = HIGH;

unsigned long startMillis = 0;
float distance_cm = 0.0;

int currentLspd = 0;
int currentRspd = 0;

bool climbing = false;
bool peaked = false;
bool slopeEventTriggered = false;

float pitch = 0.0;
float gyroXrate = 0.0;

// ------------------- Function declarations -------------------
void setMotor(int Lspd, int Rspd);
void motorStopHard();
float getDistanceCm();
int speedTrimPID();
void doRotate360_byEncoder();
void resetEncoderCounts();
long readLeftCount();
long readRightCount();

// ------------------- Encoder ISR -------------------
ISR(PCINT1_vect) {
  bool currA4 = digitalRead(A4);
  bool currA5 = digitalRead(A5);

  if (lastA5 == LOW && currA5 == HIGH) leftCount++;
  if (lastA4 == LOW && currA4 == HIGH) rightCount++;

  lastA4 = currA4;
  lastA5 = currA5;
}

// ------------------- Motors -------------------
void setMotor(int Lspd, int Rspd) {
  currentLspd = Lspd;
  currentRspd = Rspd;

  if (Lspd >= 0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); }
  else { digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); Lspd = -Lspd; }

  if (Rspd >= 0) { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW); }
  else { digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH); Rspd = -Rspd; }

  analogWrite(L_PWM, constrain(Lspd, 0, 255));
  analogWrite(R_PWM, constrain(Rspd, 0, 255));
}

void motorStopHard() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  currentLspd = currentRspd = 0;
}

// ------------------- Distance -------------------
float getDistanceCm() {
  long Lc, Rc;
  noInterrupts();
  Lc = leftCount;
  Rc = rightCount;
  interrupts();

  float rev = (Lc + Rc) / 2.0 / encoderPPR;
  distance_cm = rev * wheelCirc;
  return distance_cm;
}

int speedTrimPID() {
  long Lc, Rc;
  noInterrupts();
  Lc = leftCount;
  Rc = rightCount;
  interrupts();

  long diff = Rc - Lc;
  return constrain(diff * 2, -60, 60);
}

void resetEncoderCounts() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
}

long readLeftCount() {
  long v;
  noInterrupts();
  v = leftCount;
  interrupts();
  return v;
}

long readRightCount() {
  long v;
  noInterrupts();
  v = rightCount;
  interrupts();
  return v;
}

// ------------------- Rotation -------------------
long pulsesFor360() {
  float rotationDistanceCm = PI * TRACK_WIDTH_CM;
  float revNeeded = rotationDistanceCm / wheelCirc;
  float pulses = revNeeded * encoderPPR;
  return (long)round(pulses);
}

void doRotate360_byEncoder() {
  long targetPulses = pulsesFor360();
  resetEncoderCounts();
  int rotSpeed = 200;

  setMotor(rotSpeed, -rotSpeed);

  while (true) {
    delay(5);
    long Lc = readLeftCount();
    long Rc = readRightCount();
    long avgAbs = (labs(Lc) + labs(Rc)) / 2;

    if (avgAbs >= targetPulses) break;
  }

  motorStopHard();
  delay(100);
}

// ------------------- MPU -------------------
void imuInit() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
}

void imuReadAndComputePitch() {
  const float RAD2DEG = 57.29578;
  const float alpha = 0.98;

  static unsigned long lastMicros = micros();
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  if (Wire.available() >= 14) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    float accelPitch = atan2(ay_g, az_g) * RAD2DEG;
    float gyroPitchDelta = (gx / 131.0) * dt;

    pitch = alpha * (pitch + gyroPitchDelta) + (1 - alpha) * accelPitch;
  }
}

// ------------------- Setup -------------------
void setup() {
  lcd.begin(16, 2);

  pinMode(IR_LEFT, INPUT_PULLUP);
  pinMode(IR_RIGHT, INPUT_PULLUP);
  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);

  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT20) | (1 << PCINT21);

  imuInit();
  resetEncoderCounts();
  startMillis = millis();

  lcd.print("Car Ready!");
  delay(1000);
}

// ------------------- Loop -------------------
void loop() {
  imuReadAndComputePitch();
  float dist = getDistanceCm();

  bool L = (digitalRead(IR_LEFT) == LOW);
  bool R = (digitalRead(IR_RIGHT) == LOW);

  float t = (millis() - startMillis) / 1000.0;

  // ---------------- Slope Detection ----------------
  if (!climbing && pitch > CLIMB_THRESHOLD) climbing = true;
  if (climbing && pitch >= PEAK_ANGLE_THRESHOLD) peaked = true;
  if (climbing && peaked && pitch <= NEAR_ZERO_THRESHOLD) {
    climbing = false;
    peaked = false;
    slopeEventTriggered = true;
  }

  if (slopeEventTriggered) {
    slopeEventTriggered = false;

    unsigned long t0 = millis();
    while (millis() - t0 < STOP_4S_MS) {
      motorStopHard();
      delay(100);
    }

    doRotate360_byEncoder();
  }

  // ---------------- 330cm → Stop 2 seconds ----------------
  static bool stopped2s = false;
  static unsigned long stop2s_ts = 0;

  if (!stopped2s && dist >= 330.0) {
    motorStopHard();
    stopped2s = true;
    stop2s_ts = millis();
  }

  if (stopped2s) {
    if (millis() - stop2s_ts >= 2000) {
      stopped2s = false;
    } else {
      motorStopHard();
      return;  // 不执行循迹
    }
  }

  // ---------------- Line Following ----------------
  int trim = speedTrimPID();
  int Lspd = BASE_SPEED - trim;
  int Rspd = BASE_SPEED + trim;

  if (L && !R) {
    setMotor(Lspd - TURN_BOOST, Rspd + TURN_BOOST);
  }
  else if (!L && R) {
    setMotor(Lspd + TURN_BOOST, Rspd - TURN_BOOST);
  }
  else {
    setMotor(Lspd, Rspd);
  }

  // ---------------- LCD ----------------
  lcd.setCursor(0,0);
  lcd.print("T"); lcd.print(t,1);
  lcd.print(" D"); lcd.print(dist,1);

  lcd.setCursor(0,1);
  lcd.print("A"); lcd.print(pitch,1);
  lcd.print(" L"); lcd.print(currentLspd);
  lcd.print(" R"); lcd.print(currentRspd);
}
