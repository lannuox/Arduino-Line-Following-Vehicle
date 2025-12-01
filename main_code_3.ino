// ======================
//  Integrated: 2-sensor line-following + encoder distance
//  + MPU6050 pitch detection -> on slope event stop 4s + rotate 360°
//  LCD realtime: time, distance, angle, left/right speed
//  Author: integrated for Lanno (based on user's version), ChatGPT
// ======================

#include <Wire.h>
#include <LiquidCrystal.h>

// LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ------------------- Pins (from user's version) -------------------
#define IR_LEFT   A3
#define IR_RIGHT  0    // 注意：你把右传感器设为 0 (A0/digital 0)，若同时用 Serial 建议换掉
#define MPU_SDA A1
#define MPU_SCL A2
#define ENC_LEFT  A5   // PCINT21
#define ENC_RIGHT A4   // PCINT20

#define L_PWM   11
#define R_PWM   3
#define L_IN1   1      // 注意：pin1 = Serial TX，容易冲突，建议改为其它 IO（强烈建议）
#define L_IN2   2
#define R_IN1   12
#define R_IN2   13

// ------------------- Vehicle parameters (configurable) -------------------
const float wheelCirc = 47.1;   // cm, 你的代码里是 47.1
const float encoderPPR = 40.0;  // pulses per wheel revolution
const float TRACK_WIDTH_CM = 13.2; // 两轮中心间距 (cm) —— **请按实测值修改**

// line-following base
const int BASE_SPEED = 150;
const int TURN_BOOST = 100;

// slope detection thresholds (degrees)
const float CLIMB_THRESHOLD = 10.0;       // 开始上坡判定
const float PEAK_ANGLE_THRESHOLD = 25.0;  // 峰值判定，确认是显著坡度
const float NEAR_ZERO_THRESHOLD = 5.0;    // 回平地判定（小于该角度视为回平地）
const unsigned long STOP_4S_MS = 4000;    // 停 4 秒

// ------------------- Globals -------------------
volatile long leftCount = 0;
volatile long rightCount = 0;

volatile bool lastA4 = HIGH;
volatile bool lastA5 = HIGH;

unsigned long startMillis = 0;
float distance_cm = 0.0;

// pid-ish speed trim
int currentLspd = 0;
int currentRspd = 0;

// slope event detection states
bool climbing = false;
bool peaked = false;
bool slopeEventTriggered = false;

// imu / complementary filter
float pitch = 0.0;      // degrees
float gyroXrate = 0.0;  // deg/sec
unsigned long lastIMURead = 0;

// rotation control
bool isRotating360 = false;

// ------------------- Function declarations -------------------
void setMotor(int Lspd, int Rspd);
void motorStopHard();
float getDistanceCm();
int speedTrimPID();
void doRotate360_byEncoder();
void resetEncoderCounts();
long readLeftCount();
long readRightCount();

// ------------------- Encoder PCINT ISR -------------------
// Note: correct mapping: A5 -> leftCount, A4 -> rightCount
ISR(PCINT1_vect) {
  bool currA4 = digitalRead(A4);
  bool currA5 = digitalRead(A5);

  // rising edge detection: A5 -> left, A4 -> right
  if (lastA5 == LOW && currA5 == HIGH) leftCount++;
  if (lastA4 == LOW && currA4 == HIGH) rightCount++;

  lastA4 = currA4;
  lastA5 = currA5;
}

// ------------------- Motor control -------------------
void setMotor(int Lspd, int Rspd) {
  // Store for LCD
  currentLspd = Lspd;
  currentRspd = Rspd;

  // 左轮
  if (Lspd >= 0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); }
  else { digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); Lspd = -Lspd; }

  // 右轮
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

// ------------------- Distance measurement -------------------
float getDistanceCm() {
  // average of both encoders
  long Lc, Rc;
  noInterrupts();
  Lc = leftCount;
  Rc = rightCount;
  interrupts();

  float rev = (Lc + Rc) / 2.0 / encoderPPR;
  distance_cm = rev * wheelCirc;
  return distance_cm;
}

// ------------------- Simple speed trim using encoder diff -------------------
int speedTrimPID() {
  long Lc, Rc;
  noInterrupts();
  Lc = leftCount;
  Rc = rightCount;
  interrupts();
  long diff = Rc - Lc;
  return constrain(diff * 2, -60, 60);
}

// ------------------- Reset encoders atomically -------------------
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

// ------------------- Compute pulses needed for 360° rotation -------------------
long pulsesFor360() {
  // one wheel needs to travel: pi * track_width (cm)
  float rotationDistanceCm = PI * TRACK_WIDTH_CM;
  float revNeeded = rotationDistanceCm / wheelCirc; // wheel revolutions
  float pulses = revNeeded * encoderPPR;
  return (long)round(pulses);
}

// ------------------- Perform rotation in place using encoder counts -------------------
void doRotate360_byEncoder() {
  long targetPulses = pulsesFor360();
  if (targetPulses <= 0) return;

  // reset counts then rotate until average abs count reaches target
  resetEncoderCounts();

  // choose rotation speed (tune if too slow/fast)
  int rotSpeed = 200; // pwm for rotation

  // set motors opposite directions for in-place rotation
  setMotor(rotSpeed, -rotSpeed);

  while (true) {
    delay(5); // small delay to avoid busy-wait hogging

    long Lc = readLeftCount();
    long Rc = readRightCount();
    long avgAbs = (labs(Lc) + labs(Rc)) / 2;
    // Serial debug
    Serial.print("Rot pulse avg: "); Serial.println(avgAbs);

    if (avgAbs >= targetPulses) break;
  }

  motorStopHard();
  delay(100); // a tiny settle time
}

// ------------------- MPU6050 (Complementary Filter) -------------------
// We'll read raw accel/gyro registers and compute pitch (deg).
void imuInit() {
  Wire.begin();
  // wake up MPU6050 (address 0x68)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // wake up
  Wire.endTransmission();
  delay(50);
}

void imuReadAndComputePitch() {
  const float RAD2DEG = 57.29577951308232;
  const float alpha = 0.98; // complementary filter

  unsigned long now = micros();
  static unsigned long lastMicros = 0;
  if (lastMicros == 0) { lastMicros = now; }

  float dt = (now - lastMicros) / 1000000.0; // seconds
  if (dt <= 0) dt = 0.001;

  // read accel x,y,z and gyro x
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  if (Wire.available() >= 14) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    // units:
    // accel: raw -> g (16384 LSB/g for default range)
    // gyro: raw -> deg/s (131 LSB/(deg/s) for default ±250deg/s)
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;
    float gx_dps = gx / 131.0; // deg per sec

    // accel-based pitch (atan2)
    float accelPitch = atan2(ay_g, az_g) * RAD2DEG;

    // integrate gyro rate (assuming gx is rotation around X -> pitch rate)
    gyroXrate = gx_dps;
    float gyroPitchDelta = gx_dps * dt;

    // complementary filter
    pitch = alpha * (pitch + gyroPitchDelta) + (1 - alpha) * accelPitch;

    lastMicros = now;
  }
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);

  // Pins
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

  // enable PCINT1 suite and mask A4, A5
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT20) | (1 << PCINT21);

  // init MPU
  imuInit();

  resetEncoderCounts();
  startMillis = millis();

  lcd.print("Car Ready!");
  delay(1000);
}

// ------------------- Loop -------------------
void loop() {
  // IMU update (fast)
  imuReadAndComputePitch();

  // get distance
  float dist = getDistanceCm();

  // sensors
  bool L = (digitalRead(IR_LEFT)  == LOW);
  bool R = (digitalRead(IR_RIGHT) == LOW);

  // display time in seconds with 1 decimal
  float t = (millis() - startMillis) / 1000.0;

  // ---------------- slope detection logic ----------------
  // climbing start
  if (!climbing && pitch > CLIMB_THRESHOLD) {
    climbing = true;
    peaked = false;
    Serial.println("Climb detected");
  }
  // detect peak
  if (climbing && pitch >= PEAK_ANGLE_THRESHOLD) {
    peaked = true;
  }
  // detect return to near-zero after peak -> event
  if (climbing && peaked && pitch <= NEAR_ZERO_THRESHOLD) {
    // trigger slope event
    climbing = false;
    peaked = false;
    slopeEventTriggered = true;
    Serial.println("Slope event triggered!");
  }

  // ---------------- action when slopeEventTriggered ----------------
  if (slopeEventTriggered) {
    slopeEventTriggered = false;

    // 1) stop, display for 4s (but keep updating LCD angle/time)
    unsigned long t0 = millis();
    while (millis() - t0 < STOP_4S_MS) {
      motorStopHard();
      // update LCD while waiting
      lcd.setCursor(0,0);
      lcd.print("T:");
      lcd.print(t, 1);
      lcd.print("s D:");
      lcd.print(dist, 1);
      lcd.print("cm ");
      lcd.setCursor(0,1);
      lcd.print("A:");
      lcd.print(pitch, 1);
      lcd.print((char)223); lcd.print(" ");
      lcd.print("L:");
      lcd.print(currentLspd);
      lcd.print(" R:");
      lcd.print(currentRspd);
      delay(100);
    }

    // 2) rotate 360 deg using encoders
    Serial.println("Rotate 360 start");
    doRotate360_byEncoder();
    Serial.println("Rotate 360 done");

    // 3) continue (no other special state required)
  }

  // ---------------- reach 330 cm stop2s (original functionality) ----------------
  static bool stopped2s = false;
  static unsigned long stop2s_ts = 0;

  if (!stopped2s && dist >= 330.0) {
    // stop for 2s
    motorStopHard();
    stopped2s = true;
    stop2s_ts = millis();
  }

  if (stopped2s) {
    // while in stop2s, update LCD; after 2s continue
    if (millis() - stop2s_ts >= 2000) {
      stopped2s = false;
    } else {
      // display while waiting
      lcd.setCursor(0,0);
      lcd.print("T"); lcd.print(t,1); lcd.print("s D"); lcd.print(dist,1); lcd.print("cm");
      lcd.setCursor(0,1);
      lcd.print("A"); lcd.print(pitch,1); lcd.print(" L"); lcd.print(currentLspd); lcd.print(" R"); lcd.print(currentRspd);
      delay(50);
      return; // don't run line following during stop
    }
  }

  // ---------------- line following (2-sensor) ----------------
  int trim = speedTrimPID();
  int Lspd = BASE_SPEED - trim;
  int Rspd = BASE_SPEED + trim;

  if (L && !R) {
    // line on left sensor -> adjust leftwards
    setMotor(Lspd - TURN_BOOST, Rspd + TURN_BOOST);
  } else if (!L && R) {
    // line on right sensor -> adjust rightwards
    setMotor(Lspd + TURN_BOOST, Rspd - TURN_BOOST);
  } else {
    // both white or both black -> go forward (if both black you may want special)
    setMotor(Lspd, Rspd);
  }

  // ---------------- LCD update ----------------
  lcd.setCursor(0,0);
  lcd.print("T"); lcd.print(t,1); lcd.print("s D"); lcd.print(dist,1); lcd.print("cm");
  lcd.setCursor(0,1);
  lcd.print("A"); lcd.print(pitch,1); lcd.print(" L"); lcd.print(currentLspd); lcd.print(" R"); lcd.print(currentRspd);
  delay(20);
}
