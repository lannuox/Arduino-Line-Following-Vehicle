#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
MPU6050 mpu;

// Motor pins
const int IN1 = 1, IN2 = 2, IN3 = 12, IN4 = 13;
const int ENA = 11; // 左轮
const int ENB = 3;  // 右轮

unsigned long startMillis;
int secondsElapsed = 0;

const int maxLeft = 100;
const int maxRight = 100;
float targetYaw = 0;
float Kp = 0.2; // 调比例，让微小角度也调速
float yaw = 0;
float yawOffset = 0;

void setup() {
  lcd.begin(16, 2);
  lcd.print("Initializing...");

  Wire.begin();
  mpu.initialize();
  delay(2000);

  // ===== 开机零偏校正 =====
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(10);
  }
  yawOffset = sum / 100.0;  // 平均静态偏置

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  moveStraight();
  startMillis = millis();
}

void moveStraight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void loop() {
  secondsElapsed = (millis() - startMillis) / 1000;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // ===== 修正零偏 + 小滤波 =====
  float gz_corrected = gz - yawOffset;
  static float yawFiltered = 0;
  yawFiltered = 0.95 * yawFiltered + 0.05 * (gz_corrected / 131.0 * 0.01); // 简单低通
  yaw += yawFiltered;

  float error = targetYaw - yaw;

  // 修正百分比
  float correctionPercent = constrain(abs(Kp * error), 0, 1.0);

  int leftSpeed = maxLeft;
  int rightSpeed = maxRight;
  String status = "FWD";

  if (error > 0) {
    leftSpeed = maxLeft * (1.0 - correctionPercent);
    rightSpeed = maxRight;
    status = "<---L";
  } else if (error < 0) {
    leftSpeed = maxLeft;
    rightSpeed = maxRight * (1.0 - correctionPercent);
    status = "R--->";
  }

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // ==== LCD 输出 ====
  lcd.setCursor(0, 0);
  lcd.print("Time:");
  lcd.print(secondsElapsed);
  lcd.print("s ");
  lcd.print(status);

  lcd.setCursor(0, 1);
  lcd.print("Y:");
  lcd.print(yaw, 1); // 一位小数
  lcd.print(" ");
  lcd.print(leftSpeed);
  lcd.print(":");
  lcd.print(rightSpeed);

  delay(50);
  lcd.clear();

  if (secondsElapsed > 10) {
    stopMotors();
    lcd.clear();
    lcd.print("10 sec goal done");
    lcd.setCursor(0, 1);
    lcd.print("TOUCH MORE GRASS");
    while (1);
  }
}
