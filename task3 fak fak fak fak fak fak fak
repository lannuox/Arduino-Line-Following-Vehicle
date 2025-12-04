#include <Arduino.h>
#include <Wire.h>

#define TRIG A3
#define ECHO A2


// Motor pins
#define L_PWM 11
#define R_PWM 3
#define L_IN1 A1
#define L_IN2 2
#define R_IN1 12
#define R_IN2 13

int stopDistance = 50;
int speed = 180;
void setMotor(int Lspd, int Rspd);

// ======================= 电机控制 =======================
void setMotor(int Lspd, int Rspd) {

  // 左轮方向设置
  if (Lspd >= 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    Lspd = -Lspd;
  }

  // 右轮方向设置
  if (Rspd >= 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    Rspd = -Rspd;
  }

  analogWrite(L_PWM, constrain(Lspd, 0, 255));
  analogWrite(R_PWM, constrain(Rspd, 0, 255));
}
long getDistance() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 25000); // 超时保护 25ms
    long distance = duration * 0.034 / 2;       // 声速 340m/s → 0.034 cm/us
    return distance;
}

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  delay(2000);
  setMotor(150, 150);
}


void loop() {
  long d = getDistance();
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" cm");

    if (d > 0 && d < stopDistance) {
        setMotor(255,-255);     // 🚫 发现障碍物→停
        delay(1000);
    } else {
        setMotor(150, 150); // 🚗 正常前进
    }

    delay(10);
}
