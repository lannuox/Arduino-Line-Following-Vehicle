#include <LiquidCrystal.h>
#include <Wire.h>
#include <MPU6050_light.h>  // ElectronicCats 轻量库

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
MPU6050 mpu(Wire);        // 注意：ElectronicCats 库可直接这样初始化

// 电机引脚
#define L_PWM   11
#define R_PWM   3
#define L_IN1   1
#define L_IN2   2
#define R_IN1   12
#define R_IN2   13

// 坡道相关
bool uphillDetected = false;
bool downhillDetected = false;
bool slopeActionDone = false;

// ======================= 函数声明 =======================
void setMotor(int Lspd, int Rspd);
void updateLCD();
float getSlopeAngle();
float rampAngle;
static float angleX_filtered;

// ======================= 电机控制 =======================
void setMotor(int Lspd, int Rspd) {
    if (Lspd >= 0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); }
    else { digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); Lspd = -Lspd; }

    if (Rspd >= 0) { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW); }
    else { digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH); Rspd = -Rspd; }

    analogWrite(L_PWM, constrain(Lspd, 0, 255));
    analogWrite(R_PWM, constrain(Rspd, 0, 255));
}

// ======================= 获取坡道角度 =======================
// 使用 ElectronicCats 库计算
float getSlopeAngle() {
    mpu.update();
    return mpu.getAngleX();
}
// ======================= LCD 更新 =======================
void updateLCD() {
    mpu.update();
    lcd.setCursor(0,0);
    lcd.print("Angle:");
    lcd.print(angleX_filtered);
    lcd.setCursor(0,1);
    lcd.print("Angle Max:");
    lcd.print(rampAngle);
}

// ======================= SETUP =======================
void setup() {
    lcd.begin(16,2);
    Wire.begin();
    mpu.begin();
    mpu.setFilterGyroCoef(0.90);
    mpu.calcOffsets();   // 比 calcGyroOffsets() 更全面：是加速度+陀螺仪一起校准
    delay(500);


    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    lcd.print("TOUCH MORE GRASS");
    delay(500);
    lcd.clear();
    setMotor(160, 130);
}

// ======================= LOOP =======================
void loop() {
    mpu.update();
    float angle_raw = getSlopeAngle();       // 原始角度
    angleX_filtered = 0.3 * angleX_filtered + 0.7 * angle_raw;
    float angleX = angleX_filtered;          // 用滤波后的角度

    if (rampAngle <= angleX){
        rampAngle = angleX;
    }

    if (abs(angleX) > 6 && !uphillDetected){
      uphillDetected = true;
      setMotor(190, 160);
    }

    if (uphillDetected && abs(angleX) <= 4) {     
      delay(200);
      setMotor(0, 0);        // 停车
      updateLCD();
      delay(1000);
      updateLCD();
      delay(1000);
      updateLCD();
      delay(1000);
      updateLCD();
      delay(1000);            // 停车4秒

      setMotor(255, -200);    // 旋转360°
      delay(5000);
      updateLCD();          // turn time

      setMotor(0, 0);
      delay(1000);
      updateLCD();
      setMotor(100, 100);
      delay(1000);
      updateLCD();
      setMotor(0, 0);
      delay(99999);
  }

    updateLCD();
    delay(1);
}
