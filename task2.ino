#include <LiquidCrystal.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ===================== LCD & MPU =====================
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
MPU6050 mpu(Wire);

// ===================== Motor Pins =====================
#define L_PWM   11
#define R_PWM   3
#define L_IN1   A1
#define L_IN2   2
#define R_IN1   12
#define R_IN2   13

// ===================== Slope Detection =====================
bool uphillTriggered = false;
unsigned long angleStartTime = 0;   // 记录超过阈值的起点时间
float angleFiltered = 0;
float angleMax = 0;

const float ANGLE_THRESHOLD = 6.0;      // 判定为坡道的角度
const unsigned long HOLD_TIME = 1000;   // 超过阈值需持续 1s 才触发

// ===================== Motor =====================
void setMotor(int Lspd, int Rspd) {
    if (Lspd >= 0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); }
    else { digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); Lspd = -Lspd; }

    if (Rspd >= 0) { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW); }
    else { digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH); Rspd = -Rspd; }

    analogWrite(L_PWM, constrain(Lspd, 0, 255));
    analogWrite(R_PWM, constrain(Rspd, 0, 255));
}

// ===================== Angle =====================
float readAngleX() {
    mpu.update();
    return mpu.getAngleX();
}

// ===================== LCD =====================
void updateLCD() {
    lcd.setCursor(0, 0);
    lcd.print("Angle:");
    lcd.print(angleFiltered);

    lcd.setCursor(0, 1);
    lcd.print("Max:");
    lcd.print(angleMax);
}

// ===================== SETUP =====================
void setup() {
    lcd.begin(16, 2);

    Wire.begin();
    mpu.begin();
    mpu.setFilterGyroCoef(0.95);   // 稳一点
    mpu.calcOffsets();             // 一次性归零，后续不漂
    delay(500);

    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    lcd.print("TOUCH MORE GRASS");
    delay(1000);
    lcd.clear();

    setMotor(160, 130);   // 正常巡航
}

// ===================== LOOP =====================
void loop() {
    // 读取滤波角度
    float raw = readAngleX();
    angleFiltered = 0.3 * angleFiltered + 0.7 * raw;

    // 记录最大角度
    if (angleFiltered > angleMax) angleMax = angleFiltered;

    // ===================== 1) 坡道检测逻辑 =====================
    if (!uphillTriggered) {
        if (abs(angleFiltered) > ANGLE_THRESHOLD) {
            if (angleStartTime == 0) {
                angleStartTime = millis();  // 开始计时
            }
            else if (millis() - angleStartTime >= HOLD_TIME) {
                // *** 正式触发坡道事件 ***
                uphillTriggered = true;
                setMotor(255,255);
            }
        } else {
            angleStartTime = 0;  // 角度掉回去了，取消计时
        }
    }

    // ===================== 2) 坡道动作 =====================
if (uphillTriggered) {

    if (abs(angleFiltered) <= ANGLE_THRESHOLD) {

        // -------- 停车 4 秒 --------
        delay(100);
        setMotor(0, 0);
        for (int i = 0; i < 4; i++) {
            updateLCD();
            delay(1000);
        }

        // -------- 旋转 360° --------
        setMotor(255, -255);
        delay(3900);

        // -------- 小走一下 --------
        setMotor(0, 0); delay(1000);
        setMotor(100, 100); delay(1500);

        // -------- 永久停止 --------
        setMotor(0, 0);
        while (1);
    }
}

    updateLCD();
    delay(10);
}
