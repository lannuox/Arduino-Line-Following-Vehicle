#include <LiquidCrystal.h>
#include <Wire.h>
#include <MPU6050_light.h>  // ElectronicCats 轻量库

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
MPU6050 mpu(Wire);        // 注意：ElectronicCats 库可直接这样初始化

// IR 传感器
#define IR_LEFT   0
#define IR_MID    A3
#define IR_RIGHT  A0

// 编码器
#define ENC_LEFT  A1
#define ENC_RIGHT A2

// 电机引脚
#define L_PWM   11
#define R_PWM   3
#define L_IN1   1
#define L_IN2   2
#define R_IN1   12
#define R_IN2   13

// 车辆参数
const int baseSpeed = 120;
const float wheelCirc = 23.6;
const float encoderPPR = 40.0;

// 全局变量
volatile long leftCount = 0;
volatile long rightCount = 0;

unsigned long startTime = 0;
unsigned long allWhiteTimer = 0;
unsigned long allBlackTimer = 0;

float filteredPitch = 0;
float distance = 0;
bool stoppedForever = false;

// 坡道相关
bool uphillDetected = false;
bool slopeActionDone = false;
bool distPause = false;

// ======================= 函数声明 =======================
void readIRSensors(bool &L, bool &M, bool &R);
void motorStopHard();
void setMotor(int Lspd, int Rspd);
int speedTrim();
void updateLCD();
float updateDist();
float getSlopeAngle();

// ======================= 编码器中断 =======================
volatile bool lastA1 = HIGH;
volatile bool lastA2 = HIGH;
ISR(PCINT1_vect) {
    bool currA1 = digitalRead(A1);
    bool currA2 = digitalRead(A2);

    if (lastA1 == LOW && currA1 == HIGH) leftCount++;
    if (lastA2 == LOW && currA2 == HIGH) rightCount++;

    lastA1 = currA1;
    lastA2 = currA2;
}

// ======================= 电机控制 =======================
void setMotor(int Lspd, int Rspd) {
    if (Lspd >= 0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); }
    else { digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); Lspd = -Lspd; }

    if (Rspd >= 0) { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW); }
    else { digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH); Rspd = -Rspd; }

    analogWrite(L_PWM, constrain(Lspd, 0, 255));
    analogWrite(R_PWM, constrain(Rspd, 0, 255));
}

void motorStopHard() {
    setMotor(0, 0);
    stoppedForever = true;
}

// ======================= IRSensor =======================
void readIRSensors(bool &L, bool &M, bool &R) {
    L = (digitalRead(IR_LEFT) == LOW);
    M = (digitalRead(IR_MID) == HIGH);
    R = (digitalRead(IR_RIGHT) == LOW);
}

// ======================= 计算距离 =======================
float updateDist() {
    float rev = ((leftCount + rightCount) / 2.0) / encoderPPR;
    distance = rev * wheelCirc;
    return distance;
}

// ======================= 获取坡道角度 =======================
// 使用 ElectronicCats 库计算 Pitch
float getSlopeAngle() {
    mpu.update();

    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();

    // X 轴朝上的 Pitch
    float rawPitch = atan2(-az, ay) * 57.3;

    // 一阶低通滤波（LPF）
    float alpha = 0.15;  // 越小越稳
    filteredPitch = filteredPitch * (1 - alpha) + rawPitch * alpha;

    return filteredPitch;
}

// ======================= LCD 更新 =======================
void updateLCD() {
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print((millis() - startTime)/1000);
    lcd.print("s  ");
    lcd.print("Angle:");
    lcd.print(getSlopeAngle());

    lcd.setCursor(0,1);
    lcd.print("Distance:");
    lcd.print(distance);
    lcd.print("cm");
}

int speedTrim() {
    long diff = rightCount - leftCount;
    return constrain(diff * 2, -60, 60);
}

// ======================= SETUP =======================
void setup() {
    lcd.begin(16,2);
    Wire.begin();

    mpu.begin();
    mpu.setAccOffsets(-ax_offset, -ay_offset, -az_offset);  // 稍后我教你如何获取
    delay(500);
    mpu.calcGyroOffsets();  // 自动校准陀螺仪

    pinMode(IR_LEFT, INPUT_PULLUP);
    pinMode(IR_MID, INPUT_PULLUP);
    pinMode(IR_RIGHT, INPUT_PULLUP);
    pinMode(ENC_LEFT, INPUT_PULLUP);
    pinMode(ENC_RIGHT, INPUT_PULLUP);
    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    // 启用 PCINT1
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);

    startTime = millis();
    lcd.print("TOUCH MORE GRASS");
    delay(1000);
    lcd.clear();
}

// ======================= LOOP =======================
void loop() {
    bool L, M, R;
    readIRSensors(L,M,R);
    updateDist();
    float angleZ = getSlopeAngle();

    if (stoppedForever) { motorStopHard(); return; }

    // 坡顶动作（阻塞式）
  if (uphillDetected && !slopeActionDone && abs(angleZ) <= 10) {
    delay(200);
    setMotor(0, 0);        // 停车
    updateLCD();
    delay(4000);            // 停车4秒

    setMotor(255, -255);    // 旋转360°
    updateLCD();
    delay(1700);            // turn time

    setMotor(80, 80);
    delay(4000);         // 完成旋转后停止
    updateLCD();

    slopeActionDone = true; // 标记动作完成
  }

    // 330cm 停车
    if (!distPause && distance>=330.0) { 
      distPause=true; 
      setMotor(0,0); 
      delay(1000); 
      updateLCD(); 
      delay(1000); 
      updateLCD(); 
      return; 
    }

    // 坡道变速
    if (angleZ > 10 && !slopeActionDone) { uphillDetected=true; setMotor(255,210); updateLCD(); return; } // 上坡

    // 全黑永久停
    if (L && M && R) { if (allBlackTimer==0) allBlackTimer=millis(); if (millis()-allBlackTimer>100){ motorStopHard(); return; } }
    else allBlackTimer=0;

    // 全白原地旋转
    if (!L && !M && !R) { if (allWhiteTimer==0) allWhiteTimer=millis(); if (millis()-allWhiteTimer>200){ setMotor(255,-255); updateLCD(); return; } }
    else allWhiteTimer=0;

    // 线路跟随
    int trim = speedTrim();
    int Lspd = baseSpeed - trim;
    int Rspd = baseSpeed + trim;

    if (M) setMotor(Lspd,Rspd);
    else if ((M && L) || (L && !M && !R)) setMotor(Lspd-100,Rspd+100);
    else if ((M && R) || (!L && !M && R)) setMotor(Lspd+100,Rspd-100);

    updateLCD();
    delay(1);
}
