#include <LiquidCrystal.h>
#include <Wire.h>
#include <MPU6050.h>

// ======================= LCD 屏幕 =======================
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
MPU6050 mpu;

// ======================= 引脚定义 =======================
// IR 传感器：左 & 右黑线→LOW，中间黑线→HIGH
#define IR_LEFT   0
#define IR_MID    A3
#define IR_RIGHT  A0

// 编码器（PCINT1）
#define ENC_LEFT  A1   // PCINT9
#define ENC_RIGHT A2   // PCINT10

// 电机引脚
#define L_PWM   11
#define R_PWM   3
#define L_IN1   1
#define L_IN2   2
#define R_IN1   12
#define R_IN2   13

// ======================= 车辆参数 =======================
const int baseSpeed = 120;      // 基本速度
const float wheelCirc = 23.6;    // 轮子周长 (cm)
const float encoderPPR = 40.0;  // 每圈脉冲数

// ======================= 全局变量 =======================
volatile long leftCount = 0;
volatile long rightCount = 0;

unsigned long startTime = 0; //时间
unsigned long allWhiteTimer = 0;
unsigned long allBlackTimer = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float mpuPitch = 0.0; //mpu角度

float distance = 0.0; //距离
bool stoppedForever = false; //永久停止

bool speedUpHill = false; //上下坡变速
bool speedDownHill = false;

bool distPause = false; //行驶距离后停止 task1
unsigned long distPauseStart = 0;

bool uphillDetected = false;     // 是否已经检测到上坡
bool slopeActionDone = false;    // 是否已经完成 360 度旋转
bool slopePause = false;         // 当前是否在 4 秒 + 转圈阶段
unsigned long slopePauseStart = 0;


// ======================= 函数声明 =======================
void readIRSensors(bool &L, bool &M, bool &R);
void motorStopHard();
void setMotor(int Lspd, int Rspd);
int speedTrim();
void updateLCD();
float updateDist();

// ======================= 强制停止（永久） =======================
void motorStopHard() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  stoppedForever = true;
}

// ======================= 编码器 PCINT 中断 =======================
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

// ======================= 软件速度补偿（软修正） =======================
int speedTrim() {
  long diff = rightCount - leftCount;
  return constrain(diff * 2, -60, 60);
}

// ======================= 读取三个 IR 传感器 =======================
void readIRSensors(bool &L, bool &M, bool &R) {
  L = (digitalRead(IR_LEFT)  == LOW);
  M = (digitalRead(IR_MID)   == HIGH);
  R = (digitalRead(IR_RIGHT) == LOW);
}

// ======================= 计算距离 =======================
float updateDist() {
  float rev = ((leftCount + rightCount) / 2.0) / encoderPPR;
  distance = rev * wheelCirc;
  return distance;
}

// ======================= MPU 更新 =======================
void updateMPU() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 使用加速度仪计算 Pitch
    // pitch = atan2(ay, sqrt(ax^2 + az^2)) * (180 / PI)
    mpuPitch = atan2(ay, sqrt((long)ax * ax + (long)az * az)) * 57.2958;
}

// ======================= LCD 更新 =======================
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.print((millis() - startTime) / 1000);
  lcd.print("s       ");

  lcd.setCursor(0, 1);
  lcd.print("Dist:");
  lcd.print(distance);
  lcd.print("cm ");
  lcd.print("Angle:");
  lcd.print(mpuPitch, 1);
}

// ======================= SETUP =======================
void setup() {
  lcd.begin(16, 2);
  Wire.begin();
  mpu.initialize();

  pinMode(IR_LEFT,  INPUT_PULLUP);
  pinMode(IR_MID,   INPUT_PULLUP);
  pinMode(IR_RIGHT, INPUT_PULLUP);

  pinMode(ENC_LEFT,  INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);

  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  // 启用 PCINT1 (A0-A5)
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);

  startTime = millis();

  lcd.print("TOUCH MORE GRASS");
  delay(1000);
}

// ======================= LOOP 主程序 =======================
void loop() {
  bool L, M, R;
  updateMPU();   // 加在这里，不会打扰任何逻辑
  updateDist();
  readIRSensors(L, M, R);

  // ----------- 永久停止状态保持 -----------
  if (stoppedForever) {
    motorStopHard();
    return;
  }

  // ====================  坡道结束事件触发：停止 4 秒并旋转  ====================
  // pitch > 10° 代表上坡（你已有的自动增速逻辑也会检测）
  if (mpuPitch > 10 && !uphillDetected) {
    uphillDetected = true;     // 标记已经经过坡
  }

  // 上坡已经发生过，而且现在下来了（pitch 回归到 -10° ~ +10°）
  if (uphillDetected && !slopeActionDone && abs(mpuPitch) <= 10) {

    // 进入 4 秒停车阶段
    if (!slopePause) {
      slopePause = true;
      slopePauseStart = millis();
      setMotor(0, 0);
    }

    // 4 秒静止期
    if (slopePause && millis() - slopePauseStart < 4000) {
      setMotor(0, 0);
      updateLCD();
      delay(1);
      return;
    }

    // 4 秒结束 → 进入旋转阶段
    if (slopePause && millis() - slopePauseStart >= 4000 && millis() - slopePauseStart < 6000) {
      // 顺时针旋转：左前右后
      setMotor(255, -255);
      updateLCD();
      delay(1);
      return;
    }

    // 旋转完成（共 2 秒）
    if (slopePause && millis() - slopePauseStart >= 6000) {
      slopePause = false;
      slopeActionDone = true;  // 标记已完成，不再重复
      // 恢复正常程序
    }
  }

  // ------------330 2s 停车----------------
  if (!distPause && distance >= 330.0) {
      distPause = true;
      distPauseStart = millis();
      setMotor(0, 0);     // 停车
    }

  if (distPause) {
    // 停车状态保持
    setMotor(0, 0);

    // 等待 2 秒结束
    if (millis() - distPauseStart >= 2000) {
      distPause = false;   // 恢复正常程序
    }

    updateLCD();
    delay(1);
    return;   // 不让任何其他逻辑运行
  }

// =================== 坡道智能变速逻辑 ===================
  // 上坡：角度 > +10°
  if (mpuPitch > 10) {
    speedUpHill = true;
    speedDownHill = false;
  }

  // 下坡：角度 < -10°
  if (mpuPitch < -10) {
    speedDownHill = true;
    speedUpHill = false;
  }

  // 恢复正常逻辑（角度回归正常范围）
  if (mpuPitch <= 10 && mpuPitch >= -10) {
    speedUpHill = false;
    speedDownHill = false;
  }

  // —— 变速状态处理（抢占优先级，不触发任何 line-follow）——
  if (speedUpHill) {
    setMotor(255, 255);    // 全速上坡
    updateDist();
    updateLCD();
    delay(1);
    return;
  }

  if (speedDownHill) {
    setMotor(80, 80);      // 慢速下坡
    updateDist();
    updateLCD();
    delay(1);
    return;
  }

      // ----------- 全黑 → 永久停止 -----------
if (L && M && R) {
    if (allBlackTimer == 0) allBlackTimer = millis();
    if (millis() - allBlackTimer > 100) {
        motorStopHard();
        return;
    }
} else {
    allBlackTimer = 0;
}

// ----------- 原地旋转（全白超过300ms触发） -----------
if (!L && !M && !R) {
    if (allWhiteTimer == 0) allWhiteTimer = millis();  // 记录开始时间

    if (millis() - allWhiteTimer > 200) {      
        setMotor(255, -255);   // 顺时针旋转：左前右后
        updateLCD();
        return;
    }
} else {
    allWhiteTimer = 0;   // 只要有黑线，重置计时器
}

  // ----------- 速度补偿（软修正） -----------
  int trim = speedTrim();
  int Lspd = baseSpeed - trim;
  int Rspd = baseSpeed + trim;

  // ----------- 线路跟随逻辑 -----------
  if (M) {
    // 中间黑线 → 直行
    setMotor(Lspd, Rspd);
  } 
  else if ((M && L) || (L && !M && !R)) {
    // 左黑线 → 右转硬修正
    setMotor(Lspd - 100, Rspd + 100);
  }
  else if ((M && R) || (!L && !M && R)) {
    // 右黑线 → 左转硬修正
    setMotor(Lspd + 100 , Rspd - 100);
  }

  // ----------- 更新显示 -----------  
  updateDist();
  updateLCD();
  delay(1);
}
