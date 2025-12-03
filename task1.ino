#include <LiquidCrystal.h>

// ======================= LCD 屏幕 =======================
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ======================= 引脚定义 =======================
// IR 传感器：左 & 右黑线→LOW，中间黑线→HIGH
#define IR_LEFT   0
#define IR_MID    A3
#define IR_RIGHT  A0

// 编码器（PCINT1）
#define ENC_LEFT  A1  // PCINT21
#define ENC_RIGHT A2   // PCINT20

// 电机引脚
#define L_PWM   11
#define R_PWM   3
#define L_IN1   1
#define L_IN2   2
#define R_IN1   12
#define R_IN2   13

//车辆参数
const int baseSpeed = 90;
const float wheelCirc = 39;
const float encoderPPR = 40.0;
volatile long leftCount = 0;
volatile long rightCount = 0;

unsigned long startTime = 0;
unsigned long allWhiteTimer = 0;
unsigned long allBlackTimer = 0;

float distance = 0.0;
bool stoppedForever = false;

volatile bool lastA1 = HIGH;
volatile bool lastA2 = HIGH;
bool pause = true;

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
volatile bool lastA4 = HIGH;
volatile bool lastA5 = HIGH;

ISR(PCINT1_vect) {
  bool currA1 = digitalRead(ENC_LEFT);
  bool currA2 = digitalRead(ENC_RIGHT);

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
  return constrain(diff * 2, -70, 70);
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

// ======================= LCD 更新 =======================
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.print((millis() - startTime) / 1000);
  lcd.print("s       ");

  updateDist();
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print("cm      ");
}

// ======================= SETUP =======================
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);

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

  lcd.print("Line Car Ready!");
  delay(1000);
}

// ======================= LOOP 主程序 =======================
void loop() {
  bool L, M, R;
  readIRSensors(L, M, R);
  if (pause && distance>=330.0) { 
      setMotor(0,0); 
      delay(1000); 
      updateLCD();
      delay(1000); 
      updateLCD(); 
      pause=false;
      return; 
    }

  // ----------- 永久停止状态保持 -----------
  if (stoppedForever) {
    motorStopHard();
    return;
  }

      // ----------- 全黑 → 永久停止 -----------
  if (L && M && R) {
    motorStopHard();
    return;
  }

// ----------- 原地旋转（全白超过300ms触发） -----------
if (!L && !M && !R) {
    if (allWhiteTimer == 0) allWhiteTimer = millis();  // 记录开始时间

    if (millis() - allWhiteTimer > 200) {      
        setMotor(200, -200);   // 顺时针旋转：左前右后
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
    setMotor(Lspd - 200, Rspd + 190 );
  }
  else if ((M && R) || (!L && !M && R)){
    // 右黑线 → 左转硬修正
    setMotor(Lspd + 100, Rspd - 150);
  }

  // ----------- 更新显示 -----------  
  updateDist();
  updateLCD();
  delay(1);
}
