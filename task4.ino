#include <LiquidCrystal.h>

// ===================== LCD =====================
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ===================== Motor Pins =====================
#define L_PWM   11
#define R_PWM   3
#define L_IN1   A1
#define L_IN2   2
#define R_IN1   12
#define R_IN2   13

void setup() {
    // LCD
    lcd.begin(16, 2);
    lcd.print("BT Car Ready");

    // Motors
    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    // HC-05 Serial
    Serial.begin(9600);
}

// ===================== Motor Control =====================
void forward() {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    analogWrite(L_PWM, 180);
    analogWrite(R_PWM, 180);
}

void backward() {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    analogWrite(L_PWM, 180);
    analogWrite(R_PWM, 180);
}

void left() {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    analogWrite(L_PWM, 150);
    analogWrite(R_PWM, 150);
}

void right() {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    analogWrite(L_PWM, 150);
    analogWrite(R_PWM, 150);
}

void stopCar() {
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();

        lcd.clear();
        lcd.print("CMD: ");
        lcd.print(cmd);

        switch (cmd) {
            case 'F': forward(); break;
            case 'B': backward(); break;
            case 'L': left(); break;
            case 'R': right(); break;
            case 'S': stopCar(); break;
        }
    }
}
