#include <Arduino.h>

const int STBY = 19;
const int PWMA = 23, AIN1 = 21, AIN2 = 22;
const int PWMB = 2, BIN1 = 18, BIN2 = 16;

void setMotor(int dir1, int dir2, int pwm, int speed) {
    digitalWrite(STBY, HIGH);
    
    if (speed > 0) {
        digitalWrite(dir1, HIGH);
        digitalWrite(dir2, LOW);
    } else if (speed < 0) {
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
    } else {
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, LOW);
    }
    
    analogWrite(pwm, abs(speed));
}

void setup() {
    Serial.begin(115200);
    
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    
    Serial.println("H-Bridge test ready");
}

void loop() {
    Serial.println("Both motors forward 50%");
    setMotor(AIN1, AIN2, PWMA, 128);
    setMotor(BIN1, BIN2, PWMB, 128);
    delay(2000);
    
    Serial.println("Both motors backward 50%");
    setMotor(AIN1, AIN2, PWMA, -128);
    setMotor(BIN1, BIN2, PWMB, -128);
    delay(2000);
    
    Serial.println("Turn left");
    setMotor(AIN1, AIN2, PWMA, -128);
    setMotor(BIN1, BIN2, PWMB, 128);
    delay(2000);
    
    Serial.println("Turn right");
    setMotor(AIN1, AIN2, PWMA, 128);
    setMotor(BIN1, BIN2, PWMB, -128);
    delay(2000);
    
    Serial.println("Speed test 0-100%");
    for (int speed = 0; speed <= 255; speed += 25) {
        setMotor(AIN1, AIN2, PWMA, speed);
        setMotor(BIN1, BIN2, PWMB, speed);
        delay(500);
    }
    
    Serial.println("Stop");
    digitalWrite(STBY, LOW);
    delay(2000);
}