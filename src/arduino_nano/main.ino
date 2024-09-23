#include <Arduino.h>
#include <math.h>

#define UPDATE_FREQ 50 // Hz

#define ENA 5
#define IN1 9
#define IN2 4
#define ENB 6
#define IN3 7
#define IN4 8

#define MAX_POW 255
#define MIN_POW 0

int pow_l = 0, pow_r = 0;
void setup() {
    Serial.begin(9600);
    // Left Motor
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    // Right Motor
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}


void loop() {
    // read from serial
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'P') {
            if (Serial.available() < 2) return;
            pow_l = Serial.parseInt();
            pow_r = Serial.parseInt();
        } else return;
        Serial.print("POW: ");
        Serial.print(pow_l);
        Serial.print(" ");
        Serial.print(pow_r);
        Serial.print(" ");
        Serial.println(abs(pow_r));
    }

    // check max/min values
    // left
    if (abs(pow_l) > MAX_POW) pow_l = (pow_l > 0) ? MAX_POW : -MAX_POW;
    if (abs(pow_l) < MIN_POW) pow_l = 0;
    // right
    if (abs(pow_r) > MAX_POW) pow_r = (pow_r > 0) ? MAX_POW : -MAX_POW;
    if (abs(pow_r) < MIN_POW) pow_r = 0;

    // update motors
    // left
    if (pow_l == 0){
        analogWrite(ENA, 0);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    } else {
        analogWrite(ENA, abs(pow_l));
        digitalWrite(IN1, (pow_l > 0) ? HIGH : LOW);
        digitalWrite(IN2, (pow_l > 0) ? LOW : HIGH);
    }
    // right
    if (pow_r == 0){
        analogWrite(ENB, 0);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    } else{
        analogWrite(ENB, abs(pow_r));
        digitalWrite(IN3, (pow_r > 0) ? HIGH : LOW);
        digitalWrite(IN4, (pow_r > 0) ? LOW : HIGH);
    }

    delay(1000/UPDATE_FREQ);
}
