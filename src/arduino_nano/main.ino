/*
    ARDUINO NANO
    battery level and distance sensor

    "P" -> "PNG"

    "B" -> "<battery_mv>"
    battery request
    battery_mv: battery level mv

    "S":
        "SR" -> "<d_fl> <d_fr> <d_rl> <d_rr>"
        sensor request
        d_*: distance mm
        
        "S0"
        stop sensors 
        
        "S1"
        start sensors

    ""
*/

#include <Arduino.h>

#define DELAY 20  // ms
#define END_CHAR '\n'

#define BATTERY_PIN A0
#define FL_TRIG_PIN 2
#define FL_ECHO_PIN 3
#define FR_TRIG_PIN 4
#define FR_ECHO_PIN 5
#define RL_TRIG_PIN 6
#define RL_ECHO_PIN 7
#define RR_TRIG_PIN 8
#define RR_ECHO_PIN 9

bool sensors_running = false;
int battery_mv = 0;
int distances_mm[4] = {0, 0, 0, 0};
int current_sensor = 0;

void setup() {
    Serial.begin(9600);
    pinMode(BATTERY_PIN, INPUT);
}

void loop() {
    handle_serial();
    if (sensors_running) update_sensors();
    delay(DELAY);
}

void handle_serial() {
    while (Serial.available() > 0) {
        char c0 = Serial.read();
        if (c0 == 'P') {
            // PING
            Serial.println("P");
        } else if (c0 == 'B') {
            // BATTERY
            update_battery();
            Serial.println(battery_mv);
        } else if (c0 == 'S') {
            // SENSORS
            char c1 = Serial.read();
            if (c1 == 'R') {
                // REQUEST
                Serial.print(distances_mm[0]); Serial.print(" ");
                Serial.print(distances_mm[1]); Serial.print(" ");
                Serial.print(distances_mm[2]); Serial.print(" ");
                Serial.println(distances_mm[3]);
            } else if (c1 == '1') {
                // START
                sensors_running = true;
                distances_mm[0] = 0; distances_mm[1] = 0;
                distances_mm[2] = 0; distances_mm[3] = 0;
            } else if (c1 == '0') {
                // STOP
                sensors_running = false;
                distances_mm[0] = 0; distances_mm[1] = 0;
                distances_mm[2] = 0; distances_mm[3] = 0;
            }
        }
        // consume input
        while (Serial.available() > 0 && Serial.read() != END_CHAR) {};
    }
}

int update_battery() {
    int pin_voltage_mv = (analogRead(BATTERY_PIN) / 1023.0) * 5000;
    battery_mv = pin_voltage_mv * 3;
}

int update_sensors() {
    switch (current_sensor) {
        case 0:
            distances_mm[0] = get_distance(FL_TRIG_PIN, FL_ECHO_PIN);
            break;
        case 1:
            distances_mm[1] = get_distance(FR_TRIG_PIN, FR_ECHO_PIN);
            break;
        case 2:
            distances_mm[2] = get_distance(RL_TRIG_PIN, RL_ECHO_PIN);
            break;
        case 3:
            distances_mm[3] = get_distance(RR_TRIG_PIN, RR_ECHO_PIN);
            break;
    }
    current_sensor = (current_sensor + 1) % 4;
}

int get_distance(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // speed of sound = 343 mm/ms
    // half speed of sound = 171.5 mm/ms = 0.1715 mm/us
    long duration_us = pulseIn(echo, HIGH);
    return (duration_us * 0.1715);
}
