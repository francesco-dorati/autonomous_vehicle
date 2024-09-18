#include "SensorsController.h"

SensorsController::SensorsController(int L_TRIG, int R_TRIG, int FL_E, int FR_E, int BL_E, int BR_E) {
    LEFT_TRIG = L_TRIG;
    RIGHT_TRIG = R_TRIG;
    FL_ECHO = FL_E;
    FR_ECHO = FR_E;
    BL_ECHO = BL_E;
    BR_ECHO = BR_E;

    if (rate < 2*TIME_BETWEEN_UPDATES_MS) rate = 2*TIME_BETWEEN_UPDATES_MS;
    _rate_ms = rate;

    _next = 0;
    _last_update_ms = millis();
    
    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(FL_ECHO, INPUT);
    pinMode(FR_ECHO, INPUT);
    pinMode(BL_ECHO, INPUT);
    pinMode(BR_ECHO, INPUT);
}

void SensorsController::update(float *dist) {
    switch (_next) {
        case 0:
            *dist[0] = _update_sensor(LEFT_TRIG, FL_ECHO);
            break;
        case 1:
            *dist[1] = _update_sensor(RIGHT_TRIG, FR_ECHO);
            break;
        case 2:
            *dist[2] = _update_sensor(LEFT_TRIG, BL_ECHO);
            break;
        case 3:
            *dist[3] = _update_sensor(RIGHT_TRIG, BR_ECHO);
            break;
        
        _next = (_next + 1) % 4;
    }
}

float SensorsController::_update_sensor(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    return pulseIn(echo, HIGH) / 58.2;
}
