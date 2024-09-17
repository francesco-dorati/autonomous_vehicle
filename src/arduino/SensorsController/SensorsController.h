#define TIME_BETWEEN_UPDATES_MS 10

#include "Arduino.h"

class SensorsController {
    public:
        SensorsController(uint8_t L_TRIG, uint8_t R_TRIG, uint8_t FL_E, uint8_t FR_E, uint8_t BL_E, uint8_t BR_E);
        void update(float *dist);

    private:
        uint8_t LEFT_TRIG, RIGHT_TRIG, FL_ECHO, FR_ECHO, BL_ECHO, BR_ECHO;
        int _rate_ms;
        bool _next;

        void _update_sensor(uint8_t trig, uint8_t echo);
        void _trigger(uint8_t pin);
};