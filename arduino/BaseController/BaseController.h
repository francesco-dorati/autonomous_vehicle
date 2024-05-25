#ifndef BaseController_h
#define BaseController_h

#include "Arduino.h"
#include "../MotorController/MotorController.h"
#include "../DistanceSensor/DistanceSensor.h"

#define DIST_TRESHOLD 10

struct state {
    int x;
    int y;
    int theta;
    state(): x(0), y(0), theta(0) {};
    state(int x, int y, int t): x(x), y(y), theta(t) {};
};


class BaseController {
  public:
    BaseController(
        int mr_pwm, int mr_in1, int mr_in2, int mr_enc_a, int mr_enc_b,
        int ml_pwm, int ml_in1, int ml_in2, int ml_enc_a, int ml_enc_b,
        int s1_trig, int s1_echo, 
        float wheel_radius, float wheel_distance
    );
    state updateState(unsigned float linear_velocity, float angular_velocity, unsigned float delta_t);

    private:
        float _wheel_radius, _wheel_distance;
        MotorController _motor_right;
        MotorController _motor_left;
        DistanceSensor _sensor1;
        state _state;
        unsigned long _odometry_prev_time = 0;
        
        state _updateOdometry(float rps_right, float rps_left);
};

#endif