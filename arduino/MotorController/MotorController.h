#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"
#include "Encoder.h"
#include <PID_v1.h>

#define MAX_RPM 130
#define MIN_RPM 15

#define PWR_MAX 250
#define PWR_MIN 50

/*
MOTOR CONTROLLER

public:
    - update(goal_rpm)
        -> update ticks and velocity
        -> update pid controller
    - velocity_cms (cm/s)
    - space_cm  (cm)
*/

class MotorController {
  public:
    MotorController(
      uint8_t enable_pin, 
      uint8_t in1_pin, uint8_t in2_pin, 
      uint8_t enc_a_pin, uint8_t enc_b_pin, 
      float wheel_radius, int counts_per_rev, bool reverse,
      double kp, double ki, double kd
    );

    void update(double goal_rpm);
    
    double goal_rpm = 0, actual_rpm = 0, power = 0;
    double space_cm = 0, velocity_rpm = 0;
 
  private:
    uint8_t _enable_pin, _in1_pin, _in2_pin;

    Encoder _encoder;
    PID _motor_pid;

    int _ticks_per_rev;
    float _wheel_radius;
    bool _reverse;
    long _encoder_prev_counts = 0;
    unsigned long _encoder_prev_time = 0;

    long _delta_ticks();

    void _set_motor_power(double power);
};  

#endif 

