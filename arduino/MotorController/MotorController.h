#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"
#include "Encoder.h"
#include <PID_v1.h>

#define MAX_RPM 15
#define MIN_RPM 130

#define PWM_MAX 255
#define PWM_MIN -255
#define TICKS_PER_REV 1500

#define KP 12.0
#define KI .5
#define KD .01

class MotorController {
  public:
    MotorController(
      uint8_t enable_pin, 
      uint8_t in1_pin, uint8_t in2_pin, 
      uint8_t enc_a_pin, uint8_t enc_b_pin, 
      int counts_per_rev, bool reverse
    );

    double update(double goal_rpm);
    long ticks();
    
    double _goal_rpm = 0, _actual_rpm = 0, _power = 0;

  private:
    Encoder _encoder;
    PID _motor_pid;
    uint8_t _enable_pin, _in1_pin, _in2_pin;
    int _ticks_per_rev;
    bool _reverse;


    long _encoder_prev_counts;
    unsigned long _encoder_prev_time;
    double _get_current_rpm();

    void _set_motor_power(double power);
};  

#endif 

