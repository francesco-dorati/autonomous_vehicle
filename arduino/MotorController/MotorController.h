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

#define KP 1
#define KI 0
#define KD 0

class MotorController {
  public:
    MotorController(
      uint8_t enable_pin, 
      uint8_t in1_pin, uint8_t in2_pin, 
      uint8_t enc_a_pin, uint8_t enc_b_pin, 
      int counts_per_rev, bool reverse
    );

    float update(float goal_rpm);
    long ticks();

  private:
      Encoder _encoder;
      PID _motor_pid;
      uint8_t _enable_pin, _in1_pin, _in2_pin;
      int _counts_per_rev;
      bool _reverse;

      float _goal_rpm, _actual_rpm;
      int _power;

      long _encoder_prev_counts;
      unsigned long _encoder_prev_time;
      float _get_current_rpm();

      void _set_motor_power(int power);
};  

#endif 

