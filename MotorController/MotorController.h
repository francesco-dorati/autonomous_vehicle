#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"

class MotorController {
  public:
    MotorController(int enable_pin, int in1_pin, int in2_pin, int enc_a_pin, int enc_b_pin, float counts_per_rev);
    float updateVelocity(float goal_rpm);

  private:
      int _enable_pin, _in1_pin, _in2_pin, _enc_a_pin, _enc_b_pin;
      float _current_rpm = 0;

      int _encoder_counts = 0;
      int _encoder_prev_counts = 0;
      unsigned long _encoder_prev_time_ms = 0;
      float _counts_per_rev;

      float _kp = 2;
      float _ki = 0.5;
      float _error_integral = 0;

      void _set_motor_power(int power);
      void _read_encoder(); // update encoder counts
};

#endif 

