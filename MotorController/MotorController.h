#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"
#include "Encoder.h"

class MotorController {
  public:
    MotorController(
      uint8_t enable_pin, 
      uint8_t in1_pin, uint8_t in2_pin, 
      uint8_t enc_a_pin, uint8_t enc_b_pin, 
      int counts_per_rev, bool reverse
    );

    float set_velocity(float goal_rpm);

  private:
      Encoder _encoder;
      uint8_t _enable_pin, _in1_pin, _in2_pin;
      int _counts_per_rev;
      bool _reverse;

      int _power = 0;

      long _encoder_prev_counts = 0;
      unsigned long _encoder_prev_time = 0;
      float _get_current_rpm();

      void _set_motor_power(int power);
};  

#endif 

