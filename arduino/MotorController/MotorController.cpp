#include "MotorController.h"

MotorController::MotorController(
  uint8_t enable_pin, 
  uint8_t in1_pin, uint8_t in2_pin, 
  uint8_t enc_a_pin, uint8_t enc_b_pin, 
  int counts_per_rev, bool reverse
):
  _enable_pin(enable_pin), 
  _in1_pin(in1_pin), 
  _in2_pin(in2_pin), 
  _encoder(enc_a_pin, enc_b_pin),
  _motor_pid(&_actual_rpm, &_power, &_goal_rpm, KI, KP, KD, DIRECT),
  _ticks_per_rev(counts_per_rev),
  _reverse(reverse)
{
  pinMode(_enable_pin, OUTPUT);
  pinMode(_in1_pin, OUTPUT);
  pinMode(_in2_pin, OUTPUT);

  _encoder_prev_counts = 0;
  _encoder_prev_time = 0;

  _goal_rpm = 0;
  _actual_rpm = 0;
  _power = 0;

  _motor_pid.SetMode(AUTOMATIC);
  _motor_pid.SetOutputLimits(PWM_MIN, PWM_MAX);
}

// float MotorController::update(float goal_rpm) {
//   if (goal_rpm < MIN_RPM && goal_rpm > -MIN_RPM) {
//     _set_motor_power(0);
//     return 0;
//   }

//   if (goal_rpm > MAX_RPM) goal_rpm = MAX_RPM;
//   else if (goal_rpm < -MAX_RPM) goal_rpm = -MAX_RPM;


//   float curent_rpm = _update_current_rpm();

//   float error_rpm = goal_rpm - curent_rpm;

//   if (abs(error_rpm) < 5) error_rpm = 0;
  
//   _power += floor(2*error_rpm);
//   if (_power > 255) _power = 255;
//   else if (_power < -255) _power = -255;

//   _set_motor_power(_power);

//   return curent_rpm;
// }

double MotorController::update(double goal_rpm) {
  _goal_rpm = goal_rpm;
  _actual_rpm = _get_current_rpm();

  if (_goal_rpm == 0.0) _power = 0.0;
  
  else _motor_pid.Compute();

  _set_motor_power(_power);
  return _actual_rpm;
}

long MotorController::ticks() {
  return _reverse ? -_encoder.read() : _encoder.read();
}

double MotorController::_get_current_rpm() {
    long encoder_counts = ticks();
    int delta_counts = encoder_counts - _encoder_prev_counts;
    _encoder_prev_counts = encoder_counts;
    double delta_t_ms = ((double) (micros() - _encoder_prev_time))/1000;
    _encoder_prev_time = micros();
    return (delta_counts*60000) / (delta_t_ms*_ticks_per_rev); 
}

void MotorController::_set_motor_power(double power) {
  int pwm_val = constrain(power, -255, 255);

  // MIN POWER
  if (pwm_val < 60 && pwm_val > -60) {
    pwm_val = 0;
  }

  if (pwm_val > 0) {
    digitalWrite(_in1_pin, LOW);
    digitalWrite(_in2_pin, HIGH);
    analogWrite(_enable_pin, pwm_val);
  } else if (pwm_val < 0) {
    digitalWrite(_in1_pin, HIGH);
    digitalWrite(_in2_pin, LOW);
    analogWrite(_enable_pin, -pwm_val);
  } else {
    digitalWrite(_in1_pin, LOW);
    digitalWrite(_in2_pin, LOW);
    analogWrite(_enable_pin, 0);
  }
}