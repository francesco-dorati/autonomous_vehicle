#include "MotorController.h"

MotorController::MotorController(
      uint8_t enable_pin, 
      uint8_t in1_pin, uint8_t in2_pin, 
      uint8_t enc_a_pin, uint8_t enc_b_pin, 
      float wheel_radius, int counts_per_rev, bool reverse,
      double kp, double ki, double kd
):
  _enable_pin(enable_pin), 
  _in1_pin(in1_pin), _in2_pin(in2_pin), 
  _encoder(enc_a_pin, enc_b_pin),
  _motor_pid(&actual_rpm, &power, &goal_rpm, ki, kp, kd, DIRECT),
  _wheel_radius(wheel_radius),
  _ticks_per_rev(counts_per_rev),
  _reverse(reverse)
{
  pinMode(_enable_pin, OUTPUT);
  pinMode(_in1_pin, OUTPUT);
  pinMode(_in2_pin, OUTPUT);

  _motor_pid.SetMode(AUTOMATIC);
  _motor_pid.SetOutputLimits(-PWR_MAX, PWR_MAX);
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

void MotorController::update(double goal_rpm) {

  // update velocity and space
  long d_ticks = _delta_ticks();
  unsigned double d_time_s = (micros() - _encoder_prev_time) / 1000000;
  // velocity_cms = (d_ticks * 2*PI*_wheel_radius) / (_ticks_per_rev * d_time_s); // (cm / s)
  space_cm = (d_ticks * 2*PI*_wheel_radius) / _ticks_per_rev; // (cm)
  velocity_rpm = (d_ticks * 60) / (_ticks_per_rev*d_time_s); // (rev / min)

  // Compute PID
  if (abs(goal_rpm) < MIN_RPM) 
    power = 0.0;
  else 
    _motor_pid.Compute();

  _set_motor_power(power);
  return actual_rpm;
}

long MotorController::_delta_ticks() {
  long actual_ticks = _reverse ? -_encoder.read() : _encoder.read();
  long delta = actual_ticks - _encoder_prev_counts;
  _encoder_prev_counts = actual_ticks;
  return delta;
}

// double MotorController::_get_current_rpm() {
//     long encoder_counts = _ticks();
//     int delta_counts = encoder_counts - _encoder_prev_counts;
//     _encoder_prev_counts = encoder_counts;
//     double delta_t_ms = ((double) (micros() - _encoder_prev_time))/1000;
//     _encoder_prev_time = micros();
//     return (delta_counts*60000) / (delta_t_ms*_ticks_per_rev); 
// }

void MotorController::_set_motor_power(double power) {
  int pwm_val = constrain(power, -PWR_MAX, PWR_MAX);

  // MIN POWER
  if (abs(pwm_val) < PWR_MIN) {
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