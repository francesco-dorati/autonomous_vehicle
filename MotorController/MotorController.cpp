#include "MotorController.h"

MotorController::MotorController(
  uint8_t enable_pin, 
  uint8_t in1_pin, uint8_t in2_pin, 
  uint8_t enc_a_pin, uint8_t enc_b_pin, 
  int counts_per_rev, bool reverse = false
):
  _enable_pin(enable_pin), 
  _in1_pin(in1_pin), 
  _in2_pin(in2_pin), 
  _encoder(enc_a_pin, enc_b_pin),
  _counts_per_rev(counts_per_rev),
  _reverse(reverse)
{
  pinMode(_enable_pin, OUTPUT);
  pinMode(_in1_pin, OUTPUT);
  pinMode(_in2_pin, OUTPUT);
}

float MotorController::velocity(float goal_rpm) {
  if (goal_rpm < 50 && goal_rpm > -50) {
    _set_motor_power(0);
    return 0;
  }

  if (goal_rpm > 150) goal_rpm = 150;
  else if (goal_rpm < -150) goal_rpm = -150;

  float curent_rpm = _get_current_rpm();

  float error_rpm = goal_rpm - curent_rpm;

  if (abs(error_rpm) < 5) error_rpm = 0;
  
  _power += floor(2*error_rpm);
  if (_power > 255) _power = 255;
  else if (_power < -255) _power = -255;

  _set_motor_power(_power);

  return curent_rpm;
}

float MotorController::_get_current_rpm() {
    long encoder_counts = _reverse ? -_encoder.read() : _encoder.read();
    int delta_counts = encoder_counts - _encoder_prev_counts;
    _encoder_prev_counts = encoder_counts;
    float delta_t_ms = ((float) (micros() - _encoder_prev_time))/1000;
    _encoder_prev_time = micros();
    return (delta_counts*60000) / (delta_t_ms*_counts_per_rev); 
}

void MotorController::_set_motor_power(int power) {
  if (power > 0) {
    digitalWrite(_in1_pin, LOW);
    digitalWrite(_in2_pin, HIGH);
    analogWrite(_enable_pin, power);
  } else if (power < 0) {
    digitalWrite(_in1_pin, HIGH);
    digitalWrite(_in2_pin, LOW);
    analogWrite(_enable_pin, -power);
  } else {
    digitalWrite(_in1_pin, LOW);
    digitalWrite(_in2_pin, LOW);
    analogWrite(_enable_pin, 0);
  }
}