#include "MotorController.h"

MotorController::MotorController(int enable_pin, int in1_pin, int in2_pin, int enc_a_pin, int enc_b_pin, float counts_per_rev) {
  _enable_pin = enable;
  _in1_pin = in1;
  _in2_pin = in2;
  _enc_a_pin = enc_a_pin;
  _enc_b_pin = enc_b_pin;
  _counts_per_rev = counts_per_rev;

  pinMode(_enable_pin, OUTPUT);
  pinMode(_in1_pin, OUTPUT);
  pinMode(_in2_pin, OUTPUT);
  pinMode(_enc_a_pin, INPUT);
  pinMode(_enc_b_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(_enc_b_pin), _read_encoder, RISING);
}

float MotorController::updateVelocity(float goal_rpm) {
  // goal_rps in round/s
  // return current_rps in round/s

  int power;
  float error_rpm;
  int delta_counts;
  unsigned long delta_t_ms;

  // update speed
  delta_counts = _encoder_counts - _encoder_prev_counts;
  delta_t_ms = millis() - _encoder_prev_time_ms;
  _encoder_prev_counts = _encoder_counts;
  _encoder_prev_time_ms = millis();
  _current_rpm = (delta_counts*60000) / (delta_t_ms*_counts_per_rev); 
  
  // calculate error
  error_rpm = goal_rpm - _current_rpm;

  // PID
  _error_integral = _error_integral + error_rpm*delta_t_ms;
  power = _kp*error_rpm + _ki*_error_integral;

  // saturation
  if (power > 255) power = 255;
  else if (power < -255) power = -255;

  _set_motor_power(power);

  return _current_rpm;
}

void MotorController::_set_motor_power(int power) {
  if (power > 0) {
    digitalWrite(_in1_pin, HIGH);
    digitalWrite(_in2_pin, LOW);
    analogWrite(_enable_pin, power);
  } else if (power < 0) {
    digitalWrite(_in1_pin, LOW);
    digitalWrite(_in2_pin, HIGH);
    analogWrite(_enable_pin, -power);
  } else {
    digitalWrite(_in1_pin, LOW);
    digitalWrite(_in2_pin, LOW);
    analogWrite(_enable_pin, 0);
  }
}

void MotorController::_read_encoder() {
  if (digitalRead(_enc_a_pin) > 0) {
    pos--;
  } else {
    pos++;
  }
}



