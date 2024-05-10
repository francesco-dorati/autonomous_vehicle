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

  attachInterrupt(digitalPinToInterrupt(enc_b), readEncoder, RISING);
}

float MotorController::updateVelocity(float goal_rps) {
  // goal_rps in round/s
  // return current_rps in round/s

  int power;
  float rps_error;
  float proportional, integral;
  
  if (_encoder_prev_counts == 0) { // first iteration
    _current_rps = 0;
    _encoder_prev_time = millis();

  } else { // update current rps
    // calculate delta values
    int delta_counts = _encoder_counts - _encoder_prev_counts;
    unsigned long delta_t = millis() - _encoder_prev_time;

    // calculate current speed
    _current_rps = delta_counts / (delta_t*_counts_per_rev);

    // update values for next iteration
    _encoder_prev_counts = _encoder_counts;
    _encoder_prev_time = millis();
  }
  
  // calculate error
  rps_error = goal_rps - _current_rps;

  // PID
  proportional = _kp*rps_error;
  integral = _ki*rps_error*delta_t;
  power = proportional + integral;

  // saturation
  if (power > 255) power = 255;
  else if (power < -255) power = -255;

  _set_motor_power(power);

  return _current_rps;
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
  if (digitalRead(enc_a) > 0) {
    pos--;
  } else {
    pos++;
  }
}



