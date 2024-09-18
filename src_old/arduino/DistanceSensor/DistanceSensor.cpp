#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(int trig_pin, int echo_pin) {
  _trig_pin = trig_pin;
  _echo_pin = echo_pin;
  pinMode(_trig_pin, OUTPUT);
  pinMode(_echo_pin, INPUT);
}

int DistanceSensor::getDist() {
  digitalWrite(_trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trig_pin, LOW);

  return pulseIn(_echo_pin, HIGH) / 58.2;
}