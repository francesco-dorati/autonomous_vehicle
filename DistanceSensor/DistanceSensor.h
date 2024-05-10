#ifndef DistanceSensor_h
#define DistanceSensor_h

#include "Arduino.h"

class DistanceSensor {
  public:
    DistanceSensor(int trig_pin, int echo_pin);
    int getDist();
private:
    int _trig_pin;
    int _echo_pin;
};

#endif 
