#ifndef MOTOR_H
#define MOTOR_H
#include "Arduino.h"

class Motor {
  public:
    Motor(uint8_t pin1, uint8_t pin2);

    void forward(uint8_t pwm);   
    void backward(uint8_t pwm);
    void stop();

  private:
    uint8_t pin1_;
    uint8_t pin2_;
};

#endif