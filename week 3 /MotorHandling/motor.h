#include "motor.h"

Motor::Motor(uint8_t pin1, uint8_t pin2) {
  pin_1 = pin1;
  pin_2 = pin2;
  pinMode(pin_1, OUTPUT);
  pinMode(pin_2, OUTPUT);
}

void Motor::forward(uint8_t pwm) {
  analogWrite(_pin1, pwm);
  analogWrite(_pin2, 0);
}

void Motor::backward(uint8_t pwm) {
  analogWrite(_pin1, 0);
  analogWrite(_pin2, pwm);
}

void Motor::stop() {
  analogWrite(_pin1, 0);
  analogWrite(_pin2, 0);
}
