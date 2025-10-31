#include "motor.h"

const uint8_t LEFT_MOTOR_PIN1 = 3;
const uint8_t LEFT_MOTOR_PIN2 = 5;
const uint8_t RIGHT_MOTOR_PIN1 = 6;
const uint8_t RIGHT_MOTOR_PIN2 = 9;

Motor leftMotor(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2);
Motor rightMotor(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2);

void setup() {
}

void loop() {

  leftMotor.forward(200);
  rightMotor.forward(200);
  delay(2000);

  leftMotor.stop();
  rightMotor.stop();
  delay(1000);

  leftMotor.backward(150);
  rightMotor.backward(150);
  delay(2000);
  
}
