#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

class Motor{

private:
    uint8_t in1;
    uint8_t in2;

public:
    Motor();
    Motor(uint8_t pin1, uint8_t pin2); 
    void begin(); //Initializes the motor hardware.It will configure GPIOs,initialize PWM,stop the motor.
    void setSpeed(int pwm);
    void forward(int pwm);
    void backward(int pwm);
    void stop();
    void brake();
    int getSpeed() const;

private:
    int currentSpeed;

};
#endif