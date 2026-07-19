/*
The Motor module is the low-level motion control layer of the ESP32 maze solver robot.
Its purpose is to translate high-level movement commands into precise electrical control signals
for the DRV8833 motor driver, enabling accurate control of motor direction, speed, stopping, 
and braking so the robot can execute the navigation decisions made by the Flood Fill algorithm and other control systems.
*/

#include "Motor.h"

#define PWM_FREQ        20000      // 20 kHz
#define PWM_RESOLUTION  8          // 0-255

Motor::Motor(){  //non-parametrized constructor
    in1 = 0;
    in2 = 0;
    currentSpeed = 0;
}
Motor::Motor(uint8_t pin1,uint8_t pin2){ //parametrized constructor
    in1 = pin1;
    in2 = pin2;
    currentSpeed = 0;
}

bool Motor::begin() { //Initializes hardware.
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    bool ok1 = ledcAttach(in1,PWM_FREQ,PWM_RESOLUTION);
    bool ok2 = ledcAttach(in2,PWM_FREQ,PWM_RESOLUTION);
    stop();
    return ok1 && ok2;
}

void Motor::forward(uint8_t pwm){
    pwm = constrain(pwm,0,255);
    ledcWrite(in1,pwm);
    ledcWrite(in2,0);
    currentSpeed = pwm;
}
void Motor::backward(uint8_t pwm){
    pwm = constrain(pwm,0,255);
    ledcWrite(in1,0);
    ledcWrite(in2,pwm);
    currentSpeed = -pwm;
}
void Motor::stop(){
    ledcWrite(in1,0);
    ledcWrite(in2,0);
    currentSpeed = 0;
}
void Motor::brake(){
    ledcDetach(in1);
    ledcDetach(in2);
    pinMode(in1,OUTPUT);
    pinMode(in2,OUTPUT);
    digitalWrite(in1,HIGH);
    digitalWrite(in2,HIGH);
    delay(5);
    ledcAttach(in1,PWM_FREQ,PWM_RESOLUTION);
    ledcAttach(in2,PWM_FREQ,PWM_RESOLUTION);
    ledcWrite(in1,0);
    ledcWrite(in2,0);
    currentSpeed = 0;
}

void Motor::setSpeed(int pwm){
    pwm = constrain(pwm,-255,255);
    if(pwm>0){
        forward(pwm);
    }
    else if(pwm<0){
        backward(-pwm);
    }
    else{
        stop();
    }
}

int Motor::getSpeed() const{
    return currentSpeed;
}
