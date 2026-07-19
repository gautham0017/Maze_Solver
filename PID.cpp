/*
The PID module is the real-time motion correction system of the maze solver robot. 
Its purpose is to use sensor feedback to continuously detect movement errors and automatically 
adjust the motor outputs, ensuring the robot drives straight, turns accurately, maintains stable speed,
and navigates the maze with the precision required for successful exploration and fast speed runs.
*/

#include "PID.h"
#include <Arduino.h>

PIDController::PIDController(){
    kp = 0.0f;
    ki = 0.0f;
    kd = 0.0f;
    error = 0.0f;
    previousError = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    minOutput = -255.0f;
    maxOutput = 255.0f;
    integralMin = -1000.0f;
    integralMax = 1000.0f;
}
PIDController::PIDController(float Kp,float Ki,float Kd){
    kp = Kp;
    ki = Ki;
    kd = Kd;
    error = 0.0f;
    previousError = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    minOutput = -255.0f;
    maxOutput = 255.0f;
    integralMin = -1000.0f;
    integralMax = 1000.0f;
}
void PIDController::begin(){
    reset();
}

float PIDController::compute(float setpoint,float measurement,float dt){
    if(dt <= 0.0f)
        return 0.0f;
    error = setpoint - measurement;

    integral += error * dt;
    integral = constrain(integral,integralMin,integralMax);
    derivative =(error - previousError) / dt;

    float output =kp * error+ ki * integral+ kd * derivative;
    output = constrain(output,minOutput,maxOutput);

    previousError = error;
    return output;
}
void PIDController::reset(){
    error = 0.0f;
    previousError = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
}
void PIDController::setKp(float Kp){
    kp = Kp;
}
void PIDController::setKi(float Ki){
    ki = Ki;
}
void PIDController::setKd(float Kd){
    kd = Kd;
}
float PIDController::getKp() const{
    return kp;
}
float PIDController::getKi() const{
    return ki;
}
float PIDController::getKd() const{
    return kd;
}
void PIDController::setOutputLimits(float minimum,float maximum){
    minOutput = minimum;
    maxOutput = maximum;
}
void PIDController::setIntegralLimits(float minimum,float maximum){
    integralMin = minimum;
    integralMax = maximum;
}
float PIDController::getError() const{
    return error;
}
float PIDController::getIntegral() const{
    return integral;
}
float PIDController::getDerivative() const{
    return derivative;
}
