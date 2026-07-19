#ifndef TURN_CONTROLLER_H
#define TURN_CONTROLLER_H

#include <Arduino.h>

#include "Motor.h"
#include "MPU.h"
#include "PID.h"

class TurnController{
private:
    Motor* leftMotor;
    Motor* rightMotor;
    MPU* imu;
    PIDController pid;
    float tolerance;
    int minimumPWM;
    int maximumPWM;
    float targetHeading;
    bool targetInitialized;

public:
    TurnController();
    void begin(Motor* left,Motor* right,MPU* sensor);
    bool turnTo(float angle);
    bool turnLeft90();
    bool turnRight90();
    bool turnAround();
    void stop();

private:
    float normalize(float angle);

};

#endif
