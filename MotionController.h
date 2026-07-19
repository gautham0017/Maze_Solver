#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>

#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "TurnController.h"
#include "MPU.h"
#include "Config.h"

class MotionController{
private:
    Motor* leftMotor;
    Motor* rightMotor;
    Encoder* leftEncoder;
    Encoder* rightEncoder;
    MPU* imu;
    TurnController turnController;
    PIDController drivePID;

    float targetHeading;
    int baseSpeed;
    int minimumPWM;
    int maximumPWM;
    float cellDistance;
    bool initialized;

public:
    MotionController();
    void begin(Motor* left,Motor* right,Encoder* leftEnc,Encoder* rightEnc,MPU* sensor);
    bool driveOneCell();
    bool driveDistance(float distanceMM);
    bool turnLeft90();
    bool turnRight90();
    bool turnAround();
    void stop();
    void setBaseSpeed(int speed);
    int getBaseSpeed() const;

private:
    float getAverageDistance();
    void resetEncoders();
    void driveStraight(float correction);

};

extern MotionController motion;

#endif