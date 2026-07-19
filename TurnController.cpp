/*
The TurnController module is the precision turning system of your maze solver robot. 
Its purpose is to execute accurate rotations by combining motor control, gyroscope feedback, 
and PID control, ensuring that every commanded turn is completed with the correct orientation 
before the robot continues navigating the maze.
*/

#include "TurnController.h"

TurnController::TurnController() :pid(3.2f,0.0f,0.25f){
    tolerance = 1.0f;
    minimumPWM = 60;
    maximumPWM = 180;
    targetHeading = 0.0f;
    targetInitialized = false;
}
void TurnController::begin(Motor* left,Motor* right,MPU* sensor){
    leftMotor = left;
    rightMotor = right;
    imu = sensor;
    pid.begin();
    pid.setOutputLimits(-maximumPWM,maximumPWM);
}
bool TurnController::turnTo(float targetAngle){
    imu->update();
    float current = imu->getHeading();
    float error = normalize(targetAngle-current);
    if(abs(error)<=tolerance){
        stop();
        pid.reset();
        return true;
    }
    float correction =pid.compute(targetAngle,current,0.01f);
    if(correction>0){
        correction=max(correction,(float)minimumPWM);
    }
    else{
        correction=min(correction,(float)-minimumPWM);  
    }
    leftMotor->setSpeed(-(int)correction);
    rightMotor->setSpeed((int)correction);
    return false;
}

bool TurnController::turnLeft90(){
    if(!targetInitialized){
        targetHeading =normalize(imu->getHeading() - 90.0f);
        targetInitialized = true;
    }
    bool finished = turnTo(targetHeading);
    if(finished){
        targetInitialized = false;
    }
    return finished;
}   
bool TurnController::turnRight90(){
    if(!targetInitialized){
        targetHeading =normalize(imu->getHeading() + 90.0f);
        targetInitialized = true;
    }
    bool finished = turnTo(targetHeading);
    if(finished){
        targetInitialized = false;
    }
    return finished;
}
bool TurnController::turnAround(){
    if(!targetInitialized){
        targetHeading =normalize(imu->getHeading() + 180.0f);
        targetInitialized = true;
    }
    bool finished = turnTo(targetHeading);
    if(finished){
        targetInitialized = false;
    }
    return finished;
}
void TurnController::stop(){
    leftMotor->stop();
    rightMotor->stop();
}

float TurnController::normalize(float angle){
    while(angle>=180.0f)
        angle-=360.0f;
    while(angle<-180.0f)
        angle+=360.0f;
    return angle;
}
