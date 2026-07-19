/*
The MotionController is the robot's movement engine: it takes movement commands from the Navigation module
and uses the motors, encoders, MPU6050, PID controller, and TurnController to execute precise, reliable movements 
from one maze cell to the next while maintaining accuracy.
*/
#include "MotionController.h"

MotionController motion;

MotionController::MotionController() :drivePID(KP, KI, KD){
    leftMotor = nullptr;
    rightMotor = nullptr;
    leftEncoder = nullptr;
    rightEncoder = nullptr;
    imu = nullptr;
    targetHeading = 0.0f;
    baseSpeed = SPEED_NORMAL;
    minimumPWM = SPEED_SLOW;
    maximumPWM = SPEED_FAST;
    cellDistance = CELL_SIZE_MM;
    initialized = false;
}

void MotionController::begin(Motor* left,Motor* right,Encoder* leftEnc,Encoder* rightEnc,MPU* sensor){
    leftMotor = left;
    rightMotor = right;
    leftEncoder = leftEnc;
    rightEncoder = rightEnc;
    imu = sensor;
    turnController.begin(leftMotor,rightMotor,imu);
    drivePID.begin();
    drivePID.setOutputLimits(-80,80);
    targetHeading = imu->getHeading();
    initialized = true;
}
void MotionController::resetEncoders(){
    if(leftEncoder){
        leftEncoder->resetTicks();
    }
    if(rightEncoder){
        rightEncoder->resetTicks();
    }
}
float MotionController::getAverageDistance(){
    if(leftEncoder == nullptr || rightEncoder == nullptr){
        return 0.0f;
    }
    float left =leftEncoder->getDistanceMM();
    float right =rightEncoder->getDistanceMM();
    return (left + right) * 0.5f;
}
void MotionController::stop(){
    if(leftMotor){
        leftMotor->stop();
    }
    if(rightMotor){
        rightMotor->stop();
    }
    drivePID.reset();
}
void MotionController::setBaseSpeed(int speed){
    speed = constrain(speed,minimumPWM,maximumPWM);
    baseSpeed = speed;
}
int MotionController::getBaseSpeed() const{
    return baseSpeed;
}
void MotionController::driveStraight(float correction){
    int leftPWM = baseSpeed - (int)correction;
    int rightPWM = baseSpeed + (int)correction;
    leftPWM = constrain(leftPWM,0,255);
    rightPWM = constrain(rightPWM,0,255);
    leftMotor->setSpeed(leftPWM);
    rightMotor->setSpeed(rightPWM);
}
bool MotionController::driveDistance(float distanceMM){
    if(!initialized)
        return true;
    static bool started = false;
    if(!started){
        resetEncoders();
        imu->update();
        targetHeading = imu->getHeading();
        drivePID.reset();
        started = true;
    }
    imu->update();
    float travelled =getAverageDistance();
    if(travelled >= distanceMM){
        stop();
        started = false;
        return true;
    }
    float currentHeading = imu->getHeading();
    float correction = drivePID.compute(targetHeading,currentHeading,0.01f);
    driveStraight(correction);
    return false;
}
bool MotionController::driveOneCell(){
    return driveDistance(cellDistance);
}
bool MotionController::turnLeft90(){
    if(!initialized)
        return true;
    return turnController.turnLeft90();
}
bool MotionController::turnRight90(){
    if(!initialized)
        return true;
    return turnController.turnRight90();
}
bool MotionController::turnAround(){
    if(!initialized)
        return true;
    return turnController.turnAround();
}