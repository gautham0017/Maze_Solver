/*
The ServoScanner module is the active environmental scanning system of maze solver robot. 
Its purpose is to precisely position the ToF sensor toward the left, front, and right directions, 
collect stable distance measurements from each viewpoint, and provide a complete local view of the maze
 so that the robot can accurately detect walls, update its internal map, and make intelligent navigation decisions.
 */

#include "ServoScan.h"

#define SERVO_DELAY     250
#define SAMPLE_COUNT    5

ServoScanner::ServoScanner(){
    tof = nullptr;
    servoPin = 0;
    leftAngle = 150;
    centerAngle = 90;
    rightAngle = 30;
}

bool ServoScanner::begin(uint8_t pin,TOFSensor *sensor){
    servoPin = pin;
    tof = sensor;
    servo.setPeriodHertz(50);
    if(!servo.attach(servoPin))
        return false;
    lookCenter();
    delay(500);
    return true;
}
void ServoScanner::moveServo(uint8_t angle){
    servo.write(angle);
    delay(SERVO_DELAY);
}
void ServoScanner::lookLeft(){
    moveServo(leftAngle);
}
void ServoScanner::lookCenter(){
    moveServo(centerAngle);
}
void ServoScanner::lookRight(){
    moveServo(rightAngle);
}

uint16_t ServoScanner::readStableDistance(){
    uint32_t sum = 0;
    for(int i = 0; i < SAMPLE_COUNT; i++){
        tof->update();
        sum += tof->getFilteredDistance();
        delay(20);
    }
    return sum / SAMPLE_COUNT;
}
ScanResult ServoScanner::scan(){
    ScanResult result;
    lookLeft();
    result.left = readStableDistance();
    lookCenter();
    result.front = readStableDistance();
    lookRight();
    result.right = readStableDistance();
    lookCenter();
    return result;
}