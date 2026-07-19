#ifndef SERVOSCAN_H
#define SERVOSCAN_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "TOF.h"

struct ScanResult{
    uint16_t left;
    uint16_t front;
    uint16_t right;
};

class ServoScanner{
private:
    Servo servo;
    TOFSensor *tof;
    uint8_t servoPin;
    uint8_t leftAngle;
    uint8_t centerAngle;
    uint8_t rightAngle;
    uint16_t readStableDistance();
    void moveServo(uint8_t angle);

public:
    ServoScanner();
    bool begin(uint8_t pin,TOFSensor *sensor);
    void lookLeft();
    void lookCenter();
    void lookRight();
    ScanResult scan();
};

#endif
