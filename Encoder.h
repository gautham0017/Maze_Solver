#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
/*
✓ Tick counting
✓ Direction detection
✓ RPM calculation
✓ Wheel speed calculation
✓ Distance travelled
✓ Thread-safe reading
*/

class Encoder{
private:
    uint8_t pinA;
    uint8_t pinB;

    volatile long ticks;
    volatile uint8_t previousState;
    unsigned long previousTime;
    long previousTicks;
    float wheelDiameter;
    long ticksPerRevolution;

public:
    Encoder();
    Encoder(uint8_t encoderA,uint8_t encoderB,float wheelDiaMM,long ticksPerRev);

    void begin(); //Initializes the encoder hardware.
    void update(); //Called every time an interrupt occurs.This is the quadrature decoder.

    long getTicks();
    void resetTicks();

    float getDistanceMM();
    float getDistanceCM();
    float getDistanceM();

    float getSpeedMMps();
    float getSpeedCMps();

    float getRPM();

    int getDirection();
};

extern Encoder leftEncoder;
extern Encoder rightEncoder;

void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();
