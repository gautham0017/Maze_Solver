/*
The Encoder module is the motion feedback system of ESP32 maze solver robot.
Its purpose is to continuously measure the rotation of each wheel using quadrature Hall-effect encoders 
and convert that information into meaningful data—such as distance travelled, speed, RPM, and direction
so the robot can accurately control its movement, maintain precise positioning, support PID-based motor regulation
and navigate the maze reliably
*/

#include "Encoder.h"
#include "Config.h"
#include <math.h>

static Encoder* leftInstance = nullptr; 
static Encoder* rightInstance = nullptr;

Encoder leftEncoder(LEFT_ENC_A,LEFT_ENC_B,WHEEL_DIAMETER_MM,TICKS_PER_REV);
Encoder rightEncoder(RIGHT_ENC_A,RIGHT_ENC_B,WHEEL_DIAMETER_MM,TICKS_PER_REV);

Encoder::Encoder(){
    pinA = 0;
    pinB = 0;
    ticks = 0;
    previousState = 0;
    previousTime = 0;
    previousTicks = 0;
    wheelDiameter = 0.0f;
    ticksPerRevolution = 1;
}
Encoder::Encoder(uint8_t encoderA,uint8_t encoderB,float wheelDiaMM,long ticksPerRev){
    pinA = encoderA;
    pinB = encoderB;
    ticks = 0;
    previousState = 0;
    previousTime = millis();
    previousTicks = 0;
    wheelDiameter = wheelDiaMM;
    ticksPerRevolution = ticksPerRev;
}

void Encoder::begin(){
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    previousState =(digitalRead(pinA) << 1) | digitalRead(pinB);

    if(pinA == LEFT_ENC_A){
        leftInstance = this;
        attachInterrupt(digitalPinToInterrupt(pinA),leftEncoderISR,CHANGE);
        attachInterrupt(digitalPinToInterrupt(pinB),leftEncoderISR,CHANGE);
    }
    else{
        rightInstance = this;
        attachInterrupt(digitalPinToInterrupt(pinA),rightEncoderISR,CHANGE);
        attachInterrupt(digitalPinToInterrupt(pinB),rightEncoderISR,CHANGE);
    }
}

void Encoder::update(){
    uint8_t currentState =(digitalRead(pinA) << 1) | digitalRead(pinB);
    uint8_t transition =(previousState << 2) | currentState;
    switch(transition){
        // Forward
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            ticks++;
            break;
        // Reverse
        case 0b0010:
        case 0b0100:
        case 0b1101:
        case 0b1011:
            ticks--;
            break;
        default:
            break;
    }
    previousState = currentState;
}

void IRAM_ATTR leftEncoderISR(){
    if(leftInstance != nullptr){
        leftInstance->update();
    }
}
void IRAM_ATTR rightEncoderISR(){
    if(rightInstance != nullptr){
        rightInstance->update();
    }
}
long Encoder::getTicks(){
    noInterrupts();
    long value = ticks;
    interrupts();
    return value;
}
void Encoder::resetTicks(){
    noInterrupts();
    ticks = 0;
    previousTicks = 0;
    interrupts();
    previousTime = millis();
}
float Encoder::getDistanceMM(){
    long currentTicks = getTicks();
    float circumference = PI * wheelDiameter;
    return (currentTicks * circumference) / ticksPerRevolution;
}
float Encoder::getDistanceCM(){
    return getDistanceMM() / 10.0f;
}
float Encoder::getDistanceM(){
    return getDistanceMM() / 1000.0f;
}
float Encoder::getSpeedMMps(){
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - previousTime;
    if (deltaTime == 0)
        return 0.0f;
    long currentTicks = getTicks();
    long deltaTicks = currentTicks - previousTicks;
    float circumference = PI * wheelDiameter;
    float distance = (deltaTicks * circumference) / ticksPerRevolution;
    float speed = distance / (deltaTime / 1000.0f);
    previousTicks = currentTicks;
    previousTime = currentTime;
    return speed;
}
float Encoder::getSpeedCMps(){
    return getSpeedMMps() / 10.0f;
}
float Encoder::getRPM(){
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - previousTime;
    if (deltaTime == 0)
        return 0.0f;
    long currentTicks = getTicks();
    long deltaTicks = currentTicks - previousTicks;
    float revolutions = (float)deltaTicks / ticksPerRevolution;
    float minutes = deltaTime / 60000.0f;
    float rpm = revolutions / minutes;
    previousTicks = currentTicks;
    previousTime = currentTime;
    return rpm;
}
int Encoder::getDirection(){
    static long lastTicks = 0;
    long currentTicks = getTicks();
    int direction = 0;
    if (currentTicks > lastTicks){
        direction = 1;
    }
    else if (currentTicks < lastTicks){
        direction = -1;
    }
    else{
        direction = 0;
    }
    lastTicks = currentTicks;
    return direction;
}