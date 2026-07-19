/*
The MPU module is the orientation feedback system of the maze solver robot.
Its purpose is to continuously monitor the robot's rotational movement using the MPU6050 gyroscope, 
calculate its heading, and provide accurate orientation information so the robot can execute precise turns, 
maintain the correct direction, and navigate the maze reliably.
*/

#include "MPU.h"

#define MPU_PWR_MGMT_1   0x6B
#define MPU_GYRO_ZOUT_H  0x47
#define GYRO_SCALE       131.0f

MPU::MPU(){
    gyroBiasZ = 0.0f;
    heading = 0.0f;
    previousTime = 0;
}
bool MPU::begin(){
    Wire.begin();
    writeRegister(MPU_PWR_MGMT_1,0x00);
    delay(100);
    previousTime = micros();
    return true;
}
void MPU::calibrate(uint16_t samples){
    long sum = 0;
    Serial.println("Calibrating MPU6050...");
    for(uint16_t i=0;i<samples;i++){
        sum += read16(MPU_GYRO_ZOUT_H);
        delay(2);
    }
    gyroBiasZ = (float)sum / samples;
    Serial.print("Gyro Bias = ");
    Serial.println(gyroBiasZ);
}

void MPU::update(){
    unsigned long now = micros();
    float dt = (now - previousTime) / 1000000.0f;
    previousTime = now;
    int16_t rawGyro = read16(MPU_GYRO_ZOUT_H);
    float gyroRate =((float)rawGyro - gyroBiasZ) / GYRO_SCALE;
    heading += gyroRate * dt;
    heading = normalizeAngle(heading);
}
void MPU::resetHeading(){
    heading = 0.0f;
}
float MPU::getHeading(){
    return heading;
}
void MPU::writeRegister(uint8_t reg,uint8_t value){
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(); 
}

int16_t MPU::read16(uint8_t reg){
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR,(uint8_t)2);
    int16_t value = (Wire.read()<<8) | Wire.read();
    return value;
}

float MPU::normalizeAngle(float angle){
    while(angle >= 180.0f)
        angle -= 360.0f;
    while(angle < -180.0f)
        angle += 360.0f;
    return angle;
}