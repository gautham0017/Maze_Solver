#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <Wire.h>

class MPU{
private:
    // I2C Address
    const uint8_t MPU_ADDR = 0x68;
    // Gyroscope Bias
    float gyroBiasZ;

    float heading;
    unsigned long previousTime;

public:
    MPU();
    bool begin();
    void calibrate(uint16_t samples = 1000);
    void update();
    void resetHeading();
    float getHeading();

private:
    void writeRegister(uint8_t reg,uint8_t value);
    int16_t read16(uint8_t reg);
    float normalizeAngle(float angle);
};

#endif  
