#ifndef TOF_H
#define TOF_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

class TOFSensor{
private:
    VL53L0X sensor;
    uint16_t lastDistance;
    bool initialized;

    // Median filter
    static const uint8_t FILTER_SIZE = 5;
    uint16_t samples[FILTER_SIZE];
    uint8_t sampleIndex;

public:
    TOFSensor();
    bool begin();
    void update();
    uint16_t getDistance();
    uint16_t getFilteredDistance();
    bool isTimeout();

private:
    uint16_t medianFilter();
};

#endif
