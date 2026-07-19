/*
The TOF module is the perception system of your maze solver robot. Its purpose is to accurately 
measure the distance to surrounding walls using the VL53L0X Time-of-Flight sensor, providing reliable 
environmental information that allows the robot to detect obstacles, construct an internal map of the maze, 
and make intelligent navigation decisions.
*/

#include "TOF.h"

TOFSensor::TOFSensor(){
    initialized = false;
    lastDistance = 8190;
    sampleIndex = 0;
    for(uint8_t i = 0; i < FILTER_SIZE; i++){
        samples[i] = 8190;
    }
}

bool TOFSensor::begin(){
    Wire.begin();
    Wire.setClock(400000);
    if(!sensor.init()){
        initialized = false;
        return false;
    }
    sensor.setTimeout(50);
    sensor.setMeasurementTimingBudget(30000);
    sensor.startContinuous(20);
    initialized = true;
    return true;
}
void TOFSensor::update(){
    if(!initialized)
        return;
    uint16_t d = sensor.readRangeContinuousMillimeters();
    if(sensor.timeoutOccurred())
        return;
    lastDistance = d;
    samples[sampleIndex] = d;
    sampleIndex++;
    if(sampleIndex >= FILTER_SIZE)
        sampleIndex = 0;
}

uint16_t TOFSensor::getDistance(){
    return lastDistance;
}

bool TOFSensor::isTimeout(){
    if(!initialized)
        return true;
    return sensor.timeoutOccurred();
}
uint16_t TOFSensor::medianFilter(){
    uint16_t temp[FILTER_SIZE];
    for(uint8_t i = 0; i < FILTER_SIZE; i++){
        temp[i] = samples[i];
    }
    for(uint8_t i = 0; i < FILTER_SIZE - 1; i++){
        for(uint8_t j = i + 1; j < FILTER_SIZE; j++){
            if(temp[j] < temp[i]){
                uint16_t t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    return temp[FILTER_SIZE / 2];
}
uint16_t TOFSensor::getFilteredDistance(){
    return medianFilter();
}
