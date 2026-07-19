#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "ServoScan.h"
#include "Maze.h"
#include "Config.h"

class SensorFusion{
private:
    ServoScanner* scanner;

public:
    void begin(ServoScanner* s);
    void scanWalls(int row,int col,Direction heading);
};

#endif