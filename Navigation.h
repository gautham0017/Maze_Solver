#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

#include "Config.h"
#include "Maze.h"
#include "FloodFill.h"
#include "MotionController.h"
#include "SensorFusion.h"

enum RobotHeading{
    HEADING_NORTH = NORTH,
    HEADING_EAST  = EAST,
    HEADING_SOUTH = SOUTH,
    HEADING_WEST  = WEST
};
enum NavigationState{
    NAV_EXPLORE,
    NAV_RETURN_HOME,
    NAV_FAST_RUN_READY,
    NAV_FAST_RUN,
    NAV_FINISHED
};
struct RobotState{
    uint8_t row;
    uint8_t col;
    RobotHeading heading;
};
class Navigation{
public:
    Navigation();

    void begin(MotionController* motion,SensorFusion* sensorFusion, FloodFill* floodFill);
    void update();
    RobotState getRobotState() const;
    NavigationState getNavigationState() const;
    void setPosition(uint8_t row,uint8_t col);
    void setHeading(RobotHeading heading);
    bool goalReached() const;
    bool missionFinished() const;

private:

    RobotState robot;
    NavigationState state;
    MotionController* motion;
    SensorFusion* sensors;
    FloodFill* flood;

    void exploreStep();
    void returnStep();
    void fastRunStep();
    void rotateTo(Direction target);
    void moveOneCell();
    void updateHeading(Direction dir);
    void updatePosition();

    Direction leftOf(Direction dir);
    Direction rightOf(Direction dir);
    Direction oppositeOf(Direction dir);
    int directionDifference(Direction current,Direction target);

    void printRobotState();
};



extern Navigation navigation;

#endif