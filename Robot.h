#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Config.h"

class Robot{
private:

    // Robot Position
    int row;
    int col;

    // Current heading
    Direction heading;

    // Current robot state
    RobotState state;

    // Latest sensor readings
    int leftDistance;
    int frontDistance;
    int rightDistance;

    // Initialization flag
    bool initialized;

public:

    Robot();

    // System
    void begin();  //Starts every hardware component.
    void update(); //Runs one iteration of the robot's state machine.

    // Position
    void setPosition(int r, int c);

    int getRow() const; //The const keyword means this function promises not to modify the object's state.
    int getCol() const;

    // Heading
    void setHeading(Direction dir);
    Direction getHeading() const;  //Returns current direction.

    // Sensor Data
    void setDistances(int left,int front,int right); //Stores latest sensor readings.
    int getLeftDistance() const;
    int getFrontDistance() const;
    int getRightDistance() const;

    // State
    void setState(RobotState s);   //Changes robot state.
    RobotState getState() const;  //Returns current state.

    // Navigation
    void moveForward();
    void turnLeft();
    void turnRight();
    void turnBack();

    // Maze
    void scanWalls();
    void updateMaze();

    // Flood Fill
    void computeFlood();
    void decideNextMove();

    // Debug
    void printStatus();
};

#endif
