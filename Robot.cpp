/*
The Robot class is the central brain and coordinator of  ESP32 maze solver robot. 
It maintains the robot's current position,orientation, sensor information, and operating state, 
while orchestrating the interaction between all hardware modules (motors, encoders, sensors) and 
software modules (PID, maze mapping, flood fill, and navigation) so the robot can continuously 
sense its environment, make intelligent decisions, and navigate through the maze autonomously.
*/

//Include all the robot's subsystems.
#include "Robot.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "MPU.h"
#include "TOF.h"
#include "ServoScan.h"
#include "Maze.h"
#include "FloodFill.h"
#include "Navigation.h"

//constructor
Robot::Robot(){
    row = 0;
    col = 0;
    heading = NORTH;
    state = IDLE;
    leftDistance = 0;
    frontDistance = 0;
    rightDistance = 0;
    initialized = false;
}

//initialize all subsystems
void Robot::begin() {
    Serial.begin(115200);
    leftMotor->begin();
    rightMotor->begin();
    leftEncoder->begin();
    rightEncoder->begin();
    imu->begin();
    scanner->begin();
    floodFill.initialize();
    initialized = true;
    Serial.println("Robot Initialized");
}

//checks the current state and runs the appropriate action.
void Robot::update(){
    if (!initialized)
        return;
    switch (state){
        case IDLE:
            break;
        case SCAN:
            scanWalls();
            break;
        case FLOOD:
            computeFlood();
            break;
        case DECIDE:
            decideNextMove();
            break;
        case MOVE:
            moveForward();
            break;
        case TURN_LEFT:
            turnLeft();
            break;
        case TURN_RIGHT:
            turnRight();
            break;
        case TURN_BACK:
            turnBack();
            break;
        case SPEED_RUN:
            break;
    }
}
void Robot::setPosition(int r, int c){
    row = r;
    col = c;
}
int Robot::getRow() const{
    return row;
}
int Robot::getCol() const{
    return col;
}
void Robot::setHeading(Direction dir){
    heading = dir;
}
Direction Robot::getHeading() const{
    return heading;
}
void Robot::setDistances(int left,int front,int right){
    leftDistance = left;
    frontDistance = front;
    rightDistance = right;
}
int Robot::getLeftDistance() const{
    return leftDistance;
}
int Robot::getFrontDistance() const{
    return frontDistance;
}
int Robot::getRightDistance() const{
    return rightDistance;
}
void Robot::setState(RobotState s){
    state = s;
}

RobotState Robot::getState() const{
    return state;
}

void Robot::moveForward(){
    moveOneCell();
    switch (heading){
        case NORTH:
            row--;
            break;
        case EAST:
            col++;
            break;
        case SOUTH:
            row++;
            break;
        case WEST:
            col--;
            break;
    }
    state = SCAN; //After moving into a new cell, the next task is to scan the surroundings.
}

void Robot::turnLeft(){
    turnLeft90();
    heading = (Direction)((heading + 3) % 4); //update the heading
    state = MOVE; //After turning, the robot is ready to move forward.
}
void Robot::turnRight(){
    turnRight90();
    heading = (Direction)((heading + 1) % 4); 
    state = MOVE; 
}
void Robot::turnBack(){
    turnAround();
    heading = (Direction)((heading + 2) % 4);
    state = MOVE;
}
void Robot::scanWalls(){
    lookLeft();
    leftDistance = getFilteredDistance();
    lookCenter();
    frontDistance = getFilteredDistance();
    lookRight();
    rightDistance = getFilteredDistance();
    lookCenter();
    updateMaze(); 
    state = FLOOD; //Once the robot knows where the walls are, it can update its path-planning information.
}

void Robot::updateMaze(){
    Cell &cell = maze[row][col];
    // Mark this cell as visited
    cell.visited = true;
    bool leftWall  = (leftDistance  < WALL_DISTANCE_MM);
    bool frontWall = (frontDistance < WALL_DISTANCE_MM);
    bool rightWall = (rightDistance < WALL_DISTANCE_MM);
    switch (heading){
        case NORTH:
            cell.wall[WEST]  = leftWall;
            cell.wall[NORTH] = frontWall;
            cell.wall[EAST]  = rightWall;
            break;
        case EAST:
            cell.wall[NORTH] = leftWall;
            cell.wall[EAST]  = frontWall;
            cell.wall[SOUTH] = rightWall;
            break;
        case SOUTH:
            cell.wall[EAST]  = leftWall;
            cell.wall[SOUTH] = frontWall;
            cell.wall[WEST]  = rightWall;
            break;
        case WEST:
            cell.wall[SOUTH] = leftWall;
            cell.wall[WEST]  = frontWall;
            cell.wall[NORTH] = rightWall;
            break;
    }
}
void Robot::computeFlood(){
    floodFill();  //Runs the Flood Fill algorithm to assign a cost to every reachable cell
    state = DECIDE; //Once the cost map is ready, the robot chooses its next move.
}
void Robot::decideNextMove(){
    Direction bestDir = floodFill.getBestDirection(row, col);
    if (bestDir == heading){
        state = MOVE;
    }
    else if (bestDir == leftOf(heading)){
        state = TURN_LEFT;
    }
    else if (bestDir == rightOf(heading)){
        state = TURN_RIGHT;
    }
    else{
        state = TURN_BACK;
    }
}

void Robot::printStatus()
{
    Serial.println("------------------------");
    Serial.print("Position : ");
    Serial.print(row);
    Serial.print(",");
    Serial.println(col);

    Serial.print("Heading  : ");
    Serial.println((int)heading);

    Serial.print("Left     : ");
    Serial.println(leftDistance);
    Serial.print("Front    : ");
    Serial.println(frontDistance);
    Serial.print("Right    : ");
    Serial.println(rightDistance);

    Serial.println("------------------------");
}
