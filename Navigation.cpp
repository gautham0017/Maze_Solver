/*
For the ESP32-based maze-solving robot, the Navigation module serves as the central mission controller. 
It keeps track of the robot's position, heading, and current mission phase while coordinating Sensor Fusion, 
the Maze module, Flood Fill, and the Motion Controller. By repeatedly scanning the environment, updating the maze, 
selecting the best direction, and commanding the required movements, it enables the robot to autonomously explore the maze, 
reach the goal, return home, and prepare for an optimized fast run.
*/
#include "Navigation.h"

Navigation navigation;

Navigation::Navigation(){
    robot.row = MAZE_SIZE - 1;
    robot.col = 0;
    robot.heading = HEADING_NORTH;
    state = NAV_EXPLORE;
    motion = nullptr;
    sensors = nullptr;
    flood = nullptr;
}
void Navigation::begin(MotionController* motionController,SensorFusion* sensorFusion,FloodFill* floodFill){
    motion = motionController;
    sensors = sensorFusion;
    flood = floodFill;
    robot.row = MAZE_SIZE - 1;
    robot.col = 0;
    robot.heading = HEADING_NORTH;
    state = NAV_EXPLORE;
}

RobotState Navigation::getRobotState() const{
    return robot;
}
NavigationState Navigation::getNavigationState() const{
    return state;
}
void Navigation::setPosition(uint8_t row,uint8_t col){
    robot.row = row;
    robot.col = col;
}
void Navigation::setHeading(RobotHeading heading){
    robot.heading = heading;
}

bool Navigation::goalReached() const{
    return flood->reachedGoal(robot.row,robot.col);
}
bool Navigation::missionFinished() const{
    return state == NAV_FINISHED;
}

Direction Navigation::leftOf(Direction dir){
    switch(dir){
        case NORTH: return WEST;
        case EAST:  return NORTH;
        case SOUTH: return EAST;
        case WEST:  return SOUTH;
    }
    return NORTH;
}
Direction Navigation::rightOf(Direction dir){
    switch(dir){
        case NORTH: return EAST;
        case EAST:  return SOUTH;
        case SOUTH: return WEST;
        case WEST:  return NORTH;
    }
    return NORTH;
}
Direction Navigation::oppositeOf(Direction dir){
    switch(dir){
        case NORTH: return SOUTH;
        case EAST:  return WEST;
        case SOUTH: return NORTH;
        case WEST:  return EAST;
    }
    return NORTH;
}

int Navigation::directionDifference(Direction current,Direction target){
    int diff = (target - current + 4) % 4;
    return diff;
}

void Navigation::updateHeading(Direction dir){
    robot.heading = (RobotHeading)dir;
}

void Navigation::updatePosition(){
    switch(robot.heading){
        case HEADING_NORTH:
            if(robot.row > 0)
                robot.row--;
            break;
        case HEADING_EAST:
            if(robot.col < MAZE_SIZE - 1)
                robot.col++;
            break;
        case HEADING_SOUTH:
            if(robot.row < MAZE_SIZE - 1)
                robot.row++;
            break;
        case HEADING_WEST:
            if(robot.col > 0)
                robot.col--;
            break;
    }
}

void Navigation::rotateTo(Direction target){
    if((Direction)robot.heading == target)
        return;
    int diff = directionDifference((Direction)robot.heading,target);
    switch(diff){
        case 0:
            break;
        case 1:
            while(!motion->turnRight90()){
                delay(5);
            }
            updateHeading(target);
            break;
        case 2:
            while(!motion->turnAround()){
                delay(5);
            }
            updateHeading(target);
            break;
        case 3:
            while(!motion->turnLeft90()){
                delay(5);
            }
            updateHeading(target);
            break;
    }
    motion->stop();
}

void Navigation::moveOneCell(){
    while(!motion->driveOneCell()){
        delay(5);
    }
    motion->stop();
    updatePosition();
}
void Navigation::printRobotState(){
    Serial.println();
    Serial.println("========== ROBOT ==========");
    Serial.print("Row : ");
    Serial.println(robot.row);
    Serial.print("Col : ");
    Serial.println(robot.col);
    Serial.print("Heading : ");

    switch(robot.heading){
        case HEADING_NORTH:
            Serial.println("NORTH");
            break;
        case HEADING_EAST:
            Serial.println("EAST");
            break;
        case HEADING_SOUTH:
            Serial.println("SOUTH");
            break;
        case HEADING_WEST:
            Serial.println("WEST");
            break;
    }
    Serial.print("State : ");

    switch(state){
        case NAV_EXPLORE:
            Serial.println("EXPLORE");
            break;
        case NAV_RETURN_HOME:
            Serial.println("RETURN HOME");
            break;
        case NAV_FAST_RUN_READY:
            Serial.println("FAST RUN READY");
            break;
        case NAV_FAST_RUN:
            Serial.println("FAST RUN");
            break;
        case NAV_FINISHED:
            Serial.println("FINISHED");
            break;
    }
    Serial.println();
}

void Navigation::exploreStep(){
    sensors->scanWalls(robot.row,robot.col,(Direction)robot.heading);
    flood->updateFloodValues();
#ifdef DEBUG_FLOOD
    flood->printFloodValues();
#endif-

    if(goalReached()){
        Serial.println("Center Reached");
        state = NAV_RETURN_HOME;
        return;
    }
    Direction nextDirection =flood->getBestDirection(robot.row,robot.col);
    rotateTo(nextDirection);
    moveOneCell();
#ifdef DEBUG_NAVIGATION
    printRobotState();
#endif
}
void Navigation::returnStep(){
    sensors->scanWalls(robot.row,robot.col,(Direction)robot.heading);
    flood->updateFloodValues();
    if(robot.row == MAZE_SIZE - 1 && robot.col == 0){
        Serial.println("Returned Home");
        state = NAV_FAST_RUN_READY;
        return;
    }
    Direction nextDirection =flood->getBestDirection(robot.row,robot.col);
    rotateTo(nextDirection);
    moveOneCell();
#ifdef DEBUG_NAVIGATION
    printRobotState();
#endif
}

void Navigation::fastRunStep(){
    Serial.println("Fast Run Mode");
    state = NAV_FINISHED;
}
void Navigation::update(){
    switch(state){
        case NAV_EXPLORE:
            exploreStep();
            break;
        case NAV_RETURN_HOME:
            returnStep();
            break;
        case NAV_FAST_RUN_READY:
            Serial.println("Preparing Fast Run");
            state = NAV_FAST_RUN;
            break;
        case NAV_FAST_RUN:
            fastRunStep();
            break;
        case NAV_FINISHED:
            motion->stop();
            break;
    }
}
