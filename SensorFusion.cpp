/*
The SensorFusion module is the environment interpretation system of the maze solver robot. 
Its purpose is to combine the distance measurements from the Servo Scanner with the robot's current position 
and heading, identify where walls exist in the maze, and update the robot's internal map so that the navigation 
and Flood Fill algorithms always operate on an accurate representation of the environment.
*/

#include "SensorFusion.h"

void SensorFusion::begin(ServoScanner* s){
    scanner = s;
}
void SensorFusion::scanWalls(int row,int col,Direction heading){
    ScanResult data = scanner->scan();
    bool leftWall  = data.left  < WALL_DISTANCE_MM;
    bool frontWall = data.front < WALL_DISTANCE_MM;
    bool rightWall = data.right < WALL_DISTANCE_MM;
    setVisited(row,col);

    switch(heading){
        case NORTH:
            if(frontWall)
                setWall(row,col,NORTH);
            if(leftWall)
                setWall(row,col,WEST);
            if(rightWall)
                setWall(row,col,EAST);
            break;

        case EAST:
            if(frontWall)
                setWall(row,col,EAST);
            if(leftWall)
                setWall(row,col,NORTH);
            if(rightWall)
                setWall(row,col,SOUTH);
            break;

        case SOUTH:
            if(frontWall)
                setWall(row,col,SOUTH);
            if(leftWall)
                setWall(row,col,EAST);
            if(rightWall)
                setWall(row,col,WEST);
            break;

        case WEST:
            if(frontWall)
                setWall(row,col,WEST);
            if(leftWall)
                setWall(row,col,SOUTH);
            if(rightWall)
                setWall(row,col,NORTH);
            break;
    }
}
