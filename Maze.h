#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>
#define MAZE_SIZE 8

enum Direction{NORTH = 0,EAST,SOUTH,WEST};

struct Cell{
    bool wall[4];      // N,E,S,W
    bool visited;
    uint8_t flood;
    Cell(){
        visited = false;
        flood = 255;
        for(int i=0;i<4;i++)
            wall[i]=false;
    }
};

extern Cell maze[MAZE_SIZE][MAZE_SIZE];

void initializeMaze();
void clearMaze();
void initializeFlood();
void setVisited(int row,int col);
bool isVisited(int row,int col);
void setWall(int row,int col,Direction dir);
bool hasWall(int row,int col,Direction dir);
bool isInsideMaze(int row,int col);
void printFlood();
void printWalls();

#endif
