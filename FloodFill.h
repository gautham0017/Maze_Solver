#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <Arduino.h>
#include "Maze.h"

struct QueueNode{
    uint8_t row;
    uint8_t col;
};

class FloodFill{
private:
    QueueNode queue[MAZE_SIZE * MAZE_SIZE];
    int front;
    int rear;
    void clearQueue();
    bool isQueueEmpty();
    void enqueue(uint8_t row,uint8_t col);
    QueueNode dequeue();
    bool isValidCell(int row,int col);
    bool canMove(int row,int col,Direction dir);

public:
    FloodFill();
    void initialize();
    void updateFloodValues();
    Direction getBestDirection(int row,int col);
    bool reachedGoal(int row,int col);
    void printFloodValues();
};
extern FloodFill floodFill;

#endif
