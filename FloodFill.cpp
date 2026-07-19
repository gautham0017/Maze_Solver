/*
the Flood Fill module is the path-planning algorithm that converts the robot's discovered maze map 
into an optimal route toward the goal. Using Breadth-First Search (BFS), it propagates distance values
from the goal through every reachable cell, creating a gradient that always points toward the shortest known path. 
As the robot discovers new walls, these flood values are recalculated, allowing the robot to dynamically adapt its route, 
avoid dead ends, and efficiently navigate to the maze center.
*/
#include "FloodFill.h"

FloodFill floodFill;
static const int dr[4] ={-1,0,1,0};
static const int dc[4] ={0,1,0,-1};

FloodFill::FloodFill(){
    clearQueue();
}

void FloodFill::initialize(){
    initializeMaze();
    initializeFlood();
}
void FloodFill::clearQueue(){
    front = 0;
    rear = 0;
}
bool FloodFill::isQueueEmpty(){
    return front == rear;
}
void FloodFill::enqueue(uint8_t row,uint8_t col){
    if(rear >= MAZE_SIZE * MAZE_SIZE)
        return;
    queue[rear].row = row;
    queue[rear].col = col;
    rear++;
}
QueueNode FloodFill::dequeue(){
    QueueNode node = queue[front];
    front++;
    return node;
}
bool FloodFill::isValidCell(int row,int col){
    return row >= 0 &&row < MAZE_SIZE &&col >= 0 &&col < MAZE_SIZE;
}
bool FloodFill::canMove(int row,int col,Direction dir){
    if(!isValidCell(row,col))
        return false;
    return !maze[row][col].wall[dir];
}

void FloodFill::updateFloodValues(){
    for(int r = 0; r < MAZE_SIZE; r++){
        for(int c = 0; c < MAZE_SIZE; c++){
            maze[r][c].flood = 255;
        }
    }
    clearQueue();

    const uint8_t goalRow1 = MAZE_SIZE/2 - 1;
    const uint8_t goalRow2 = MAZE_SIZE/2;
    const uint8_t goalCol1 = MAZE_SIZE/2 - 1;
    const uint8_t goalCol2 = MAZE_SIZE/2;

    maze[goalRow1][goalCol1].flood = 0;
    maze[goalRow1][goalCol2].flood = 0;
    maze[goalRow2][goalCol1].flood = 0;
    maze[goalRow2][goalCol2].flood = 0;

    enqueue(goalRow1,goalCol1);
    enqueue(goalRow1,goalCol2);
    enqueue(goalRow2,goalCol1);
    enqueue(goalRow2,goalCol2);

    while(!isQueueEmpty()){
        QueueNode current = dequeue();
        uint8_t row = current.row;
        uint8_t col = current.col;
        uint8_t currentFlood = maze[row][col].flood;
        for(int dir = 0; dir < 4; dir++){
            if(!canMove(row,col,(Direction)dir)){
                continue;
            }
            int nr = row + dr[dir];
            int nc = col + dc[dir];
            if(!isValidCell(nr,nc))
                continue;
            if(maze[nr][nc].flood >currentFlood + 1){
                maze[nr][nc].flood =currentFlood + 1;
                enqueue(nr,nc);
            }
        }
    }
}

void FloodFill::printFloodValues(){
    printFlood();
}
Direction FloodFill::getBestDirection(int row,int col){
    uint8_t bestFlood = maze[row][col].flood;
    Direction bestDirection = NORTH;
    bool found = false;
    for(int dir = 0; dir < 4; dir++){
        if(!canMove(row, col, (Direction)dir))
            continue;
        int nr = row + dr[dir];
        int nc = col + dc[dir];
        if(!isValidCell(nr, nc))
            continue;
        if(maze[nr][nc].flood < bestFlood){
            bestFlood = maze[nr][nc].flood;
            bestDirection = (Direction)dir;
            found = true;
        }
    }
    if(found)
        return bestDirection;
    return NORTH;
}
bool FloodFill::reachedGoal(int row,int col){
    int center1 = MAZE_SIZE / 2 - 1;
    int center2 = MAZE_SIZE / 2;
    return (row == center1 && col == center1) || (row == center1 && col == center2) ||(row == center2 && col == center1) || (row == center2 && col == center2);
}