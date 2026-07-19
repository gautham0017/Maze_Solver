/*
The Maze module serves as the central environmental memory. It maintains a complete digital representation 
of the maze by storing wall locations, visited status, and flood-fill values for every cell. As the robot explores, 
Sensor Fusion updates this map with newly discovered walls, while the Flood Fill algorithm reads it to compute 
the shortest path to the goal. Because all navigation decisions depend on the accuracy of this map, the Maze module
forms the foundation upon which the robot's perception, planning, and movement systems operate.
*/

#include "Maze.h"

Cell maze[MAZE_SIZE][MAZE_SIZE];
const int dr[4]={-1,0,1,0};
const int dc[4]={ 0,1,0,-1};

bool isInsideMaze(int row,int col){
    return row>=0 && row<MAZE_SIZE && col>=0 && col<MAZE_SIZE;
}
void clearMaze(){
    for(int r=0;r<MAZE_SIZE;r++){
        for(int c=0;c<MAZE_SIZE;c++){
            maze[r][c].visited=false;
            maze[r][c].flood=255;
            for(int i=0;i<4;i++){
                maze[r][c].wall[i]=false;
            }    
        }
    }
}
void initializeMaze(){
    clearMaze();
    for(int i=0;i<MAZE_SIZE;i++){
        maze[0][i].wall[NORTH]=true;
        maze[MAZE_SIZE-1][i].wall[SOUTH]=true;
        maze[i][0].wall[WEST]=true;
        maze[i][MAZE_SIZE-1].wall[EAST]=true;
    }
}
void initializeFlood(){
    uint8_t goal1=MAZE_SIZE/2-1;
    uint8_t goal2=MAZE_SIZE/2;
    for(int r=0;r<MAZE_SIZE;r++){
        for(int c=0;c<MAZE_SIZE;c++){
            uint8_t d1=abs(r-goal1)+abs(c-goal1);
            uint8_t d2=abs(r-goal1)+abs(c-goal2);
            uint8_t d3=abs(r-goal2)+abs(c-goal1);
            uint8_t d4=abs(r-goal2)+abs(c-goal2);
            maze[r][c].flood=min(min(d1,d2),min(d3,d4));
        }
    }
}
void setVisited(int row,int col){
    if(isInsideMaze(row,col)){
        maze[row][col].visited=true;
    }
}
bool isVisited(int row,int col){
    if(!isInsideMaze(row,col))
        return false;
    return maze[row][col].visited;
}
void setWall(int row,int col,Direction dir){
    if(!isInsideMaze(row,col)){
        return;
    }
    maze[row][col].wall[dir]=true;
    int nr=row+dr[dir];
    int nc=col+dc[dir];
    if(!isInsideMaze(nr,nc)){
        return;
    }
    Direction opposite=(Direction)((dir+2)%4);
    maze[nr][nc].wall[opposite]=true;
}
bool hasWall(int row,int col,Direction dir){
    if(!isInsideMaze(row,col)){
        return true;
    }
    return maze[row][col].wall[dir];
}
void printFlood(){
    Serial.println();
    for(int r=0;r<MAZE_SIZE;r++){
        for(int c=0;c<MAZE_SIZE;c++){
            Serial.print(maze[r][c].flood);
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}
void printWalls(){
    Serial.println();
    for(int r=0;r<MAZE_SIZE;r++){
        for(int c=0;c<MAZE_SIZE;c++){
            Serial.print("[");
            Serial.print(maze[r][c].wall[NORTH]);
            Serial.print(",");
            Serial.print(maze[r][c].wall[EAST]);
            Serial.print(",");
            Serial.print(maze[r][c].wall[SOUTH]);
            Serial.print(",");
            Serial.print(maze[r][c].wall[WEST]);
            Serial.print("] ");
        }
        Serial.println();
    }
    Serial.println();
}