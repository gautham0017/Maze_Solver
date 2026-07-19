#include "Config.h"
#include "Robot.h"

Robot robot;

void setup(){
    robot.begin(); //Turn on every subsystem.

    // Starting position
    robot.setPosition(0, 0);

    // Robot initially faces North
    robot.setHeading(NORTH);

    // Start with wall scanning
    robot.setState(SCAN);

    Serial.println();
    Serial.println(" ESP32 Maze Solver Robot ");
}

// Main Loop
void loop(){
    robot.update();  //Checks the current robot state. Executes the corresponding action. Changes the state if needed.
}
