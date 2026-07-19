/*This file acts as the central configuration file forthe entire maze-solving robot. It groups together:
Maze parameters: maze size and cell dimensions.
Robot geometry: wheel diameter, wheel base, and derived wheel circumference.
Encoder configuration: pulses per revolution, gear ratio, and total ticks per wheel revolution.
Hardware pin assignments: motors, encoders, servo, and I²C.
Sensor parameters: wall detection threshold and ToF sampling count.
Motion settings: predefined PWM speed levels.
Control parameters: initial PID gains.
Servo timing: delay before taking measurements.
Enumerations: readable names for robot directions and operating states
*/

#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

//MAZE CONFIGURATION
#define MAZE_SIZE          8          
#define CELL_SIZE_MM       300.0f

//ROBOT DIMENSIONS
#define WHEEL_DIAMETER_MM      30.0f
#define WHEEL_BASE_MM          90.0f
#define WHEEL_CIRCUMFERENCE    (PI * WHEEL_DIAMETER_MM)

//ENCODER
#define ENCODER_PPR            3
#define GEAR_RATIO             100
#define TICKS_PER_REV          (ENCODER_PPR * GEAR_RATIO)

//MOTOR PINS
#define LEFT_IN1       26
#define LEFT_IN2       27
#define RIGHT_IN1      32
#define RIGHT_IN2      33

//ENCODER PINS
#define LEFT_ENC_A     34
#define LEFT_ENC_B     36
#define RIGHT_ENC_A    35
#define RIGHT_ENC_B    39

//SERVO
#define SERVO_PIN      18
#define SERVO_LEFT     150
#define SERVO_CENTER   90
#define SERVO_RIGHT    30

//I2C
#define SDA_PIN        21
#define SCL_PIN        22

//SENSOR
#define WALL_THRESHOLD     150      // mm

//MOTOR SPEEDS
#define SPEED_STOP     0
#define SPEED_SLOW     80
#define SPEED_NORMAL   150
#define SPEED_FAST     220


//PID DEFAULTS
#define KP     2.0f
#define KI     0.02f
#define KD     0.40f

//SERVO TIMING
#define SERVO_SETTLE_TIME     250

//TOF
#define TOF_SAMPLES           5

//DIRECTIONS
enum Direction
{
    NORTH = 0,
    EAST,
    SOUTH,
    WEST
};

//ROBOT STATES
enum RobotState
{
    IDLE,
    SCAN,
    FLOOD,
    DECIDE,
    MOVE,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_BACK,
    SPEED_RUN
};

#endif
