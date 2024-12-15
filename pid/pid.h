/**************************************************************
* Class    : CSC-615-01 Fall 2024
* Name     : Nhan Nguyen
* Student ID: 923100929
* Github   : nhannguyensf
* Project  : Final Assignment - PID Control for Line Tracking
*
* File     : pid.h
*
* Description:
*   This header file defines the constants, function prototypes,
*   and shared variables for implementing PID control for a car
*   with 5 line sensors.
**************************************************************/

#include "line_sensor.h" // Provides NUM_SENSORS definition

#ifndef PID_H
#define PID_H

#define KP 35.0f
#define KI 0.5f
#define KD 0.0f

// Motor speed limits
#define MAX_SPEED 100
#define MIN_SPEED 0

// Shared variables for sensor and PID values
extern int sensor_positions[NUM_SENSORS];  // Positions of line sensors
extern float pid_error;                    // PID calculated error
extern int motor_left_speed;               // Speed for left motor
extern int motor_right_speed;              // Speed for right motor

// Function Prototypes
void pid_init();                        // Initialize PID variables
float pid_compute(int* sensor_states);  // Compute the PID output
void adjust_motor_speed(float error);   // Adjust motor speeds based on PID error
void print_debug_info(int* sensor_states, float error); // Print debug information

#endif // PID_H
