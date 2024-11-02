/************************************************************** *
 * Class:: CSC-615-01 Fall 2024
 * Name:: Yuvraj Gupta
 * Student ID:: 922933190
 * Github-Name:: YuvrajGupta1808
 * Project: Assignment 3 - Start Your Motors
 * File: MotorDriver.h
 *
 * Description::
 * Header file for controlling a DC motor using the WaveShare
 * Motor Driver HAT on a Raspberry Pi. Defines constants and
 * function prototypes for motor direction and speed control
 * using PWM. This header also includes necessary dependencies
 * for motor control.
 * **************************************************************/
#ifndef __TB6612FNG_
#define __TB6612FNG_

// Including necessary dependencies for motor control
#include "DEV_Config.h"
#include "PCA9685.h"

// Defining GPIO pins and PWM channels for Motor A control
#define PWMA PCA_CHANNEL_0 // PWM channel for controlling speed of Motor A
#define AIN1 PCA_CHANNEL_1 // GPIO channel for direction control AIN1
#define AIN2 PCA_CHANNEL_2 // GPIO channel for direction control AIN2

#define MOTORA 0 // Defining Motor A identifier

// Enumeration to represent motor direction
typedef enum
{
    FORWARD = 1,
    BACKWARD,
} DIR;

// Function prototype for motor operation
void Motor_Run(DIR dir, UWORD speed);

#endif