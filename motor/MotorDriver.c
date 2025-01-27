/**
Class         : CSC-615-01 - Embedded Linux - Fall 2024
Team Name     : Wayno
Github        : nhannguyensf
Project       : Final Assignment - Robot Car
File          : MotorDriver.c
Description:
This file contains the motor driver functions for the robot car project. 
It includes functions to initialize the motor system, run the motors, and stop the motors.
*
Team Members:
Kiran Poudel
Nhan Nguyen
Yuvraj Gupta
Fernando Abel Malca Luque

*
**/ 
#include "MotorDriver.h"
#include "Debug.h"

/**
 * Motor rotation.
 *
 * @param dir: forward and backward.
 * @param speed: Rotation speed.  //(0~100)
 *
 * Example:
 * @code
 * Motor_Run(FORWARD, 50);
 */
//Initialize the motor system
void Motor_Init(void)
{   
    if (DEV_ModuleInit() != 0) {
        fprintf(stderr, "Error: Failed to initialize system module\n");
        exit(1);
    }

    if (gpioInitialise() < 0) {
        fprintf(stderr, "Error: Failed to initialize pigpio\n");
        exit(1);
    }

    PCA9685_Init(0x40);
    PCA9685_SetPWMFreq(200);

    printf("Motor system initialized successfully.\n");

}
// Run the motor with specified speed and motor
void Motor_Run(UBYTE motor, int speed)
{
    DIR dir;

    if (speed < 0) {
        dir = BACKWARD;
        speed = -speed; 
    } else {
        dir = FORWARD;
    }

    if (speed > 100)
        speed = 100;

    if (motor == MOTORA)
    {
        // DEBUG("Motor A Speed = %d\r\n", speed);
        PCA9685_SetPwmDutyCycle(PWMA, speed);
        if (dir == FORWARD)
        {
            // DEBUG("forward...\r\n");
            PCA9685_SetLevel(AIN1, 0);
            PCA9685_SetLevel(AIN2, 1);
        }
        else
        {
            // DEBUG("backward...\r\n");
            PCA9685_SetLevel(AIN1, 1);
            PCA9685_SetLevel(AIN2, 0);
        }
    }
    else
    {
        // DEBUG("Motor B Speed = %d\r\n", speed);
        PCA9685_SetPwmDutyCycle(PWMB, speed);
        if (dir == FORWARD)
        {
            // DEBUG("forward...\r\n");
            PCA9685_SetLevel(BIN1, 1);
            PCA9685_SetLevel(BIN2, 0);
        }
        else
        {
            // DEBUG("backward...\r\n");
            PCA9685_SetLevel(BIN1, 0);
            PCA9685_SetLevel(BIN2, 1);
        }
    }
}
// Stop the motor
void Motor_Stop(UBYTE motor)
{
    if (motor == MOTORA)
    {
        PCA9685_SetPwmDutyCycle(PWMA, 0);
    }
    else
    {
        PCA9685_SetPwmDutyCycle(PWMB, 0);
    }
}
// Stop all motors
void Motor_Stop_All(void) {
    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);

    gpioTerminate();
    DEV_ModuleExit();
    printf("Motors stopped successfully.\n");
}
