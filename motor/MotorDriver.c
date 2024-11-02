/************************************************************** *
 * Class:: CSC-615-01 Fall 2024
 * Name:: Yuvraj Gupta
 * Student ID:: 922933190
 * Github-Name:: YuvrajGupta1808
 * Project: Assignment 3 - Start Your Motors
 * File: MotorDriver.c
 *
 * Description::
 * Measures distance using the HC-SR04 ultrasonic sensor and
 * controls GPIO pins using PiGPIO library. The program
 * calculates the distance by sending ultrasonic pulses and
 * reading the echo, converting the duration into a distance
 * value in centimeters.
 * **************************************************************/
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
void Motor_Run(DIR dir, UWORD speed)
{
    DEBUG("Motor A Speed = %d\r\n", speed);

    // Setting PWM duty cycle to control motor speed
    PCA9685_SetPwmDutyCycle(PWMA, speed);

    // Configuring motor direction based on input parameter
    if (dir == FORWARD)
    {
        DEBUG("forward...\r\n");
        PCA9685_SetLevel(AIN1, 0); // Setting AIN1 to low for forward direction
        PCA9685_SetLevel(AIN2, 1); // Setting AIN2 to high for forward direction

        DEBUG("Setting Motor A: AIN1 = %d, AIN2 = %d\r\n", 0, 1); // For forward
    }
    else
    {
        DEBUG("backward...\r\n");
        PCA9685_SetLevel(AIN1, 1); // Setting AIN1 to high for backward direction
        PCA9685_SetLevel(AIN2, 0); // Setting AIN2 to low for backward direction

        DEBUG("Setting Motor A: AIN1 = %d, AIN2 = %d\r\n", 1, 0); // For backward
    }
}
