// motor.c
#include "motor.h"
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include "MotorDriver.h"  // Adjusted path if necessary
#include "PCA9685.h"
#include "Debug.h"

// External I2C handle defined in MotorDriver.c if needed
// extern int i2c_handle_pca9685;

// Function to initialize the motor system using pigpio exclusively
int Motor_Init(void) {   
    // Initialize MotorDriver which uses pigpio and PCA9685
    if (initializeMotorSystem() != 0) {
        fprintf(stderr, "Error: Failed to initialize MotorDriver\n");
        return -1;
    }
    return 0;
}

// Function to set motor speed
int setMotorSpeed(int motor, int speed) {
    if (motor != MOTORA && motor != MOTORB) {
        fprintf(stderr, "Error: Invalid motor identifier %d\n", motor);
        return -1;
    }

    if (speed < 0 || speed > 100) {
        fprintf(stderr, "Error: Speed %d out of range (0-100)\n", speed);
        return -1;
    }

    // Scale speed from 0-100 to 0-4095 for PCA9685 (12-bit resolution)
    uint16_t pwm_off = (uint16_t)((speed / 100.0) * 4095);

    if (motor == MOTORA) {
        if (PCA9685_SetPwmDutyCycle(PWMA, 0, pwm_off) < 0) {
            fprintf(stderr, "Error: Failed to set PWM for Motor A\n");
            return -1;
        }
    }
    else if (motor == MOTORB) {
        if (PCA9685_SetPwmDutyCycle(PWMB, 0, pwm_off) < 0) {
            fprintf(stderr, "Error: Failed to set PWM for Motor B\n");
            return -1;
        }
    }

    return 0;
}

// Function to set motor direction
int setMotorDirection(int motor, DIR direction) {
    if (motor != MOTORA && motor != MOTORB) {
        fprintf(stderr, "Error: Invalid motor identifier %d\n", motor);
        return -1;
    }

    if (direction != FORWARD && direction != BACKWARD) {
        fprintf(stderr, "Error: Invalid direction %d\n", direction);
        return -1;
    }

    if (motor == MOTORA) {
        if (direction == FORWARD) {
            gpioWrite(AIN1, 0);
            gpioWrite(AIN2, 1);
        }
        else { // BACKWARD
            gpioWrite(AIN1, 1);
            gpioWrite(AIN2, 0);
        }
    }
    else if (motor == MOTORB) {
        if (direction == FORWARD) {
            gpioWrite(BIN1, 1);
            gpioWrite(BIN2, 0);
        }
        else { // BACKWARD
            gpioWrite(BIN1, 0);
            gpioWrite(BIN2, 1);
        }
    }

    return 0;
}

// Function to run a motor with specified direction and speed
int Motor_Run(int motor, DIR direction, int speed) {
    if (setMotorDirection(motor, direction) != 0) {
        return -1;
    }

    if (setMotorSpeed(motor, speed) != 0) {
        return -1;
    }

    return 0;
}

// Function to stop a specific motor
void Motor_Stop(int motor)
{
    if (motor == MOTORA)
    {
        PCA9685_SetPwmDutyCycle(PWMA, 0, 0); // Stop Motor A
    }
    else if (motor == MOTORB)
    {
        PCA9685_SetPwmDutyCycle(PWMB, 0, 0); // Stop Motor B
    }
    else
    {
        fprintf(stderr, "Error: Invalid motor identifier %d\n", motor);
    }
}

// Function to stop all motors and perform cleanup
void Motor_Stop_All(void) {
    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);

    printf("Motors stopped successfully.\n");
}
