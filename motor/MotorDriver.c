// MotorDriver.c

#include "MotorDriver.h"
#include "PCA9685.h"  // Include the PCA9685 header
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>

// Static I2C handle for PCA9685
static int i2c_handle_pca9685 = -1;

// Function to initialize the motor system
int Motor_Init(void) {   
    // Open I2C handle for PCA9685
    i2c_handle_pca9685 = i2cOpen(1, 0x40, 0); // Bus 1, address 0x40, no flags
    if (i2c_handle_pca9685 < 0) {
        fprintf(stderr, "Error: Failed to open I2C for PCA9685. Error: %d\n", i2c_handle_pca9685);
        return -1;
    }

    // Initialize PCA9685
    if (PCA9685_Init(i2c_handle_pca9685) < 0) {
        fprintf(stderr, "Error: Failed to initialize PCA9685\n");
        i2cClose(i2c_handle_pca9685);
        return -1;
    }

    // Set PWM frequency (e.g., 200Hz)
    if (PCA9685_SetPWMFreq(i2c_handle_pca9685, 200.0f) < 0) {
        fprintf(stderr, "Error: Failed to set PWM frequency\n");
        i2cClose(i2c_handle_pca9685);
        return -1;
    }

    // Initialize GPIO pins for motor directions
    if (gpioSetMode(AIN1, PI_OUTPUT) < 0 ||
        gpioSetMode(AIN2, PI_OUTPUT) < 0 ||
        gpioSetMode(BIN1, PI_OUTPUT) < 0 ||
        gpioSetMode(BIN2, PI_OUTPUT) < 0) {
        fprintf(stderr, "Error: Failed to set GPIO modes for motor directions\n");
        i2cClose(i2c_handle_pca9685);
        return -1;
    }

    // Ensure motors are stopped initially
    Motor_Stop_All();

    printf("Motor system initialized successfully.\n");
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
        if (PCA9685_SetPwmDutyCycle(i2c_handle_pca9685, PWMA, 0, pwm_off) < 0) {
            fprintf(stderr, "Error: Failed to set PWM for Motor A\n");
            return -1;
        }
    }
    else if (motor == MOTORB) {
        if (PCA9685_SetPwmDutyCycle(i2c_handle_pca9685, PWMB, 0, pwm_off) < 0) {
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
        PCA9685_SetPwmDutyCycle(i2c_handle_pca9685, PWMA, 0, 0); // Stop Motor A
    }
    else if (motor == MOTORB)
    {
        PCA9685_SetPwmDutyCycle(i2c_handle_pca9685, PWMB, 0, 0); // Stop Motor B
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

    // Optionally, you can close the I2C handle here if you don't intend to use it again
    // i2cClose(i2c_handle_pca9685);

    printf("Motors stopped successfully.\n");
}
