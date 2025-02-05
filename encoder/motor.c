/**
Class         : CSC-615-01 - Embedded Linux - Fall 2024
Team Name     : Wayno
Github        : nhannguyensf
Project       : Final Assignment - Robot Car
File          : motor.c
Description:
This file contains the motor system initialization and encoder reading functions for the robot car project.
*
Team Members:
Kiran Poudel
Nhan Nguyen
Yuvraj Gupta
Fernando Abel Malca Luque

*
**/ 
#include "motor.h"
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include "../motor/DEV_Config.h"
#include "../motor/MotorDriver.h"


void initializeMotorSystem() {
    if (DEV_ModuleInit() != 0) {
        fprintf(stderr, "Error: Failed to initialize system module\n");
        exit(1);
    }

    if (gpioInitialise() < 0) {
        fprintf(stderr, "Error: Failed to initialize pigpio\n");
        exit(1);
    }

    Motor_Init();
    printf("Motor system initialized successfully.\n");
}

int initializeEncoder(int cePin, const char* motorName) {
    printf("Initializing encoder for %s...\n", motorName);
    int ret = initLS7336RChip(cePin);
    if (ret != 0) {
        printf("Error initializing encoder for %s. Error code: %d\n", motorName, ret);
        return ret;
    }
    printf("Encoder for %s initialized successfully.\n", motorName);
    return 0;
}

void readEncoder(int cePin, int* lastCount, const char* motorName) {
    usleep(10);          // Short delay for stability

    int result = readLS7336RCounter(cePin);

    if(result<0){
        result = result - 2*result;
    }

    // Log raw data
    printf("Raw data from %s encoder: %08X\n", motorName, result);

    // Calculate delta, including negative values for backward motion
    int delta = result - *lastCount;

    // Calculate revolutions and speed (negative indicates reverse motion)
    double revolutions = (double)delta / COUNTS_PER_REVOLUTION;
    double speed = revolutions * WHEEL_CIRCUMFERENCE; // cm/s

    // Log processed data with motion direction
    if (delta < 0) {
        printf("%s is moving backward. Count: %d, Delta: %d, Revolutions: %f, Speed: %f cm/s\n", 
               motorName, result, delta, revolutions, speed);
    } else {
        printf("%s is moving forward. Count: %d, Delta: %d, Revolutions: %f, Speed: %f cm/s\n", 
               motorName, result, delta, revolutions, speed);
    }

    // Update last count
    *lastCount = result;
}

void stopMotors() {
    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);
    printf("Motors stopped successfully.\n");
}
