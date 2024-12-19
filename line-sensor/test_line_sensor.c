/**
Class         : CSC-615-01 - Embedded Linux - Fall 2024
Team Name     : Wayno
Github        : nhannguyensf
Project       : Final Assignment - Robot Car
File          : test_line_sensor.c
Description:
This file is the main file for the line sensor testing. It initializes the line sensors and runs the test to log sensor readings.
*
Team Members:
Kiran Poudel
Nhan Nguyen
Yuvraj Gupta
Fernando Abel Malca Luque

*
**/ 
#include "line_sensor.h"
#include <bcm2835.h>
#include <stdio.h>

int main() {
    // Initialize the line sensors
    line_sensors_init();

    // Run the line sensors test
    test_line_sensors();

    // Terminate GPIO safely
    bcm2835_close();
    printf("Program terminated successfully. Stored %d readings.\n", reading_index);
    fflush(stdout);
    return 0;
}
