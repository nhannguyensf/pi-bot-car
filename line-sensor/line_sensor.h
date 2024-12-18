/**************************************************************
* Class    : CSC-615-01 Fall 2024
* Name     : Nhan Nguyen
* Student ID: 923100929
* Github   : nhannguyensf
* Project  : Final Assignment - Line Sensors Testing
*
* File     : line_sensor.h
*
* Description:
*   This header file contains the constants, function prototypes,
*   and shared variables for testing multiple line sensors.
**************************************************************/

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

// Define total number of sensors
#define NUM_SENSORS 5

// Define time duration and array size
#define TIME_DURATION_SECONDS 20
#define MAX_READINGS 400  // Assuming 50ms intervals for 20 seconds

// GPIO pins for Line Sensors
static const uint8_t LINE_SENSOR_PINS[NUM_SENSORS] = {17, 27, 22, 23, 24};

// Array to store sensor readings
extern int sensor_readings[MAX_READINGS][NUM_SENSORS];
extern int reading_index;

// Function Prototypes
int initializeLineSensors();                 // Initialize GPIO pins for line sensors
void read_line_sensors(int* sensor_states); // Read the states of all sensors
void test_line_sensors();                    // Test the line sensors and log data
void cleanupLineSensors();                   // Cleanup function for line sensors

#endif // LINE_SENSOR_H
