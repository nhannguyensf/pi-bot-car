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

// GPIO pins for Line Sensors
#define SENSOR_0_PIN 17
#define SENSOR_1_PIN 27
#define SENSOR_2_PIN 22
#define SENSOR_3_PIN 23
#define SENSOR_4_PIN 24

// Define total number of sensors
#define NUM_SENSORS 5

// Define time duration and array size
#define TIME_DURATION_SECONDS 20
#define MAX_READINGS 400  // Assuming 50ms intervals for 20 seconds

// Array to store sensor readings
extern int sensor_readings[MAX_READINGS][NUM_SENSORS];
extern int reading_index;

// Function Prototypes
void line_sensors_init();                 // Initialize GPIO pins for line sensors
void read_line_sensors(int* sensor_states); // Read the states of all sensors
void test_line_sensors();                 // Test the line sensors and log data

#endif // LINE_SENSOR_H
