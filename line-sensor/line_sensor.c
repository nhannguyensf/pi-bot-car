// line_sensor.c
#include "line_sensor.h"
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Array to store sensor readings
int sensor_readings[MAX_READINGS][NUM_SENSORS];
int reading_index = 0;

// Initialize the line sensors GPIO pins using pigpio
int initializeLineSensors() {
    // Set the sensor pins as input
    for(int i = 0; i < NUM_SENSORS; i++) {
        if (gpioSetMode(LINE_SENSOR_PINS[i], PI_INPUT) < 0) {
            fprintf(stderr, "Error: Failed to set GPIO %d as input\n", LINE_SENSOR_PINS[i]);
            return -1;
        }
    }
    printf("Line sensors initialized successfully.\n");
    return 0;
}

// Read the state of all line sensors and store in an array
void read_line_sensors(int* sensor_states) {
    for(int i = 0; i < NUM_SENSORS; i++) {
        sensor_states[i] = gpioRead(LINE_SENSOR_PINS[i]);
    }
}

// Test the line sensors and store results in an array
void test_line_sensors() {
    time_t start_time = time(NULL);
    time_t current_time;

    while (1) {
        current_time = time(NULL);

        // Check if the program should terminate
        if (difftime(current_time, start_time) >= TIME_DURATION_SECONDS) {
            break;
        }

        // Read the sensor states and store them in the array
        if (reading_index < MAX_READINGS) {
            read_line_sensors(sensor_readings[reading_index]);
            reading_index++;
        }

        // Print the states of the line sensors as 1 or 0
        printf("Reading #%d:", reading_index);
        for (int i = 0; i < NUM_SENSORS; i++) {
            printf(" %d", sensor_readings[reading_index - 1][i]);
        }
        printf("\n");

        fflush(stdout);  // Ensure output is displayed immediately
        gpioSleep(GPIO_TIME_RELATIVE, 0, 50000);   // Sleep for 50ms
    }
}

// Cleanup function for line sensors if needed
void cleanupLineSensors() {
    // For pigpio, there's no need for explicit cleanup of GPIO modes
    // Any necessary cleanup can be done here
    printf("Line sensors cleanup completed.\n");
}
