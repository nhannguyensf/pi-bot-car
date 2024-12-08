#include "line_sensor.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

// Array to store sensor readings
int sensor_readings[MAX_READINGS][NUM_SENSORS];
int reading_index = 0;

// Initialize the line sensors GPIO pins
void line_sensors_init() {
    if (!bcm2835_init()) {
        fprintf(stderr, "bcm2835 initialization failed\n");
        exit(1);
    }

    // Set the sensor pins as input
    bcm2835_gpio_fsel(SENSOR_0_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(SENSOR_1_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(SENSOR_2_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(SENSOR_3_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(SENSOR_4_PIN, BCM2835_GPIO_FSEL_INPT);
}

// Read the state of all line sensors and store in an array
void read_line_sensors(int* sensor_states) {
    sensor_states[0] = bcm2835_gpio_lev(SENSOR_0_PIN);
    sensor_states[1] = bcm2835_gpio_lev(SENSOR_1_PIN);
    sensor_states[2] = bcm2835_gpio_lev(SENSOR_2_PIN);
    sensor_states[3] = bcm2835_gpio_lev(SENSOR_3_PIN);
    sensor_states[4] = bcm2835_gpio_lev(SENSOR_4_PIN);
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
        usleep(50000);   // Sleep for 50ms
    }
}
