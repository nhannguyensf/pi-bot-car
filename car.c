/**************************************************************
* Class    : CSC-615-01 Fall 2024
* Name     : Nhan Nguyen
* Student ID: 923100929
* Github   : nhannguyensf
* Project  : Final Assignment - PID Control for Line Tracking
*
* File     : car.c
*
* Description:
*   This program initializes the car's components, including
*   motors, line sensors, and PID control. The car follows a
*   line using real-time sensor data and PID calculations.
*
* Usage:
*   Run the program and ensure the car is placed on the track.
*   Press Ctrl + C (SIGINT) to stop the program and halt motors.
**************************************************************/

#include "line_sensor.h"
#include "pid.h"
#include "MotorDriver.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

// Signal flag for stopping the program safely
volatile sig_atomic_t stop = 0;

/**
 * @brief Signal handler to stop motors safely and set stop flag.
 * @param signo Signal number (e.g., SIGINT).
 */
void signal_handler(int signo) {
    printf("\nTerminating program. Stopping all motors...\n");
    stop = 1;
}

int main(void) {
    // Initialize car components
    printf("Initializing car components...\n");
    Motor_Init();          // Initialize motors
    line_sensors_init();   // Initialize line sensors
    pid_init();            // Initialize PID control

    // Attach the signal handler for SIGINT (Ctrl + C)
    signal(SIGINT, signal_handler);

    // Array to store the current sensor states
    int sensor_states[NUM_SENSORS];

    printf("Car is now following the line. Press Ctrl + C to stop.\n");

    // Main loop for line tracking
    while (!stop) {
        // Read the current states of the line sensors
        read_line_sensors(sensor_states);

        // Compute the PID error based on sensor readings
        float error = pid_compute(sensor_states);

        // Print debug information (if desired, you can disable this for cleaner output)
        print_debug_info(sensor_states, error);

        // Adjust motor speeds based on the PID error
        adjust_motor_speed(error);

        // Add a short delay for stability (adjust as needed)
        // The delay gives time for motors to react and sensors to stabilize
        usleep(50000); // 50ms
    }

    // Stop all motors safely upon program termination
    Motor_Stop_All();
    printf("Program exited successfully. All motors stopped.\n");

    return 0;
}
