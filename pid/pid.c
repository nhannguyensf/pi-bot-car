/**************************************************************
* File    : pid.c
* Purpose : Implements PID control for line tracking using 5 sensors.
* Author  : Nhan Nguyen
**************************************************************/

#include "pid.h"
#include "line_sensor.h" // To access line sensor states
#include "MotorDriver.h" // To control motor speeds
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Shared variables
int sensor_positions[NUM_SENSORS] = {-2, -1, 0, 1, 2}; // Positions of the 5 sensors
float pid_error = 0;
int motor_left_speed = 0;
int motor_right_speed = 0;

// Internal variables for PID calculation
static float prev_error = 0; // Previous error for derivative term
static float integral = 0;   // Integral term accumulator

/**
 * @brief Initialize PID variables.
 */
void pid_init() {
    pid_error = 0;
    prev_error = 0;
    integral = 0;
    printf("PID Control initialized.\n");
}

/**
 * @brief Print debug information for PID computation.
 * @param sensor_states Array of sensor states (1 or 0).
 * @param error Current calculated error.
 */
void print_debug_info(int* sensor_states, float error) {
    printf("Sensor States: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        printf("%d ", sensor_states[i]);
    }
    printf("\n");

    printf("PID Error: %.2f\n", error);
    printf("Motor Speeds: Left: %d, Right: %d\n", motor_left_speed, motor_right_speed);
}

/**
 * @brief Compute PID output based on sensor states.
 * @param sensor_states Array of 5 sensor readings (1 or 0).
 * @return Calculated PID error.
 */
float pid_compute(int* sensor_states) {
    int weighted_sum = 0;
    int total_active_sensors = 0;

    // Calculate weighted sum of active sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_states[i] == 1) {
            weighted_sum += sensor_positions[i];
            total_active_sensors++;
        }
    }

    // Check if no sensors are active (all 0)
    if (total_active_sensors == 0) {
        printf("No line detected. Stopping motors.\n");
        motor_left_speed = 0;
        motor_right_speed = 0;
        return 0;
    }

    // Calculate the proportional error
    float position = (float)weighted_sum / total_active_sensors;
    float error = position; // Deviation from the center (0 position)

    // PID terms
    float proportional = KP * error;
    integral += error; // Accumulate integral
    float integral_term = KI * integral;
    float derivative = KD * (error - prev_error);

    // Update PID error
    pid_error = proportional + integral_term + derivative;

    // Update previous error for next iteration
    prev_error = error;

    // Print debugging information
    print_debug_info(sensor_states, pid_error);

    return pid_error;
}

/**
 * @brief Adjust motor speeds based on PID error.
 * @param error Calculated PID error.
 */
void adjust_motor_speed(float error) {
    // Base speed for motors
    int base_speed = 50; // Adjust as per your system's requirement

    // Calculate motor speeds
    motor_left_speed = base_speed - error;
    motor_right_speed = base_speed + error;

    // Clamp motor speeds to valid range
    if (motor_left_speed > MAX_SPEED) motor_left_speed = MAX_SPEED;
    if (motor_left_speed < MIN_SPEED) motor_left_speed = MIN_SPEED;
    if (motor_right_speed > MAX_SPEED) motor_right_speed = MAX_SPEED;
    if (motor_right_speed < MIN_SPEED) motor_right_speed = MIN_SPEED;

    // Set motor speeds
    Motor_Run(LEFT_MOTOR, motor_left_speed);
    Motor_Run(RIGHT_MOTOR, motor_right_speed);

    // Print motor speed adjustments for debugging
    printf("Adjusting Motors: Left Motor Speed: %d, Right Motor Speed: %d\n", motor_left_speed, motor_right_speed);
}
