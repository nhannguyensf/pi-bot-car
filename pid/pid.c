/**************************************************************
 * File    : pid.c
 * Purpose : Implements PID control for line tracking using 5 sensors.
 * Author  : Nhan Nguyen
 * Note    : Adjusted for sharper turns with no smoothing.
 *           This code uses higher KP and allows negative speeds
 *           for turning in place.
 **************************************************************/

#include "pid.h"
#include "MotorDriver.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int sensor_positions[NUM_SENSORS] = {-2, -1, 0, 1, 2};

// PID error variables
float pid_error = 0;
int motor_left_speed = 0;
int motor_right_speed = 0;

// Internal PID state
static float prev_error = 0;           
static float integral = 0;             
static const float integral_max = 50.0f;   
static const float integral_min = -50.0f;

// Base speed for forward motion
// If you need sharper turns, you can lower this base speed.
static const int base_speed = 45;

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
 * @brief Compute the PID output based on sensor states.
 * @param sensor_states Array of 5 sensor readings (1 or 0).
 * @return Calculated PID error.
 */
float pid_compute(int* sensor_states) {
    float weighted_sum = 0;
    int total_active_sensors = 0;

    // Calculate weighted sum of active sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_states[i] == 1) {
            weighted_sum += sensor_positions[i];
            total_active_sensors++;
        }
    }

    // Check if no sensors are active
    if (total_active_sensors == 0) {
        // No line detected: perform a sharp pivot turn in place
        printf("No line detected. Pivoting to find line.\n");

        // Pivot direction based on the previous error
        // If prev_error > 0, line was last seen to the right, so turn right:
        if (prev_error > 0) {
            motor_left_speed = 30;   
            motor_right_speed = -30; 
        } else {
            motor_left_speed = -30;  
            motor_right_speed = 30;  
        }

        // Clamp speeds to allowed range
        if (motor_left_speed > MAX_SPEED) motor_left_speed = MAX_SPEED;
        if (motor_left_speed < -MAX_SPEED) motor_left_speed = -MAX_SPEED;
        if (motor_right_speed > MAX_SPEED) motor_right_speed = MAX_SPEED;
        if (motor_right_speed < -MAX_SPEED) motor_right_speed = -MAX_SPEED;

        // Set motors directly
        Motor_Run(LEFT_MOTOR, motor_left_speed);
        Motor_Run(RIGHT_MOTOR, motor_right_speed);

        // No error update since line is not found
        return 0;
    }

    // Calculate the proportional error (position deviation)
    float position = weighted_sum / total_active_sensors;
    float error = position;

    // Compute PID terms
    float proportional = KP * error;

    integral += error;
    if (integral > integral_max) integral = integral_max;
    if (integral < integral_min) integral = integral_min;
    float integral_term = KI * integral;

    float derivative = KD * (error - prev_error);

    pid_error = proportional + integral_term + derivative;
    prev_error = error;

    // Clamp pid_error
    if (pid_error > 100.0f) pid_error = 100.0f;
    if (pid_error < -100.0f) pid_error = -100.0f;

    print_debug_info(sensor_states, pid_error);

    return pid_error;
}

/**
 * @brief Adjust motor speeds based on PID error.
 * @param error The PID error calculated by pid_compute.
 */
void adjust_motor_speed(float error) {
    // Calculate target speeds
    // Large KP means error will cause a big difference in speeds
    int target_left_speed = (int)((float)base_speed - error);
    int target_right_speed = (int)((float)base_speed + error);

    // Clamp target speeds
    if (target_left_speed > MAX_SPEED) target_left_speed = MAX_SPEED;
    if (target_left_speed < -MAX_SPEED) target_left_speed = -MAX_SPEED;
    if (target_right_speed > MAX_SPEED) target_right_speed = MAX_SPEED;
    if (target_right_speed < -MAX_SPEED) target_right_speed = -MAX_SPEED;

    // Directly set motor speeds
    motor_left_speed = target_left_speed;
    motor_right_speed = target_right_speed;

    Motor_Run(LEFT_MOTOR, motor_left_speed);
    Motor_Run(RIGHT_MOTOR, motor_right_speed);

    printf("Adjusting Motors: Left Motor Speed: %d, Right Motor Speed: %d\n", motor_left_speed, motor_right_speed);
}
