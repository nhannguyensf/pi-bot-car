#include <stdio.h>
#include "../motor/MotorDriver.h"
#include "../line-sensor/line_sensor.h"
#include "../echoSensor/echoSensor.h"
#include <unistd.h>
#include <stdbool.h>

// PID constants
#define KP 35.5  
#define KI 0  
#define KD 0  // Derivative gain

// Control limits
#define MAX_CONTROL 100
#define BASE_SPEED 75

// Object detection threshold
#define OBJECT_THRESHOLD 20.0

// Global variables for PID calculation
static double last_error = 0;
static double integral = 0;

// Function to check for obstacles
static bool check_for_obstacles() {
    double distances[NUM_SENSORS];
    
    if (getCurrentDistances(distances) == 0) {
        // Print all sensor distances for debugging
        printf("Echo Distances: [");
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (distances[i] < 0) {
                printf("NaN");
            } else {
                printf("%.2f", distances[i]);
            }
            if (i < NUM_SENSORS - 1) {
                printf(", ");
            }
        }
        printf("] cm\n");
        fflush(stdout);

        // Check each sensor for obstacles
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (distances[i] > 0 && distances[i] < OBJECT_THRESHOLD) {
                printf("Obstacle detected at sensor %d (%.2f cm)! Stopping!\n", 
                       i, distances[i]);
                fflush(stdout);
                return true;
            }
        }
    }
    return false;
}

// Function to calculate weighted position from line sensors
double calculate_line_position(int* sensor_states) {
    // Weights for each sensor from left to right
    const double weights[] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    double weighted_sum = 0;
    int active_sensors = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_states[i]) {
            weighted_sum += weights[i];
            active_sensors++;
        }
    }
    
    // If no sensors are active, return the last known position
    if (active_sensors == 0) {
        return last_error;
    }
    
    return weighted_sum / active_sensors;
}

// Main PID control function
void pid_control() {
    static int echo_check_counter = 0;
    int sensor_states[NUM_SENSORS];

    // Check for obstacles every few iterations
    if (++echo_check_counter >= 1) {
        echo_check_counter = 0;
        if (check_for_obstacles()) {
            // Stop motors and wait before rechecking
            Motor_Run(MOTORA, 0);
            Motor_Run(MOTORB, 0);
            usleep(100000); // Wait 100ms before rechecking
            return;
        }
    }

    // No obstacles detected, proceed with line following
    read_line_sensors(sensor_states);
    
    // Calculate current position error
    double error = calculate_line_position(sensor_states);
    
    // PID calculations
    integral += error;
    double derivative = error - last_error;
    
    // Anti-windup for integral term
    if (integral > MAX_CONTROL) integral = MAX_CONTROL;
    if (integral < -MAX_CONTROL) integral = -MAX_CONTROL;
    
    // Calculate control signal
    double control = KP * error + KI * integral + KD * derivative;
    
    // Limit control signal
    if (control > MAX_CONTROL) control = MAX_CONTROL;
    if (control < -MAX_CONTROL) control = -MAX_CONTROL;
    
    // Apply control to motors
    int left_speed = BASE_SPEED - control;
    int right_speed = BASE_SPEED + control;
    
    // Ensure speeds are within bounds
    if (left_speed > 100) left_speed = 100;
    if (left_speed < -100) left_speed = -100;
    if (right_speed > 100) right_speed = 100;
    if (right_speed < -100) right_speed = -100;
    
    // Update motor speeds
    Motor_Run(MOTORA, left_speed);  // Left motor
    Motor_Run(MOTORB, right_speed); // Right motor
    
    // Update last error for next iteration
    last_error = error;
    
    // Debug output
    printf("Error: %.2f, Control: %.2f, Left: %d, Right: %d\n", 
           error, control, left_speed, right_speed);
}