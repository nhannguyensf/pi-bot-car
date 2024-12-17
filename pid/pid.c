#include <stdio.h>
#include "../motor/MotorDriver.h"
#include "../line-sensor/line_sensor.h"
#include "../echoSensor/echoSensor.h"
#include <unistd.h>
#include <stdbool.h>

// PID constants for tuning the control loop
#define KP 35.5         // Proportional gain
#define KI 0            // Integral gain (set to 0, currently not in use)
#define KD 0            // Derivative gain (set to 0, currently not in use)

// Motor speed and control limits
#define BASE_SPEED 75       // Base speed for motors
#define MAX_CONTROL 100     // Maximum allowable control adjustment

// Obstacle detection threshold (in cm)
#define FRONT_THRESHOLD 20.0

// Line sensor configuration
#define NUM_SENSORS 5   // Number of line sensors

// Global PID control variables
static double last_error = 0.0;   // To store the previous error for derivative calculation
static double integral = 0.0;     // Integral error accumulation

// Function to safely stop both motors
void stop_motors() {
    Motor_Run(MOTORA, 0);
    Motor_Run(MOTORB, 0);
    printf("Motors stopped.\n");
}

// Function to check for a front obstacle using ultrasonic sensors
bool check_front_obstacle() {
    double distances[NUM_SENSORS]; // Array to hold distance readings
    if (getCurrentDistances(distances) == 0) { // Fetch sensor distances
        if (distances[1] > 0 && distances[1] < FRONT_THRESHOLD) {
            printf("Front obstacle detected at %.2f cm!\n", distances[1]);
            return true; // Obstacle detected
        }
    }
    return false; // No obstacle detected
}

// Function to calculate position error from line sensor states
double calculate_line_position(int* sensor_states) {
    // Weight assignments for each sensor: [-2, -1, 0, 1, 2]
    const double weights[NUM_SENSORS] = {-2.0, -1.0, 0.0, 1.0, 2.0}; 
    double weighted_sum = 0.0;  // Sum of weighted sensor values
    int active_sensors = 0;     // Count of active sensors

    // Compute the weighted sum of active sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_states[i]) { // If sensor detects line
            weighted_sum += weights[i];
            active_sensors++;
        }
    }

    // Return the average position error, or the last error if no sensors are active
    return (active_sensors == 0) ? last_error : (weighted_sum / active_sensors);
}

// PID control function for line-following
void pid_control() {
    int sensor_states[NUM_SENSORS] = {0}; // Array to hold sensor readings, initialized to 0

    // Check for obstacle at the front
    if (check_front_obstacle()) {
        stop_motors();
        printf("Obstacle detected. Stopping PID control.\n");
        return; // Exit control loop
    }

    // Read line sensor states
    read_line_sensors(sensor_states);

    // Calculate position error
    double error = calculate_line_position(sensor_states);
    integral += error;  // Accumulate integral error
    double derivative = error - last_error; // Calculate derivative of error

    // Calculate PID control output
    double control = KP * error + KI * integral + KD * derivative;

    // Limit control values to maximum bounds
    if (control > MAX_CONTROL) control = MAX_CONTROL;
    if (control < -MAX_CONTROL) control = -MAX_CONTROL;

    // Adjust motor speeds based on PID control
    int left_speed = BASE_SPEED - control;
    int right_speed = BASE_SPEED + control;

    // Clamp motor speeds within [-100, 100] range
    if (left_speed > 100) left_speed = 100;
    if (left_speed < -100) left_speed = -100;
    if (right_speed > 100) right_speed = 100;
    if (right_speed < -100) right_speed = -100;

    // Set motor speeds using motor driver
    Motor_Run(MOTORA, left_speed);
    Motor_Run(MOTORB, right_speed);

    // Debug output for PID status
    printf("PID Control - Left Speed: %d, Right Speed: %d, Error: %.2f\n", 
           left_speed, right_speed, error);

    // Update last error for the next iteration
    last_error = error;
}
