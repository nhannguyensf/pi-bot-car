#include <stdio.h>
#include <time.h>
#include "../motor/MotorDriver.h"
#include "../line-sensor/line_sensor.h"
#include "../echoSensor/echoSensor.h"
#include <unistd.h>

// PID constants
#define KP 30.0
#define KI 0.0
#define KD 20.0

// Control limits
#define MAX_CONTROL 100
#define BASE_SPEED 60
#define MIN_SPEED 20

// Object detection threshold
#define OBJECT_THRESHOLD 20.0

// Global variables for PID calculation
static double last_error = 0;
static double integral = 0;

// Function to calculate weighted position from line sensors
double calculate_line_position(int* sensor_states) {
    const double weights[] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    double weighted_sum = 0;
    int active_sensors = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_states[i]) {
            weighted_sum += weights[i];
            active_sensors++;
        }
    }
    
    if (active_sensors == 0) {
        return (last_error > 0) ? 4.0 : -4.0;
    }
    
    return weighted_sum / active_sensors;
}

// Main PID control function
void pid_control() {
    double distances[NUM_SENSORS];

    // Check for objects first
    if (getCurrentDistances(distances) == 0) {
        // Print sensor distances for debugging
        printf("Sensor Distances: [");
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
        fflush(stdout); // Force print immediately

        // Check if any sensor detects an object within threshold
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (distances[i] > 0 && distances[i] < OBJECT_THRESHOLD) {
                // Object detected! Stop motors
                Motor_Run(MOTORA, 0);
                Motor_Run(MOTORB, 0);
                printf("Object detected at %.2f cm! Stopping motors.\n", distances[i]);
                fflush(stdout); // Force print immediately
                return;
            }
        }
    }

    // No objects detected, proceed with line following
    int sensor_states[NUM_SENSORS];
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

    // Dynamic speed adjustment
    int dynamic_speed = BASE_SPEED;
    if (fabs(error) > 1.0) {
        dynamic_speed = BASE_SPEED - ((fabs(error) - 1.0) * 25);
        if (dynamic_speed < MIN_SPEED) dynamic_speed = MIN_SPEED;
    }

    // Calculate motor speeds
    int left_speed = dynamic_speed;
    int right_speed = dynamic_speed;

    if (error > 0) {
        right_speed -= control;
    } else {
        left_speed += control;
    }

    // Ensure speeds are within bounds
    if (left_speed > 100) left_speed = 100;
    if (left_speed < -100) left_speed = -100;
    if (right_speed > 100) right_speed = 100;
    if (right_speed < -100) right_speed = -100;

    // Apply motor speeds
    Motor_Run(MOTORA, left_speed);
    Motor_Run(MOTORB, right_speed);

    // Update last error for next iteration
    last_error = error;

    // Debug output
    printf("PID Control - Error: %.2f, Left Speed: %d, Right Speed: %d\n",
           error, left_speed, right_speed);
}