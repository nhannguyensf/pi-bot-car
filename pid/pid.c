#include <stdio.h>
#include <time.h>
#include "../motor/MotorDriver.h"
#include "../line-sensor/line_sensor.h"
#include "../ultrasonic/ultrasonic.h"

// PID constants
#define KP 30.0
#define KI 0.0
#define KD 20.0  // Add derivative for predictive correction

// Control limits
#define MAX_CONTROL 100
#define BASE_SPEED 60  // Lower base speed for better control
#define MIN_SPEED 20   // Minimum speed during sharp turns

// Control states
#define STATE_LINE_FOLLOWING 0
#define STATE_AVOIDING_OBJECT 1

// Object avoidance parameters
#define TURN_SPEED 50
#define FORWARD_SPEED 40
#define TURN_TIME_MS 500
#define FORWARD_TIME_MS 1000

// Global variables for PID calculation
static double last_error = 0;
static double integral = 0;
static int current_state = STATE_LINE_FOLLOWING;
static int avoidance_step = 0;
static time_t last_state_change = 0;

// Function to calculate weighted position from line sensors
// Returns a value between -2 and 2, where:
// -2 means far left, 0 means centered, 2 means far right
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
    
    // If no sensors are active, return a large error for aggressive correction
    if (active_sensors == 0) {
        // Return an amplified error in the last known direction
        return (last_error > 0) ? 4.0 : -4.0;
    }
    
    return weighted_sum / active_sensors;
}

// Function to handle object avoidance
void handle_object_avoidance(int direction) {
    time_t current_time = time(NULL);
    
    switch(avoidance_step) {
        case 0: // Initial turn
            if (direction > 0) {
                Motor_Run(MOTORA, TURN_SPEED);    // Left motor forward
                Motor_Run(MOTORB, -TURN_SPEED);   // Right motor backward
            } else {
                Motor_Run(MOTORA, -TURN_SPEED);   // Left motor backward
                Motor_Run(MOTORB, TURN_SPEED);    // Right motor forward
            }
            if (difftime(current_time, last_state_change) > TURN_TIME_MS/1000) {
                avoidance_step = 1;
                last_state_change = current_time;
            }
            break;
            
        case 1: // Move forward
            Motor_Run(MOTORA, FORWARD_SPEED);
            Motor_Run(MOTORB, FORWARD_SPEED);
            if (difftime(current_time, last_state_change) > FORWARD_TIME_MS/1000) {
                avoidance_step = 2;
                last_state_change = current_time;
            }
            break;
            
        case 2: // Return turn
            if (direction > 0) {
                Motor_Run(MOTORA, -TURN_SPEED);   // Left motor backward
                Motor_Run(MOTORB, TURN_SPEED);    // Right motor forward
            } else {
                Motor_Run(MOTORA, TURN_SPEED);    // Left motor forward
                Motor_Run(MOTORB, -TURN_SPEED);   // Right motor backward
            }
            if (difftime(current_time, last_state_change) > TURN_TIME_MS/1000) {
                current_state = STATE_LINE_FOLLOWING;
                avoidance_step = 0;
            }
            break;
    }
}

// Main PID control function
void pid_control() {
    double distances[NUM_SENSORS];
    int avoid_direction;
    
    // Check for objects
    getCurrentDistances(distances);
    if (checkForObject(distances, &avoid_direction)) {
        if (current_state == STATE_LINE_FOLLOWING) {
            current_state = STATE_AVOIDING_OBJECT;
            last_state_change = time(NULL);
            avoidance_step = 0;
        }
    }
    
    // Handle different states
    if (current_state == STATE_AVOIDING_OBJECT) {
        handle_object_avoidance(avoid_direction);
        return;
    }
    
    // Normal line following PID control
    int sensor_states[NUM_SENSORS];
    read_line_sensors(sensor_states);
    
    // Calculate current position error
    double error = calculate_line_position(sensor_states);
    
    // Calculate error magnitude for speed control
    double error_magnitude = fabs(error);
    
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
    
    // Dynamic speed adjustment based on error magnitude
    int dynamic_speed = BASE_SPEED;
    if (error_magnitude > 1.0) {
        // Reduce speed more aggressively during sharp turns
        dynamic_speed = BASE_SPEED - (error_magnitude - 1.0) * 25;
        if (dynamic_speed < MIN_SPEED) dynamic_speed = MIN_SPEED;
    }
    
    // Apply control to motors with dynamic speed
    int left_speed = dynamic_speed;
    int right_speed = dynamic_speed;
    
    if (error > 0) {
        // Turning right - slow down right motor
        right_speed -= control;
    } else {
        // Turning left - slow down left motor
        left_speed += control;
    }
    
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