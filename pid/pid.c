#include <stdio.h>
#include <time.h>
#include "../motor/MotorDriver.h"
#include "../line-sensor/line_sensor.h"
#include "../echoSensor/echoSensor.h"

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
#define AVOIDANCE_SPEED 50
#define SAFE_DISTANCE 15.0
#define CORRECTION_FACTOR 2.0

// Global variables for PID calculation
static double last_error = 0;
static double integral = 0;
static int current_state = STATE_LINE_FOLLOWING;

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

// Function to handle dynamic object avoidance
void handle_object_avoidance(ObjectDetectionState* state) {
    double left_distance = state->side_distances[0];
    double right_distance = state->side_distances[1];
    double front_center = state->front_distances[1];
    
    int left_speed = AVOIDANCE_SPEED;
    int right_speed = AVOIDANCE_SPEED;
    
    // If front is blocked, make a wider turn
    if (state->front_blocked) {
        if (state->recommended_direction < 0) {  // Turn left
            right_speed = AVOIDANCE_SPEED;
            left_speed = -AVOIDANCE_SPEED/2;
        } else {  // Turn right
            left_speed = AVOIDANCE_SPEED;
            right_speed = -AVOIDANCE_SPEED/2;
        }
    } else {
        // Maintain safe distance from object while moving forward
        if (state->recommended_direction < 0) {  // Object on right
            // Adjust based on right sensor distance
            double distance_error = right_distance - SAFE_DISTANCE;
            int speed_adjustment = (int)(distance_error * CORRECTION_FACTOR);
            
            // Apply adjustment to maintain distance
            right_speed += speed_adjustment;
            left_speed -= speed_adjustment;
        } else {  // Object on left
            // Adjust based on left sensor distance
            double distance_error = left_distance - SAFE_DISTANCE;
            int speed_adjustment = (int)(distance_error * CORRECTION_FACTOR);
            
            // Apply adjustment to maintain distance
            left_speed += speed_adjustment;
            right_speed -= speed_adjustment;
        }
    }
    
    // Ensure speeds are within bounds
    if (left_speed > 100) left_speed = 100;
    if (left_speed < -100) left_speed = -100;
    if (right_speed > 100) right_speed = 100;
    if (right_speed < -100) right_speed = -100;
    
    // Apply motor speeds
    Motor_Run(MOTORA, left_speed);
    Motor_Run(MOTORB, right_speed);
    
    // Debug output
    printf("Avoiding - Left: %d, Right: %d, Front: %.2f, Sides: %.2f, %.2f\n",
           left_speed, right_speed, front_center, left_distance, right_distance);
}

// Main PID control function
void pid_control() {
    double distances[NUM_SENSORS];
    ObjectDetectionState state;

    // Get current sensor readings
    if (getCurrentDistances(distances) == 0) {
        // Print sensor distances in array format
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
    } else {
        printf("Failed to read distances.\n");
    }

    // Update object detection state
    updateObjectDetection(distances);
    getObjectDetectionState(&state);

    // Check if we need to avoid an object
    if (state.object_detected) {
        current_state = STATE_AVOIDING_OBJECT;
        handle_object_avoidance(&state);
        return;
    }

    // Return to line following if no object detected
    current_state = STATE_LINE_FOLLOWING;

    // Normal line-following PID control
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

    // Apply control to motors
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

    // Update motor speeds
    Motor_Run(MOTORA, left_speed);
    Motor_Run(MOTORB, right_speed);

    // Update last error for next iteration
    last_error = error;

    // Debug output
    printf("PID Control - Error: %.2f, Control: %.2f, Left Speed: %d, Right Speed: %d\n",
           error, control, left_speed, right_speed);
}
