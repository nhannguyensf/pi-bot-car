/**************************************************************
 * File    : pid.c
 * Purpose : Implements PID control for line tracking using 5 sensors.
 * Author  : Nhan Nguyen
 **************************************************************/

#include "pid.h"
// PID internal state
static double last_error = 0.0;
static double integral = 0.0;

/**
 * @brief Initialize PID variables.
 */
void pid_init() {
    last_error = 0.0;
    integral = 0.0;
    printf("PID Control initialized.\n");
}

/**
 * @brief Calculate weighted position from line sensors.
 * @param sensor_states Array of sensor states (1 or 0).
 * @return Weighted position between -2 and 2.
 */
double calculate_line_position(int* sensor_states) {
    // Weights for each sensor from left to right
    const double weights[] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    double weighted_sum = 0.0;
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

/**
 * @brief Compute PID output based on sensor states.
 * @param sensor_states Array of 5 sensor readings (1 or 0).
 * @return Calculated PID error.
 */
double pid_compute(int* sensor_states) {
    // Calculate current position error
    double error = calculate_line_position(sensor_states);
    
    // PID calculations
    integral += error;
    double derivative = error - last_error;
    
    // Anti-windup for integral term (if KI > 0)
    // Currently KI = 0, so this has no effect
    if (integral > PID_MAX) integral = PID_MAX;
    if (integral < PID_MIN) integral = PID_MIN;
    
    // Calculate control signal
    double control = (KP * error) + (KI * integral) + (KD * derivative);
    
    // Clamp control signal to prevent extreme motor adjustments
    if (control > PID_MAX) {
        control = PID_MAX;
    } else if (control < PID_MIN) {
        control = PID_MIN;
    }
    
    // Update previous error for next iteration
    last_error = error;
    
    // Debug output
    printf("Error: %.2f, Control: %.2f\n", error, control);
    
    return control;
}

/**
 * @brief Adjust motor speeds based on PID error.
 * @param error Calculated PID error.
 */
void adjust_motor_speed(double error) {
    // Calculate motor speeds
    double left_speed = BASE_SPEED - error;
    double right_speed = BASE_SPEED + error;
    
    // Ensure speeds are within bounds
    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (left_speed < MIN_SPEED) left_speed = MIN_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
    
    // Update motor speeds
    Motor_Run(MOTORA, (int)left_speed);  // Left motor
    Motor_Run(MOTORB, (int)right_speed); // Right motor
    
    // Print motor speed adjustments for debugging
    printf("Motor Speeds -> Left: %d, Right: %d\n", (int)left_speed, (int)right_speed);
}


