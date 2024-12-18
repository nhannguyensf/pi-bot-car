#include <stdio.h>
#include "../motor/MotorDriver.h"
#include "../line-sensor/line_sensor.h"
#include "../echoSensor/echoSensor.h"
#include <unistd.h>
#include <stdbool.h>
#include <time.h>

// PID constants
#define KP 40
#define KI 1.7
#define KD 1.2

// Control limits
#define MAX_CONTROL 200
#define BASE_SPEED 75

// Object detection thresholds
#define FRONT_THRESHOLD 20.0  // Distance to detect front obstacle
#define SIDE_THRESHOLD 25.0   // Distance to detect side obstacle
#define TURN_SPEED 15        // Speed for turning
#define AVOID_SPEED 50       // Speed while avoiding obstacle
#define ARC_INNER_SPEED 30   // Speed for inner wheel during arc turn
#define ARC_OUTER_SPEED 80   // Speed for outer wheel during arc turn

// Turn timing (microseconds)
#define TURN_90_TIME 12500000  // 750ms for 90-degree turn at speed 15
#define ARC_TURN_TIME 2000000 // 1.5s for arc turn

// Robot states
typedef enum {
    FOLLOWING_LINE,
    STOPPING
} RobotState;

static RobotState current_state = FOLLOWING_LINE;
static struct timespec turn_start_time;
static bool turn_started = false;

// Global variables for PID calculation
static double last_error = 0;
static double integral = 0;

// Function to safely stop motors
static void stop_motors() {
    Motor_Run(MOTORA, 0);
    Motor_Run(MOTORB, 0);
    usleep(50000); // Short delay after stopping
}

// Function to start turn timer
static void start_turn_timer() {
    clock_gettime(CLOCK_MONOTONIC, &turn_start_time);
    turn_started = true;
}

// Function to check if turn is complete
static bool is_turn_complete() {
    if (!turn_started) return false;
    
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    long elapsed_us = (current_time.tv_sec - turn_start_time.tv_sec) * 1000000 +
                     (current_time.tv_nsec - turn_start_time.tv_nsec) / 1000;
                     
    return elapsed_us >= TURN_90_TIME;
}

// Function to check front sensor for obstacles
static bool check_front_obstacle() {
    double distances[NUM_SENSORS];
    if (getCurrentDistances(distances) == 0) {
        // Check middle sensor (index 1)
        if (distances[1] > 0 && distances[1] < FRONT_THRESHOLD) {
            printf("Front obstacle detected at %.2f cm!\n", distances[1]);
            return true;
        }
    }
    return false;
}

// Function to check side sensors for obstacles
static bool check_side_obstacle(int side) {
    double distances[NUM_SENSORS];
    if (getCurrentDistances(distances) == 0) {
        // side 0 for left (index 0), side 2 for right (index 2)
        if (distances[side] > 0 && distances[side] < SIDE_THRESHOLD) {
            printf("%s obstacle detected at %.2f cm!\n", 
                   side == 0 ? "Left" : "Right", distances[side]);
            return true;
        }
    }
    return false;
}

// Function to calculate weighted position from line sensors
double calculate_line_position(int* sensor_states) {
    const double weights[] = {-3.2, -1.8, 0.0, 1.8, 3.2};
    double weighted_sum = 0;
    int active_sensors = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_states[i]) {
            weighted_sum += weights[i];
            active_sensors++;
        }
    }
    
    if (active_sensors == 0) {
        return last_error;
    }
    
    return weighted_sum / active_sensors;
}

// Function to check if we've found the line
static bool check_for_line() {
    int sensor_states[NUM_SENSORS];
    read_line_sensors(sensor_states);
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_states[i]) {
            return true;
        }
    }
    return false;
}

// Main PID control function
void pid_control() {
    static int echo_check_counter = 0;
    int sensor_states[NUM_SENSORS];

    // State machine for robot behavior
    switch (current_state) {
        case FOLLOWING_LINE:
            // Check for obstacles periodically
            if (++echo_check_counter >= 1) {
                echo_check_counter = 0;
                if (check_front_obstacle()) {
                    printf("Front obstacle detected! Stopping...\n");
                    stop_motors();
                    current_state = STOPPING;
                    return;
                }
            }

            // Normal line following
            read_line_sensors(sensor_states);
            double error = calculate_line_position(sensor_states);
            integral += error;
            double derivative = error - last_error;
            
            if (integral > MAX_CONTROL) integral = MAX_CONTROL;
            if (integral < -MAX_CONTROL) integral = -MAX_CONTROL;
            
            double control = KP * error + KI * integral + KD * derivative;
            if (control > MAX_CONTROL) control = MAX_CONTROL;
            if (control < -MAX_CONTROL) control = -MAX_CONTROL;
            
            int left_speed = BASE_SPEED - control;
            int right_speed = BASE_SPEED + control;
            
            if (left_speed > 100) left_speed = 100;
            if (left_speed < -100) left_speed = -100;
            if (right_speed > 100) right_speed = 100;
            if (right_speed < -100) right_speed = -100;
            
            Motor_Run(MOTORA, left_speed);
            Motor_Run(MOTORB, right_speed);
            last_error = error;
            break;

        case STOPPING:
            // Just stay stopped
            break;
    }
}