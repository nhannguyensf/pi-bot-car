#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "../motor/MotorDriver.h"
#include "../line-sensor/line_sensor.h"

// Number of sensors
#define NUM_SENSORS 5

// PID Constants (These may need tuning)
#define KP 30.0f    // Reduced from 35.5f
#define KI 0.0f     // Integral term not used
#define KD 0.0f     // Introduced derivative gain

// PID Output Limits
#define PID_MAX 30.0f
#define PID_MIN -30.0f

// Motor speed limits
#define MAX_SPEED 100
#define MIN_SPEED 0

// Motor identifiers
#define MOTORA 0
#define MOTORB 1

// Base speed for motors
#define BASE_SPEED 50

// Function Prototypes
void pid_init();
double pid_compute(int* sensor_states);
void adjust_motor_speed(double error);

#endif // PID_H
