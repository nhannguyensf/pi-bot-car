// MotorDriver.h

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

// Define motor identifiers
#define MOTORA 0
#define MOTORB 1

// Define PWM channels for PCA9685 (0-15)
#define PWMA 0  // PWM channel 0 for Motor A
#define PWMB 1  // PWM channel 1 for Motor B

// Define GPIO pins for motor direction control
#define AIN1 17 // GPIO pin 17 for Motor A IN1
#define AIN2 27 // GPIO pin 27 for Motor A IN2
#define BIN1 22 // GPIO pin 22 for Motor B IN1
#define BIN2 23 // GPIO pin 23 for Motor B IN2

// Enumeration to represent motor direction
typedef enum
{
    FORWARD = 1,
    BACKWARD,
} DIR;

// Function Prototypes
int Motor_Init(void);                    // Initialize motor system
int setMotorSpeed(int motor, int speed); // Set motor speed (0-100)
int setMotorDirection(int motor, DIR direction); // Set motor direction
int Motor_Run(int motor, DIR direction, int speed); // Run motor with direction and speed
void Motor_Stop(int motor);              // Stop a specific motor
void Motor_Stop_All(void);               // Stop all motors

#endif // MOTOR_DRIVER_H
