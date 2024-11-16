#ifndef __TB6612FNG_
#define __TB6612FNG_

// Including necessary dependencies for motor control
#include "DEV_Config.h"
#include "PCA9685.h"

// Defining GPIO pins and PWM channels for Motor A control
#define PWMA PCA_CHANNEL_0 // PWM channel for controlling speed of Motor A
#define AIN1 PCA_CHANNEL_1 // GPIO channel for direction control AIN1
#define AIN2 PCA_CHANNEL_2 // GPIO channel for direction control AIN2

#define PWMB PCA_CHANNEL_5
#define BIN1 PCA_CHANNEL_3
#define BIN2 PCA_CHANNEL_4

#define MOTORA 0
#define MOTORB 1 // Defining Motor A identifier

// Enumeration to represent motor direction
typedef enum
{
    FORWARD = 1,
    BACKWARD,
} DIR;

// Function prototype for motor operation
void Motor_Init(void);
void Motor_Run(UBYTE motor, DIR dir, UWORD speed);
void Motor_Stop(UBYTE motor);

#endif
