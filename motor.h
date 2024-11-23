#ifndef MOTOR_H
#define MOTOR_H

#include <pigpio.h>
#include "ls7336r.h"

// Constants
#define COUNTS_PER_REVOLUTION 540
#define PI 3.141592654
#define WHEEL_DIAMETER_CM 6.5
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER_CM)

// Function declarations
void initializeMotorSystem();
int initializeEncoder(int cePin, const char* motorName);
void readEncoder(int cePin, int* lastCount, const char* motorName);
void stopMotors();

#endif // MOTOR_UTILS_H
