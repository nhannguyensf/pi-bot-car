/**
Class         : CSC-615-01 - Embedded Linux - Fall 2024
Team Name     : Wayno
Github        : nhannguyensf
Project       : Final Assignment - Robot Car
File          : car.c
Description:
This file is the header file for the car.c maine file. It includes all the necessary libraries and headers for the project.
*
Team Members:
Kiran Poudel
Nhan Nguyen
Yuvraj Gupta
Fernando Abel Malca Luque

*
**/ 
#ifndef __CAR__
#define __CAR__

// Including necessary standard libraries
#include <stdio.h>  //printf()
#include <stdlib.h> //exit()
#include <signal.h>

// Including custom configurations and motor control headers
#include <time.h>
#include "motor/DEV_Config.h"
#include "motor/PCA9685.h"
#include "motor/MotorDriver.h"

// Including motor-related headers
#include "motor/DEV_Config.h"
#include "motor/PCA9685.h"
#include "motor/MotorDriver.h"
#include "encoder/ls7336r.h"
#include "encoder/motor.h"
#include "rgb/tcs34725.h"

#endif
