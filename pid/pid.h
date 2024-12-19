/**
Class         : CSC-615-01 - Embedded Linux - Fall 2024
Team Name     : Wayno
Github        : nhannguyensf
Project       : Final Assignment - Robot Car
File          : pid.h
Description:
This file is the header file for the pid.c file. It includes all the necessary function declarations for the PID control algorithm.
*
Team Members:
Kiran Poudel
Nhan Nguyen
Yuvraj Gupta
Fernando Abel Malca Luque

*
**/ 
#ifndef PID_H
#define PID_H

// Function declarations
double calculate_line_position(int* sensor_states);
void pid_control(void);

#endif // PID_H