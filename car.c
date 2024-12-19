/**
Class         : CSC-615-01 - Embedded Linux - Fall 2024
Team Name     : Wayno
Github        : nhannguyensf
Project       : Final Assignment - Robot Car
File          : car.c
Description:
This file is the main file for the robot car project. 
It initializes all the systems, including the motor system, echo sensors, encoders, and the TCS34725 sensor. 
It then enters the main control loop, which uses a PID controller to control the car's movement. 
The control loop also uses the TCS34725 sensor to detect the color of the LED and adjust the car's movement accordingly. 
The program exits when the user presses Ctrl+C, and all systems are cleaned up.
*
Team Members:
Kiran Poudel
Nhan Nguyen
Yuvraj Gupta
Fernando Abel Malca Luque

*
**/ 
#include "car.h"
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <pigpio.h>
#include <bcm2835.h>
#include <signal.h>

volatile sig_atomic_t stop = 0; 
static int color_result = 0;

// Signal handler to stop the motor safely and set stop flag
void Handler(int signo)
{
    //Ensure the motor stops
    printf("\r\nHandler: Motor Stop\r\n");
    stop = 1;
}


int main(void) {
    // Initialize all systems
    printf("Initializing motor system...\n");
    initializeMotorSystem();

    printf("Initializing echo sensors...\n");
    if (initEchoSensors() < 0) {
        printf("Failed to initialize echo sensors\n");
        return 1;
    }

    // Set up signal handler
    signal(SIGINT, Handler);
    // Set up the encoder
    printf("Initializing encoders...\n");
    initializeEncoder(SPI0_CE0, "Motor A");
    initializeEncoder(SPI0_CE1, "Motor B");

    /*
    printf("Initializing TCS34725 sensor...\n");
    int tcs34725 = init_TCS34725("101ms", "60X");
    if (tcs34725 < 0) {
        gpioTerminate();
        return EXIT_FAILURE;
    }
    */
    //Begin the robot car's main operational loop, where it reacts to sensor inputs in real-time.
    printf("All systems initialized. Starting control loop...\n");

    // Main control loop
    while(!stop) {
        // Use PID control to adjust the car's movement based on sensor feedback.
        pid_control();
        // Use the TCS34725 sensor to detect the color of the LED and stop the car.
        /*
        if (detect_and_adjust_led(tcs34725, &color_result) < 0) {
            stop = 1;
        }

        if(!color_result) {
            stop = 1;
        }
        */
    }

    // Once the program exits, ensure all resources are safely released and motors are stopped.
    printf("\nCleaning up...\n");
    stopMotors();
//    i2cClose(tcs34725);
    cleanupEchoSensors();
    gpioTerminate();
    DEV_ModuleExit();
    printf("Program exited successfully.\n");
    return 0;
}
