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
#include <bcm2835.h> // Include bcm2835 library for SPI
#include <signal.h>

volatile sig_atomic_t stop = 0; // Flag to indicate program termination (ctrl + c)

// Signal handler to stop the motor safely and set stop flag
void Handler(int signo)
{
    // System Exit
    printf("\r\nHandler: Motor Stop\r\n");
    stop = 1;
    stopMotors();
    DEV_ModuleExit();
    exit(0);
}

int main(void) {
    initializeMotorSystem();

    // Start motors
    printf("Running motors forward at 50%% speed\n");
    Motor_Run(MOTORA, FORWARD, 50);
    Motor_Run(MOTORB, FORWARD, 50);

    // Initialize encoders
    initializeEncoder(SPI0_CE0, "Motor A");
    initializeEncoder(SPI0_CE1, "Motor B");

    int lastCountA = 0, lastCountB = 0;

    // Exception handling:ctrl + c
    signal(SIGINT, Handler);
    while(1) {
    for (int i = 0; i < 20 && !stop; i++) {
        readEncoder(SPI0_CE0, &lastCountA, "Motor A");
        readEncoder(SPI0_CE1, &lastCountB, "Motor B");
        sleep(1); // 1-second interval
    }
    }
    // Stop motors and cleanup
    stopMotors();
    gpioTerminate();
    DEV_ModuleExit();
    printf("Program exited successfully.\n");

    return 0;
}