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

    printf("Initializing encoders...\n");
    initializeEncoder(SPI0_CE0, "Motor A");
    initializeEncoder(SPI0_CE1, "Motor B");

    printf("All systems initialized. Starting control loop...\n");

    // Main control loop
    while(!stop) {
        pid_control();
    }

    // Cleanup
    printf("\nCleaning up...\n");
    stopMotors();
    cleanupEchoSensors();
    gpioTerminate();
    DEV_ModuleExit();
    printf("Program exited successfully.\n");
    return 0;
}