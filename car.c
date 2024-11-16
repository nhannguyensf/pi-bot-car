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
#include "motor/PCA9685.h"
#include "motor/DEV_Config.h"
#include "motor/MotorDriver.h"
#include "motor/Debug.h"
#include <signal.h>

volatile sig_atomic_t stop = 0; // Flag to indicate program termination (ctrl + c)

// Signal handler to stop the motor safely and set stop flag
void Handler(int signo)
{
    // System Exit
    printf("\r\nHandler: Motor Stop\r\n");
    PCA9685_SetPwmDutyCycle(PWMA, 0);
    PCA9685_SetPwmDutyCycle(PWMB, 0);
    stop = 1;
}

int main(void)
{
    // Handle ctrl + c signal
    signal(SIGINT, Handler);

    printf("Initializing motor system...\n");

    // 1. System Initialization
    if (DEV_ModuleInit())
        return 1;

    // 2. Motor Initialization (PCA9685 PWM controller setup)
    Motor_Init(); // Initialize TS-25GA370H motor

    // Motor A running forward at 100% speed
    printf("Running motor forward at 100%% speed\n");
    Motor_Run(MOTORA, FORWARD, 100);

    // Keep running until ctrl + c is pressed
    while (!stop)
    {
        usleep(100000); // Poll every 100ms
    }

    // Stop the motor
    printf("Stopping motor\n");
    Motor_Stop(MOTORA);

    // Program finished, exit gracefully
    printf("Exiting program\n");
    DEV_ModuleExit();
    return 0;
}
