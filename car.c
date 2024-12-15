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
    // Initialize pigpio library
    if (gpioInitialise() < 0) {
        printf("pigpio initialization failed.\n");
        return 1;
    }

    // Initialize motors
    Motor_Init();

    // Initialize line sensors
    line_sensors_init();

    // Initialize PID controller
    pid_init();

    // (Optional) Initialize LED pin
    /*
    gpioSetMode(LED_PIN, PI_OUTPUT);
    gpioWrite(LED_PIN, 0); // LED off
    */

    // Register signal handler for graceful termination
    if (signal(SIGINT, Handler) == SIG_ERR) {
        printf("Failed to set signal handler.\n");
        gpioTerminate();
        return 1;
    }

    printf("Line-Following Car with PID Control is running. Press CTRL+C to exit.\n");

    while (!stop) {
        int sensor_states[NUM_SENSORS];

        // Read line sensor states (1 for line detected, 0 otherwise)
        read_line_sensors(sensor_states);

        // Compute PID control signal based on sensor states
        double control = pid_compute(sensor_states);

        // Adjust motor speeds based on PID control signal
        adjust_motor_speed(control);

        // (Optional) Adjust LED Brightness or other features
        /*
        // Example: Adjust LED based on control signal or sensor states
        if (control > 10) {
            gpioWrite(LED_PIN, 1); // Turn LED on
        } else {
            gpioWrite(LED_PIN, 0); // Turn LED off
        }
        */

        usleep(50000); // 50ms delay for smoother control
    }

    // Stop motors and cleanup
    Motor_Stop_All();
    gpioTerminate();
    printf("Program exited successfully.\n");

    return 0;
}