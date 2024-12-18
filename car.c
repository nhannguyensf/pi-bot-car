// car.c
#include "car.h"
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <pigpio.h>  // Include pigpio library
#include <signal.h>

#include "MotorDriver.h"
#include "line_sensor.h"
#include "tcs34725.h"

// Define other necessary includes and macros...

volatile sig_atomic_t stop = 0; // Flag to indicate program termination (ctrl + c)
static int color_result = 0;

// Signal handler to stop the motor safely and set stop flag
void Handler(int signo)
{
    printf("\r\nHandler: Terminating program...\n");
    stop = 1;
}

int main(void) {
    // Initialize pigpio
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Error: Failed to initialize pigpio\n");
        return EXIT_FAILURE;
    }

    // Set up signal handler for graceful termination
    signal(SIGINT, Handler);

    // Initialize Motor System (handles PCA9685 via pigpio)
    printf("Initializing motor system...\n");
    initializeMotorSystem(); // Adjusted to use pigpio exclusively

    // Initialize Echo Sensors (Assuming these are ultrasonic sensors using pigpio)
    printf("Initializing echo sensors...\n");
    if (initEchoSensors() < 0) {
        printf("Failed to initialize echo sensors\n");
        gpioTerminate();
        return EXIT_FAILURE;
    }

    // Initialize Line Sensors
    printf("Initializing line sensors...\n");
    if (initializeLineSensors() < 0) {
        printf("Failed to initialize line sensors\n");
        gpioTerminate();
        return EXIT_FAILURE;
    }

    // Initialize Encoders
    printf("Initializing encoders...\n");
    initializeEncoder(SPI0_CE0, "Motor A");
    initializeEncoder(SPI0_CE1, "Motor B");

    // Initialize TCS34725 RGB Sensor
    printf("Initializing TCS34725 sensor...\n");
    int tcs34725 = init_TCS34725("101ms", "60X");
    if (tcs34725 < 0) {
        fprintf(stderr, "Error: Failed to initialize TCS34725 sensor\n");
        stopMotors();
        cleanupEchoSensors();
        gpioTerminate();
        return EXIT_FAILURE;
    }

    printf("All systems initialized. Starting control loop...\n");

    // Main control loop
    while(!stop) {
        pid_control(); // Your PID control function

        // Detect color and adjust LED brightness
        if (detect_and_adjust_led(tcs34725, &color_result) < 0) {
            fprintf(stderr, "Error: Failed to detect color and adjust LED\n");
            // Optionally, handle the error or set stop = 1 to exit
        }

        if(!color_result) {
            printf("Color detected: Unknown. Stopping motors.\n");
            stop = 1;
        }

        // Additional application logic...

        gpioSleep(GPIO_TIME_RELATIVE, 0, 100000); // 100ms delay (pigpio uses microseconds)
    }

    // Cleanup
    printf("\nCleaning up...\n");
    stopMotors();
    i2cClose(tcs34725);   // Close I2C handle for TCS34725
    cleanupEchoSensors();
    cleanupLineSensors();  // Assuming you have a cleanup function for line sensors
    gpioTerminate();       // Terminate pigpio

    printf("Program exited successfully.\n");
    return 0;
}
