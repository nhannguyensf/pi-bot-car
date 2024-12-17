#include "tcs34725.h"
#include <pigpio.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

int keep_running = 1;

// Signal tcs34725r for graceful termination
void signal_handler(int signal) {
    keep_running = 0;
}

int main()
{

    // Initialize pigpio library
    if (gpioInitialise() < 0) {
        printf("Failed to initialize pigpio.\n");
        return EXIT_FAILURE;
    }

    // Setup signal tcs34725rs for graceful termination
    signal(SIGINT, signal_handler);

    // Initialize the TCS34725 sensor
    int tcs34725 = init_TCS34725("101ms", "60X");
    if (tcs34725 < 0) {
        gpioTerminate();
        return EXIT_FAILURE;
    }

    // Set initial LED brightness to 100%
    int current_brightness = 100;
    if (set_led_brightness(LED_PIN, current_brightness) < 0) {
        printf("Failed to set initial LED brightness.\n");
        i2cClose(tcs34725);
        gpioTerminate();
        return EXIT_FAILURE;
    }

    // Main detection loop
    while (keep_running) 
    {
        // Call the function from tcs34725.c to detect color and adjust LED
        if (detect_and_adjust_led(tcs34725) < 0) {
            keep_running = 0;
        }

        // Optional: Add a small delay to prevent rapid changes
        usleep(DELAY_US);
    }

    // Cleanup
    i2cClose(tcs34725);
    gpioTerminate();
    printf("Terminated.\n");
    return EXIT_SUCCESS;
}