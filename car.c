/************************************************************** *
 * Class:: CSC-615-01 Fall 2024
 * Name:: Yuvraj Gupta
 * Student ID:: 922933190
 * Github-Name:: YuvrajGupta1808
 * Project: Assignment 3 - Start Your Motors
 * File: assignment3.c
 *
 * Description::
 * This assignment involves creating a motor control setup
 * using a Raspberry Pi with the WaveShare Motor Driver HAT
 * board. The objective is to control a motor using I2C
 * communication, perform operations like stopping, moving
 * forward, moving in reverse, and controlling speed with
 * Pulse Width Modulation (PWM). A button is used to initiate the
 * motor control program, and the implementation is verified
 * using a physical demonstration.
 * **************************************************************/
#include "car.h"
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "PCA9685.h"

#define BUTTON_PIN 17             // GPIO pin number for the button
#define LINE_SENSOR_LEFT_PIN 17   // GPIO pin for left line sensor
#define LINE_SENSOR_MIDDLE_PIN 27 // GPIO pin for middle line sensor
#define LINE_SENSOR_RIGHT_PIN 22  // GPIO pin for right line sensor
#define GPIO_BASE_ADDR 0x3F200000 // Base address for GPIO (Raspberry Pi 3)
#define BLOCK_SIZE (4 * 1024)     // Block size for mapping memory

volatile unsigned int *gpio;    // Pointer for accessing GPIO memory
volatile sig_atomic_t stop = 0; // Flag to indicate program termination (ctrl + c)

// Signal handler to stop the motor safely and set stop flag
void Handler(int signo)
{
    // System Exit
    printf("\r\nHandler:Motor Stop\r\n"); // Stop the motor by setting duty cycle to 0
    PCA9685_SetPwmDutyCycle(PWMA, 0);     // Set stop flag to exit main loop
    stop = 1;
}

// Function to set up GPIO memory for direct access
void setup_gpio_memory()
{
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC); // Open /dev/mem for accessing GPIO registers
    if (mem_fd < 0)
    {
        perror("Unable to open /dev/mem"); // Exit if /dev/mem can't be opened
        exit(1);
    }

    void *gpio_map = mmap(
        NULL,                   // Any address in our space will do
        BLOCK_SIZE,             // Map length
        PROT_READ | PROT_WRITE, // Enable reading & writing to mapped memory
        MAP_SHARED,             // Shared with other processes
        mem_fd,                 // File to map
        GPIO_BASE_ADDR          // Offset to GPIO peripheral
    );

    if (gpio_map == MAP_FAILED)
    {
        perror("mmap error");
        close(mem_fd);
        exit(1);
    }

    gpio = (volatile unsigned int *)gpio_map; // Cast mapped memory to GPIO pointer
    close(mem_fd);
    printf("GPIO memory setup complete\n"); // Debug
}

// Function to read the button or line sensor state
int read_pin(int pin)
{
    int reg_offset = pin / 32; // Calculate register offset
    int bit = pin % 32;        // Calculate bit position in register
    // Read the pin value (GPIO input level)
    int value = (*(gpio + 13 + reg_offset) & (1 << bit)) ? 1 : 0;
    printf("Pin %d read: %d\n", pin, value); // Debug
    return value;
}

int main(void)
{
    // Handle ctrl + c signal
    signal(SIGINT, Handler);

    printf("Initializing system...\n"); // Debug

    // 1.System Initialization
    if (DEV_ModuleInit())
        exit(0);

    printf("System initialization complete\n"); // Debug

    // Set up GPIO memory for direct access
    setup_gpio_memory();

    // 2. Motor Initialization (PCA9685 PWM controller setup)
    PCA9685_Init(0x40);      // Initialize PCA9685 with default I2C address
    PCA9685_SetPWMFreq(100); // Set PWM frequency to 100 Hz

    // Wait for button press to start
    printf("Waiting for button press...\n");
    while (!read_pin(BUTTON_PIN) && !stop)
    {
        usleep(100000); // Poll every 100ms
    }

    printf("Button pressed, starting motor sequence\n"); // Debug

    if (stop) // Exit if ctrl + c was pressed
    {
        printf("Exiting program due to ctrl + c\n");
        DEV_ModuleExit();
        return 0;
    }

    printf("Motor_Run\r\n");

    // Motor A running forward at 100% speed for 2 seconds
    Motor_Run(FORWARD, 100);

    // Line sensor control logic
    printf("Starting line sensor control...\n");
    while (!stop)
    {
        int left_sensor = read_pin(LINE_SENSOR_LEFT_PIN);
        int middle_sensor = read_pin(LINE_SENSOR_MIDDLE_PIN);
        int right_sensor = read_pin(LINE_SENSOR_RIGHT_PIN);

        if (right_sensor == 1 && middle_sensor == 0)
        {
            // Right sensor is out, turn left
            Motor_Run_Left_Slow(); // Make left motor slower
            printf("Turning left\n");
        }
        else if (left_sensor == 1 && middle_sensor == 0)
        {
            // Left sensor is out, turn right
            Motor_Run_Right_Slow(); // Make right motor slower
            printf("Turning right\n");
        }
        else
        {
            // All sensors on line or middle sensor is on
            Motor_Run(FORWARD, 50); // Move forward at moderate speed
            printf("Moving forward\n");
        }

        usleep(100000); // Poll every 100ms
    }

exit_program:

    // Program finished, exit gracefully
    printf("Exiting program\n"); // Debug
    PCA9685_SetPwmDutyCycle(PWMA, 0);
    // 3.System Exit
    DEV_ModuleExit();
    return 0;
}
