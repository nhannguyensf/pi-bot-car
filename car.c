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
#include <pthread.h>
#include <pigpio.h>
#include "motor/PCA9685.h"
#include "motor/DEV_Config.h"

#define BUTTON_PIN 17             // GPIO pin number for the button
#define LINE_SENSOR_LEFT_PIN 18   // GPIO pin for left line sensor
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
    PCA9685_SetPwmDutyCycle(PWMA, 0);
    PCA9685_SetPwmDutyCycle(PWMB, 0); // Set stop flag to exit main loop
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
    int value = gpioRead(pin);               // Read the GPIO pin value using pigpio
    printf("Pin %d read: %d\n", pin, value); // Debug
    return value;
}

// Thread function to control line sensors
void *line_sensor_control(void *arg)
{
    printf("Starting line sensor control...\n");
    while (!stop)
    {
        int left_sensor = read_pin(LINE_SENSOR_LEFT_PIN);
        int middle_sensor = read_pin(LINE_SENSOR_MIDDLE_PIN);
        int right_sensor = read_pin(LINE_SENSOR_RIGHT_PIN);

        if (right_sensor == 1 && middle_sensor == 0)
        {
            // Right sensor is out, turn left
            Motor_Run(PWMA, FORWARD, 30); // Make left motor slower
            printf("Turning left\n");
        }
        else if (left_sensor == 1 && middle_sensor == 0)
        {
            // Left sensor is out, turn right
            Motor_Run(PWMB, FORWARD, 30); // Make right motor slower
            printf("Turning right\n");
        }
        else
        {
            // All sensors on line or middle sensor is on
            Motor_Run(PWMA, FORWARD, 50); // Move forward at moderate speed
            Motor_Run(PWMA, FORWARD, 50); // Move forward at moderate speed
            printf("Moving forward\n");
        }

        usleep(100000); // Poll every 100ms
    }
    pthread_exit(NULL);
}

// Function to initialize the SPI interface
void setup_spi()
{
    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root?\n");
        exit(1);
    }

    if (!bcm2835_spi_begin())
    {
        printf("bcm2835_spi_begin failed. Are you running as root?\n");
        exit(1);
    }

    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // Most significant bit first
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // SPI mode 0
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // Slowest clock speed
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // Select CE0 pin
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // Chip select active low
    printf("SPI setup complete\n");                               // Debug
}

// Function to read encoder data via SPI
uint8_t read_encoder()
{
    char data = 0x00;                  // Placeholder data for reading
    data = bcm2835_spi_transfer(0x00); // Read data from SPI
    return data;
}

int main(void)
{
    // Handle ctrl + c signal
    signal(SIGINT, Handler);

    printf("Initializing system...\n"); // Debug
    // Initialize pigpio library
    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "pigpio initialization failed\n");
        return 1;
    }

    // 1.System Initialization
    if (DEV_ModuleInit())
        exit(0);

    // Set up GPIO memory for direct access
    setup_gpio_memory();

    // Set up SPI for motor encoder
    setup_spi();

    printf("System initialization complete\n"); // Debug

    // 2. Motor Initialization (PCA9685 PWM controller setup)
    PCA9685_Init(0x40);      // Initialize PCA9685 with default I2C address
    PCA9685_SetPWMFreq(100); // Set PWM frequency to 100 Hz

    /*
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
        Motor_Run(PWMA, FORWARD, 100);

        // Create thread for line sensor control
        pthread_t line_sensor_thread;
        if (pthread_create(&line_sensor_thread, NULL, line_sensor_control, NULL) != 0)
        {
            fprintf(stderr, "Error creating line sensor thread\n");
            DEV_ModuleExit();
            gpioTerminate();
            return 1;
        }

        // Wait for line sensor control thread to finish
        pthread_join(line_sensor_thread, NULL);

        // exit_program:

        // Program finished, exit gracefully
        printf("Exiting program\n"); // Debug
        PCA9685_SetPwmDutyCycle(PWMA, 0);
        PCA9685_SetPwmDutyCycle(PWMB, 0);
        // 3.System Exit
        DEV_ModuleExit();
        gpioTerminate();
        return 0;
        */
    // Motor A running forward at 100% speed
    printf("Running motor forward at 100%% speed\n");
    Motor_Run(FORWARD, 100);

    // Monitor encoder data for feedback
    for (int i = 0; i < 50 && !stop; i++)
    {
        uint8_t encoder_data = read_encoder();
        printf("Encoder Data: %d\n", encoder_data); // Debug for encoder feedback
        usleep(100000);                             // 0.1 second delay, total 5 seconds
    }

    // Stop the motor
    printf("Stopping motor\n");
    PCA9685_SetPwmDutyCycle(PWMA, 0);

    // Program finished, exit gracefully
    printf("Exiting program\n"); // Debug
    // 3. System Exit
    DEV_ModuleExit();
    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
