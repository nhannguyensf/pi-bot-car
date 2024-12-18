// PCA9685.c
#include "PCA9685.h"
#include <pigpio.h>
#include <stdio.h>
#include <math.h>

// Initialize the PCA9685 module
int PCA9685_Init(int i2c_handle)
{
    // Reset PCA9685
    if (i2cWriteByteData(i2c_handle, MODE1, 0x00) < 0) {
        fprintf(stderr, "Error: Failed to reset PCA9685\n");
        return -1;
    }

    // Set MODE2 to OUTDRV (to have totem-pole structure)
    if (i2cWriteByteData(i2c_handle, MODE2, OUTDRV) < 0) {
        fprintf(stderr, "Error: Failed to set MODE2\n");
        return -1;
    }

    // Set MODE1 to ALLCALL
    if (i2cWriteByteData(i2c_handle, MODE1, ALLCALL) < 0) {
        fprintf(stderr, "Error: Failed to set MODE1 to ALLCALL\n");
        return -1;
    }

    gpioSleep(GPIO_TIME_RELATIVE, 0, 100000); // Sleep for 100ms

    // Clear SLEEP bit to wake up
    uint8_t mode1;
    if (i2cReadByteData(i2c_handle, MODE1) < 0) {
        fprintf(stderr, "Error: Failed to read MODE1\n");
        return -1;
    }
    mode1 = i2cReadByteData(i2c_handle, MODE1);
    mode1 &= ~SLEEP;
    if (i2cWriteByteData(i2c_handle, MODE1, mode1) < 0) {
        fprintf(stderr, "Error: Failed to clear SLEEP bit\n");
        return -1;
    }

    gpioSleep(GPIO_TIME_RELATIVE, 0, 50000); // Sleep for 50ms

    return 0;
}

// Set the PWM frequency
int PCA9685_SetPWMFreq(int i2c_handle, float freq)
{
    // Calculate prescale value
    float prescaleval = 25000000.0f; // 25MHz
    prescaleval /= 4096.0f;           // 12-bit
    prescaleval /= freq;
    prescaleval -= 1.0f;

    uint8_t prescale = (uint8_t)floorf(prescaleval + 0.5f);

    // Read MODE1
    uint8_t oldmode = i2cReadByteData(i2c_handle, MODE1);
    if (oldmode < 0) {
        fprintf(stderr, "Error: Failed to read MODE1 for prescale\n");
        return -1;
    }

    // Put PCA9685 to sleep to set prescale
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
    if (i2cWriteByteData(i2c_handle, MODE1, newmode) < 0) {
        fprintf(stderr, "Error: Failed to set MODE1 to sleep for prescale\n");
        return -1;
    }

    // Set prescale
    if (i2cWriteByteData(i2c_handle, PRESCALE, prescale) < 0) {
        fprintf(stderr, "Error: Failed to write prescale\n");
        return -1;
    }

    // Wake up
    if (i2cWriteByteData(i2c_handle, MODE1, oldmode) < 0) {
        fprintf(stderr, "Error: Failed to wake up MODE1\n");
        return -1;
    }

    gpioSleep(GPIO_TIME_RELATIVE, 0, 50000); // Sleep for 50ms

    // Restart
    if (i2cWriteByteData(i2c_handle, MODE1, oldmode | 0x80) < 0) {
        fprintf(stderr, "Error: Failed to set RESTART bit\n");
        return -1;
    }

    return 0;
}

// Set PWM duty cycle for a specific channel
int PCA9685_SetPwmDutyCycle(int i2c_handle, uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel > 15) {
        fprintf(stderr, "Error: Invalid PCA9685 channel %d\n", channel);
        return -1;
    }

    uint8_t base_address = LED0_ON_L + 4 * channel;

    // Write ON time (2 bytes)
    if (i2cWriteByteData(i2c_handle, base_address, on & 0xFF) < 0 ||
        i2cWriteByteData(i2c_handle, base_address + 1, (on >> 8) & 0xFF) < 0) {
        fprintf(stderr, "Error: Failed to write ON time for channel %d\n", channel);
        return -1;
    }

    // Write OFF time (2 bytes)
    if (i2cWriteByteData(i2c_handle, base_address + 2, off & 0xFF) < 0 ||
        i2cWriteByteData(i2c_handle, base_address + 3, (off >> 8) & 0xFF) < 0) {
        fprintf(stderr, "Error: Failed to write OFF time for channel %d\n", channel);
        return -1;
    }

    return 0;
}

// Set PWM level (fully on or off)
int PCA9685_SetLevel(int i2c_handle, uint8_t channel, uint8_t level)
{
    if (level > 1) {
        fprintf(stderr, "Error: Invalid level %d. Must be 0 or 1.\n", level);
        return -1;
    }

    if (level == 1) {
        // Fully on
        if (i2cWriteByteData(i2c_handle, LED0_ON_L + 4 * channel, 0xFF) < 0 ||
            i2cWriteByteData(i2c_handle, LED0_ON_H + 4 * channel, 0xFF) < 0 ||
            i2cWriteByteData(i2c_handle, LED0_OFF_L + 4 * channel, 0x00) < 0 ||
            i2cWriteByteData(i2c_handle, LED0_OFF_H + 4 * channel, 0x10) < 0) { // SET_BIT(4) for full on
            fprintf(stderr, "Error: Failed to set channel %d to fully ON\n", channel);
            return -1;
        }
    }
    else {
        // Fully off
        if (i2cWriteByteData(i2c_handle, LED0_ON_L + 4 * channel, 0x00) < 0 ||
            i2cWriteByteData(i2c_handle, LED0_ON_H + 4 * channel, 0x00) < 0 ||
            i2cWriteByteData(i2c_handle, LED0_OFF_L + 4 * channel, 0x00) < 0 ||
            i2cWriteByteData(i2c_handle, LED0_OFF_H + 4 * channel, 0x10) < 0) { // SET_BIT(4) for full off
            fprintf(stderr, "Error: Failed to set channel %d to fully OFF\n", channel);
            return -1;
        }
    }

    return 0;
}
