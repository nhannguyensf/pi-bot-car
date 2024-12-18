// PCA9685.h
#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>

// PCA9685 Registers
#define MODE1        0x00
#define MODE2        0x01
#define SUBADR1      0x02
#define SUBADR2      0x03
#define SUBADR3      0x04
#define PRESCALE     0xFE
#define LED0_ON_L    0x06
#define LED0_ON_H    0x07
#define LED0_OFF_L   0x08
#define LED0_OFF_H   0x09
#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD

// MODE1 bits
#define RESTART      0x80
#define SLEEP        0x10
#define ALLCALL      0x01

// MODE2 bits
#define OUTDRV       0x04

// Function Declarations
int PCA9685_Init(int i2c_handle);
int PCA9685_SetPWMFreq(int i2c_handle, float freq);
int PCA9685_SetPwmDutyCycle(int i2c_handle, uint8_t channel, uint16_t on, uint16_t off);
int PCA9685_SetLevel(int i2c_handle, uint8_t channel, uint8_t level);

#endif // PCA9685_H
