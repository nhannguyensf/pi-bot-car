/**************************************************************
* Class: CSC-615-01 Fall 2024
* Team Name: Wayno
* Github Name: n-ndo
* Project: Assignment 5 - RGB Sensor
*
* File: tcs34725.h
*
* Description:  Header file for the TCS34725 RGB sensor library. 
*               This file defines the constants, macros, and 
*               function prototypes for interfacing with the 
*               TCS34725 sensor. It supports initialization, 
*               data acquisition (RGB and clear), and utilities 
*               like LED control and gamma correction.
**************************************************************/

#ifndef TCS34725_H
#define TCS34725_H

#include <stdint.h>

// Sensor I2C address and command bit
#define TCS34725_ADDR 0x29  // Default I2C address
#define TCS34725_CMD 0x80   // Command register bit

// Power and ADC enable settings
#define TCS34725_PON 0x01   // Power ON
#define TCS34725_AEN 0x02   // ADC Enable

// Register addresses
#define TCS34725_ENABLE 0x00    // Enable register
#define TCS34725_ATIME 0x01     // Integration time register
#define TCS34725_CONTROL 0x0F   // Control register for gain

// Color data registers
#define TCS34725_CDATAL  0x14  // Low byte of the clear (no filter) channel color data
#define TCS34725_CDATH   0x15  // High byte of the clear (no filter) channel color data
#define TCS34725_RDATAL  0x16  // Low byte of the red channel color data
#define TCS34725_RDATH   0x17  // High byte of the red channel color data
#define TCS34725_GDATAL  0x18  // Low byte of the green channel color data
#define TCS34725_GDATH   0x19  // High byte of the green channel color data
#define TCS34725_BDATAL  0x1A  // Low byte of the blue channel color data
#define TCS34725_BDATH   0x1B  // High byte of the blue channel color data

// Integration times
#define TCS34725_INTEGRATIONTIME_2_4MS  0xFF  // Shortest integration time
#define TCS34725_INTEGRATIONTIME_24MS   0xF6  // 24ms integration time
#define TCS34725_INTEGRATIONTIME_50MS   0xEB  // 50ms integration time
#define TCS34725_INTEGRATIONTIME_101MS  0xD6  // 101ms integration time
#define TCS34725_INTEGRATIONTIME_240MS  0x9C  // 240ms integration time
#define TCS34725_INTEGRATIONTIME_600MS  0x06  // Longest integration time

// Gain settings
#define TCS34725_GAIN_1X 0x00   // No gain
#define TCS34725_GAIN_4X 0x01   // 4x gain
#define TCS34725_GAIN_16X 0x02  // 16x gain
#define TCS34725_GAIN_60X 0x03  // 60x gain
#define TCS34725_GAIN_64X 0x06  // 64x gain

// Functions
// Initialize the TCS34725 sensor
int init_TCS34725(uint8_t integration_time, uint8_t gain);

// Enable the sensor
void enable_sensor(int handle);

// Read RGB and clear channel data
void read_color_data(int handle, uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* clear);

// Print the HEX and COLOR name converted from the raw RGB values
void print_color_hex_and_name(uint16_t r, uint16_t g, uint16_t b);

// Set LED brightness (0-100%)
void set_led_brightness(int gpioPin, int brightness);

// Apply gamma correction to a value
uint8_t apply_gamma(float value, float gamma);

#endif
