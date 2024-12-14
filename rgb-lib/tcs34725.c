/**************************************************************
* Class: CSC-615-01 Fall 2024
* Team Name: Wayno
* Github Name: n-ndo
* Project: Assignment 5 - RGB Sensor
*
* File: tcs34725.c
*
* Description: This program integrates the TCS34725 RGB sensor
*              with a Raspberry Pi to detect colors. The program
*              reads raw RGB values, converts them into hexadecimal
*              representation, and identifies the closest matching
*              color with a confidence level. Additional functionality
*              includes LED brightness control and gamma correction
*              to normalize color values for accuracy.
**************************************************************/

#include "tcs34725.h"
#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>
#include <math.h>
#include <float.h>

// Color structure
typedef struct
{
    char *name; 
    uint8_t r;  
    uint8_t g;  
    uint8_t b;  
} color_t;

// Color match definition
typedef struct
{
    char *color_name;
    double confidence;
} color_match_t;

// Color list
color_t major_colors[] = {
    {"Red", 255, 0, 0},
    {"Green", 0, 255, 0},
    {"Blue", 0, 0, 255},
    {"Yellow", 255, 255, 0},
    {"Cyan", 0, 255, 255},
    {"Magenta", 255, 0, 255},
    {"White", 255, 255, 255},
    {"Black", 0, 0, 0},
    {"Orange", 255, 165, 0},
    {"Pink", 255, 192, 203},
    {"Purple", 128, 0, 128},
    {"Brown", 139, 69, 19},
    {"Gray", 128, 128, 128},
    {"Light Blue", 173, 216, 230},
    {"Dark Green", 0, 100, 0},
    {"Beige", 245, 245, 220},
    {"Olive", 128, 128, 0},
    {"Teal", 0, 128, 128},
    {"Navy", 0, 0, 128},
    {"Maroon", 128, 0, 0},
    {"Dark Red", 139, 0, 0},
    {"Dark Blue", 0, 0, 139},
    {"Dark Cyan", 0, 139, 139},
    {"Dark Magenta", 139, 0, 139},
    {"Dark Yellow", 139, 139, 0},
    {"Dark Orange", 255, 140, 0},
    {"Dark Pink", 231, 84, 128},
    {"Dark Brown", 92, 51, 23},
    {"Dark Gray", 64, 64, 64},
    {"Dark Purple", 48, 25, 52},
    {"Dark Teal", 0, 77, 77},
    {"Dark Navy", 0, 0, 51},
    {"Dark Maroon", 77, 0, 0}
    };

// Initialize the TCS34725 sensor
int init_TCS34725(uint8_t integration_time, uint8_t gain)
{
    int handle = i2cOpen(1, TCS34725_ADDR, 0);
    if (handle < 0)
    {
        printf("Failed to open I2C communication\n");
        return -1;
    }
 // Enable the sensor
    enable_sensor(handle);
    usleep(2400);

 // Set the integration time and gain
    i2cWriteByteData(handle, TCS34725_CMD | TCS34725_ATIME, integration_time);
    i2cWriteByteData(handle, TCS34725_CMD | TCS34725_CONTROL, gain);

    return handle;
}
// Enable the sensor
void enable_sensor(int handle)
{
    uint8_t enable = TCS34725_PON | TCS34725_AEN;
    i2cWriteByteData(handle, TCS34725_CMD | TCS34725_ENABLE, enable);
}

// Read the raw RGB data returned from the sensor
void read_color_data(int handle, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *clear)
{
    // Read the clear, red, green, and blue data
    uint8_t clearLowByte = i2cReadByteData(handle, TCS34725_CMD | TCS34725_CDATAL);
    uint8_t clearHighByte = i2cReadByteData(handle, TCS34725_CMD | TCS34725_CDATH);
    *clear = (clearHighByte << 8) | clearLowByte;
    uint8_t redLowByte = i2cReadByteData(handle, TCS34725_CMD | TCS34725_RDATAL);
    uint8_t redHighByte = i2cReadByteData(handle, TCS34725_CMD | TCS34725_RDATH);
    *r = (redHighByte << 8) | redLowByte;
    uint8_t greenLowByte = i2cReadByteData(handle, TCS34725_CMD | TCS34725_GDATAL);
    uint8_t greenHighByte = i2cReadByteData(handle, TCS34725_CMD | TCS34725_GDATH);
    *g = (greenHighByte << 8) | greenLowByte;
    uint8_t blueLowByte = i2cReadByteData(handle, TCS34725_CMD | TCS34725_BDATAL);
    uint8_t blueHighByte = i2cReadByteData(handle, TCS34725_CMD | TCS34725_BDATH);
    *b = (blueHighByte << 8) | blueLowByte;

    printf("Raw RGB: R=%d G=%d B=%d\n", *r, *g, *b);
}

color_match_t get_closest_color_with_confidence(uint8_t r, uint8_t g, uint8_t b) {

    // Find the closest color
    double min_diff = DBL_MAX;
    char *closest_color = "Unknown";

    // Iterate through the major colors
    for (int i = 0; i < sizeof(major_colors) / sizeof(major_colors[0]); i++) {
        double diff = fabs(r - major_colors[i].r) +
                      fabs(g - major_colors[i].g) +
                      fabs(b - major_colors[i].b);

        if (diff < min_diff) {
            min_diff = diff;
            closest_color = major_colors[i].name;
        }
    }
    // Calculate the confidence level
    double confidence = 100 - (min_diff * 1); 
    if (confidence < 0) confidence = 0;
    if (confidence > 100) confidence = 100;
    // Create a color match object
    color_match_t result;
    result.color_name = closest_color;
    result.confidence = confidence;

    return result;
}


void print_color_hex_and_name(uint16_t r, uint16_t g, uint16_t b)
{
    // Calculate the clear channel
    uint16_t clear = r + g + b;
    // Normalize the RGB values
    float redNorm = (float)r / clear * 255;
    float greenNorm = (float)g / clear * 255;
    float blueNorm = (float)b / clear * 255;

    if (redNorm > 255)
    {
        redNorm = 255;
    }

    if (greenNorm > 255)
    {
        greenNorm = 255;
    }

    if (blueNorm > 255)
    {
        blueNorm = 255;
    }
    // Apply gamma correction
    float gamma = 0.9;
    uint8_t red = apply_gamma(redNorm, gamma);
    uint8_t green = apply_gamma(greenNorm, gamma);
    uint8_t blue = apply_gamma(blueNorm, gamma);

    color_match_t match = get_closest_color_with_confidence(red, green, blue);

    printf("HEX Color: #%02X%02X%02X\n", red, green, blue);
    printf("Detected Color: %s (Confidence: %.2f%%)\n", match.color_name, match.confidence);
}

void set_led_brightness(int gpio, int percentage)
{
    // Validate the brightness percentage
    if (percentage < 0 || percentage > 100)
    {
        printf("Brightness should be between 0%% and 100%%.\n");
        return;
    }
    // Calculate the PWM value
    int pwmValue = (percentage * 255) / 100;

    gpioPWM(gpio, pwmValue);

    printf("Brightness set to %d%%.\n", percentage);
}
// Apply gamma correction
uint8_t apply_gamma(float value, float gamma)
{
    return (uint8_t)(pow(value / 255.0, gamma) * 255.0);
}
