#include "tcs34725.h"
#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

// Define the color names array locally within this file
static const char* color_names[] = {
    "Red",
    "Unknown"
};

// Initialize the TCS34725 sensor with specified integration time and gain
int init_TCS34725(const char *integration_time_str, const char *gain_str)
{
    uint8_t integration_time;
    if (strcmp(integration_time_str, "2.4ms") == 0)
        integration_time = TCS34725_INTEGRATIONTIME_2_4MS;
    else if (strcmp(integration_time_str, "24ms") == 0)
        integration_time = TCS34725_INTEGRATIONTIME_24MS;
    else if (strcmp(integration_time_str, "50ms") == 0)
        integration_time = TCS34725_INTEGRATIONTIME_50MS;
    else if (strcmp(integration_time_str, "101ms") == 0)
        integration_time = TCS34725_INTEGRATIONTIME_101MS;
    else if (strcmp(integration_time_str, "240ms") == 0)
        integration_time = TCS34725_INTEGRATIONTIME_240MS;
    else if (strcmp(integration_time_str, "600ms") == 0)
        integration_time = TCS34725_INTEGRATIONTIME_600MS;
    else {
        printf("Invalid integration time.\n");
        return -1;
    }

    uint8_t gain;
    if (strcmp(gain_str, "1X") == 0)
        gain = TCS34725_GAIN_1X;
    else if (strcmp(gain_str, "4X") == 0)
        gain = TCS34725_GAIN_4X;
    else if (strcmp(gain_str, "16X") == 0)
        gain = TCS34725_GAIN_16X;
    else if (strcmp(gain_str, "60X") == 0)
        gain = TCS34725_GAIN_60X;
    else if (strcmp(gain_str, "64X") == 0)
        gain = TCS34725_GAIN_64X;
    else {
        printf("Invalid gain.\n");
        return -1;
    }

    int handle = i2cOpen(1, TCS34725_ADDR, 0);
    if (handle < 0){
        printf("Failed to open I2C. Error: %d\n", handle);
        return -1;
    }

    uint8_t enable = TCS34725_PON | TCS34725_AEN;
    if (i2cWriteByteData(handle, TCS34725_CMD | TCS34725_ENABLE, enable) < 0){
        printf("Failed to enable sensor.\n");
        i2cClose(handle);
        return -1;
    }
    usleep(SENSOR_ENABLE_DELAY_US);

    if (i2cWriteByteData(handle, TCS34725_CMD | TCS34725_ATIME, integration_time) < 0){
        printf("Failed to set integration time.\n");
        i2cClose(handle);
        return -1;
    }

    if (i2cWriteByteData(handle, TCS34725_CMD | TCS34725_CONTROL, gain) < 0){
        printf("Failed to set gain.\n");
        i2cClose(handle);
        return -1;
    }

    return handle;
}

// Read raw color data from the sensor
int read_color_data(int handle, uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* clear)
{
    int clearLow = i2cReadByteData(handle, TCS34725_CMD | TCS34725_CDATAL);
    int clearHigh = i2cReadByteData(handle, TCS34725_CMD | TCS34725_CDATH);
    if (clearLow < 0 || clearHigh < 0) return -1;
    *clear = (clearHigh << 8) | clearLow;

    int redLow = i2cReadByteData(handle, TCS34725_CMD | TCS34725_RDATAL);
    int redHigh = i2cReadByteData(handle, TCS34725_CMD | TCS34725_RDATH);
    if (redLow < 0 || redHigh < 0) return -1;
    *r = (redHigh << 8) | redLow;

    int greenLow = i2cReadByteData(handle, TCS34725_CMD | TCS34725_GDATAL);
    int greenHigh = i2cReadByteData(handle, TCS34725_CMD | TCS34725_GDATH);
    if (greenLow < 0 || greenHigh < 0) return -1;
    *g = (greenHigh << 8) | greenLow;

    int blueLow = i2cReadByteData(handle, TCS34725_CMD | TCS34725_BDATAL);
    int blueHigh = i2cReadByteData(handle, TCS34725_CMD | TCS34725_BDATH);
    if (blueLow < 0 || blueHigh < 0) return -1;
    *b = (blueHigh << 8) | blueLow;

    return 0;
}

// Convert RGB to HSV color space
hsv_t rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b)
{
    float rf = r / 255.0f;
    float gf = g / 255.0f;
    float bf = b / 255.0f;

    float max = fmaxf(rf, fmaxf(gf, bf));
    float min = fminf(rf, fminf(gf, bf));
    float delta = max - min;

    hsv_t hsv;
    hsv.brightness = max;

    if (delta == 0.0f){
        hsv.hue = 0.0f;
        hsv.saturation = 0.0f;
        return hsv;
    }

    hsv.saturation = delta / max;

    if (max == rf){
        hsv.hue = 60.0f * fmodf(((gf - bf) / delta), 6.0f);
    }
    else if (max == gf){
        hsv.hue = 60.0f * (((bf - rf) / delta) + 2.0f);
    }
    else {
        hsv.hue = 60.0f * (((rf - gf) / delta) + 4.0f);
    }

    if (hsv.hue < 0.0f){
        hsv.hue += 360.0f;
    }

    return hsv;
}

// Match HSV values to predefined color ranges
color_match_t get_hsb_color(float hue, float saturation, float brightness) 
{
    color_match_t match;

    // Detect Red based on hue range
    // Typically, Red has a hue around 0° or 360°
    if ((hue >= 0.0f && hue < 25.0f) || (hue >= 330.0f && hue <= 360.0f)) 
    {
        match.color = COLOR_RED;
        // Confidence based on saturation and brightness
        match.confidence = (saturation * brightness) * 100.0f;
    }
    else 
    {
        match.color = COLOR_UNKNOWN;
        match.confidence = 0.0f;
    }

    // Ensure confidence doesn't exceed 100%
    if (match.confidence > 100.0f)
        match.confidence = 100.0f;

    return match;
}

// Set LED brightness using PWM
int set_led_brightness(int gpio, int percentage)
{
    if (percentage < 0 || percentage > 100) return -1;

    if (gpioSetMode(gpio, PI_PWM_OUTPUT) < 0) return -1;

    int pwmValue = (percentage * 255) / 100; // Convert percentage to PWM value (0-255)
    if (gpioPWM(gpio, pwmValue) < 0) return -1;

    return 0;
}

// Apply gamma correction to a value
uint8_t apply_gamma(float value, float gamma)
{
    float corrected = powf(value / 255.0f, gamma) * 255.0f;
    corrected = corrected > 255.0f ? 255.0f : (corrected < 0.0f ? 0.0f : corrected);
    return (uint8_t)corrected;
}

// Read average color data over multiple readings
int read_average_color_data(int handle, uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* clear, int num_readings, int delay_us)
{
    uint32_t sum_r = 0, sum_g = 0, sum_b = 0, sum_clear = 0;

    for (int i = 0; i < num_readings; i++)
    {
        if (read_color_data(handle, r, g, b, clear) < 0)
            return -1;
        sum_r += *r;
        sum_g += *g;
        sum_b += *b;
        sum_clear += *clear;
        usleep(delay_us);
    }

    *r = sum_r / num_readings;
    *g = sum_g / num_readings;
    *b = sum_b / num_readings;
    *clear = sum_clear / num_readings;

    return 0;
}

// Set sensor gain dynamically
int set_sensor_gain(int handle, const char *gain_str)
{
    uint8_t gain;
    if (strcmp(gain_str, "1X") == 0)
        gain = TCS34725_GAIN_1X;
    else if (strcmp(gain_str, "4X") == 0)
        gain = TCS34725_GAIN_4X;
    else if (strcmp(gain_str, "16X") == 0)
        gain = TCS34725_GAIN_16X;
    else if (strcmp(gain_str, "60X") == 0)
        gain = TCS34725_GAIN_60X;
    else if (strcmp(gain_str, "64X") == 0)
        gain = TCS34725_GAIN_64X;
    else
        return -1;

    if (i2cWriteByteData(handle, TCS34725_CMD | TCS34725_CONTROL, gain) < 0)
        return -1;

    return 0;
}

// Detect color by reading sensor data and processing it
const char* detect_color(int handle)
{
    uint16_t r, g, b, clear;
    if (read_average_color_data(handle, &r, &g, &b, &clear, NUM_READINGS, DELAY_US) < 0)
    {
        return "Error";
    }

    // Avoid division by zero
    if (clear == 0)
    {
        return "Error";
    }

    // Normalize RGB values based on clear channel
    float redNorm = ((float)r / (float)clear) * 255.0f;
    float greenNorm = ((float)g / (float)clear) * 255.0f;
    float blueNorm = ((float)b / (float)clear) * 255.0f;

    // Clamp values to [0, 255]
    redNorm = (redNorm > 255.0f) ? 255.0f : redNorm;
    greenNorm = (greenNorm > 255.0f) ? 255.0f : greenNorm;
    blueNorm = (blueNorm > 255.0f) ? 255.0f : blueNorm;

    // Apply gamma correction
    float gamma = 2.2f; // Standard gamma value for better color accuracy
    uint8_t red = apply_gamma(redNorm, gamma);
    uint8_t green = apply_gamma(greenNorm, gamma);
    uint8_t blue = apply_gamma(blueNorm, gamma);

    // Convert RGB to HSV
    hsv_t hsv = rgb_to_hsv(red, green, blue);

    // Get the matched color
    color_match_t match = get_hsb_color(hsv.hue, hsv.saturation, hsv.brightness);

    // Return the detected color name
    return color_names[match.color];
}

// New Function: Detect color and adjust LED brightness
int detect_and_adjust_led(int handle, int *result) {
    const char* color = detect_color(handle);
    
    if (strcmp(color, "Error") == 0) {
        printf("Failed to read color data.\n");
        // Optionally, set LED to a default brightness or take other actions
        return -1;
    }

    // Adjust brightness based on detected color
    if (strcmp(color, "Red") == 0) {
        // Set LED brightness to 50%
        if (set_led_brightness(LED_PIN, 50) < 0) {
            return -1;
        }

        *result = 1;
    }
    else if (strcmp(color, "Unknown") == 0) {
        // Set LED brightness to 100%
        if (set_led_brightness(LED_PIN, 100) < 0) {
            return -1;
        }

        *result = 0;
    }

    return 0;
}