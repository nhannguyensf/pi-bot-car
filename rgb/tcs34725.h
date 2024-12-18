#ifndef TCS34725_H
#define TCS34725_H

#include <stdint.h>

// I2C Address and Commands
#define TCS34725_ADDR         0x29
#define TCS34725_CMD          0x80
#define TCS34725_PON          0x01
#define TCS34725_AEN          0x02
#define TCS34725_ENABLE       0x00
#define TCS34725_ATIME        0x01
#define TCS34725_CONTROL      0x0F
#define TCS34725_CDATAL       0x14
#define TCS34725_CDATH        0x15
#define TCS34725_RDATAL       0x16
#define TCS34725_RDATH        0x17
#define TCS34725_GDATAL       0x18
#define TCS34725_GDATH        0x19
#define TCS34725_BDATAL       0x1A
#define TCS34725_BDATH        0x1B

// Integration Times
#define TCS34725_INTEGRATIONTIME_2_4MS  0xFF
#define TCS34725_INTEGRATIONTIME_24MS   0xF6
#define TCS34725_INTEGRATIONTIME_50MS   0xEB
#define TCS34725_INTEGRATIONTIME_101MS  0xD6
#define TCS34725_INTEGRATIONTIME_240MS  0x9C
#define TCS34725_INTEGRATIONTIME_600MS  0x06

// Gain Settings
#define TCS34725_GAIN_1X    0x00
#define TCS34725_GAIN_4X    0x01
#define TCS34725_GAIN_16X   0x02
#define TCS34725_GAIN_60X   0x03
#define TCS34725_GAIN_64X   0x06

// Delay
#define SENSOR_ENABLE_DELAY_US 2400

// Thresholds for Brightness and Saturation
#define BRIGHTNESS_LOW_THRESHOLD 0.3f
#define BRIGHTNESS_HIGH_THRESHOLD 0.7f
#define BRIGHTNESS_BLACK_THRESHOLD 0.1f
#define BRIGHTNESS_WHITE_THRESHOLD 0.9f
#define SATURATION_THRESHOLD 0.2f

// LED and Reading Configuration
#define LED_PIN 6
#define NUM_READINGS 5
#define DELAY_US 100000

// PWM Output Mode
#define PI_PWM_OUTPUT 2

// Enumerated Types for Colors
typedef enum {
    COLOR_RED,
    COLOR_UNKNOWN
} color_t;

// Structure to hold HSV values
typedef struct {
    float hue;
    float saturation;
    float brightness;
} hsv_t;

// Structure to hold color match information
typedef struct {
    color_t color;
    double confidence;
} color_match_t;

// Function Declarations
int init_TCS34725(const char *integration_time_str, const char *gain_str);
int read_color_data(int handle, uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* clear);
hsv_t rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b);
color_match_t get_hsb_color(float hue, float saturation, float brightness);
int set_led_brightness(int gpio, int percentage);
uint8_t apply_gamma(float value, float gamma);
int read_average_color_data(int handle, uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* clear, int num_readings, int delay_us);
int set_sensor_gain(int handle, const char *gain_str);
const char* detect_color(int handle);
int detect_and_adjust_led(int handle, int *result);

#endif // TCS34725_H