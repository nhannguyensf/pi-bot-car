#include <stdio.h>
#include <pigpio.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>

// GPIO Pins for 5 sensors
#define NUM_SENSORS 5

typedef struct {
    int trig;
    int echo;
    int sensorId;
} SensorPins;

// Array of sensor pins
static const SensorPins sensorPins[NUM_SENSORS] = {
    {4, 5, 0},    // Sensor 0
    {6, 13, 1},   // Sensor 1
    {26, 12, 2},  // Sensor 2
    {20, 21, 3},  // Sensor 3
    {25, 16, 4}   // Sensor 4
};

// Global variables
static double sensorDistances[NUM_SENSORS] = {0};
static pthread_mutex_t distanceMutex = PTHREAD_MUTEX_INITIALIZER;
static bool isRunning = false;
static pthread_t pollThread;

// Function declarations
static double getDistance(const SensorPins* sensor);
static void* pollSensorsSequentially(void* arg);

// Initialize the echo sensor system
int initEchoSensors() {
    if (isRunning) return 0;

    if (gpioInitialise() < 0) {
        printf("pigpio initialization failed\n");
        return -1;
    }
    printf("pigpio initialized successfully\n");

    // Initialize GPIO pins for all sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        gpioSetMode(sensorPins[i].echo, PI_INPUT);
        gpioSetMode(sensorPins[i].trig, PI_OUTPUT);
        printf("Initialized Sensor %d: Trig=%d, Echo=%d\n", 
               i, sensorPins[i].trig, sensorPins[i].echo);
        gpioWrite(sensorPins[i].trig, 0);
    }

    // Create a thread for sequential polling
    if (pthread_create(&pollThread, NULL, pollSensorsSequentially, NULL) != 0) {
        printf("Failed to create polling thread\n");
        gpioTerminate();
        return -1;
    }

    isRunning = true;
    return 0;
}

// Get the current distances from all sensors
int getCurrentDistances(double distances[NUM_SENSORS]) {
    if (!isRunning) return -1;

    pthread_mutex_lock(&distanceMutex);
    for (int i = 0; i < NUM_SENSORS; i++) {
        distances[i] = sensorDistances[i];
    }
    pthread_mutex_unlock(&distanceMutex);
    return 0;
}

// Cleanup and stop the echo sensor system
void cleanupEchoSensors() {
    if (!isRunning) return;

    isRunning = false;
    pthread_join(pollThread, NULL);
    gpioTerminate();
}

// Get the distance for a specific sensor
static double getDistance(const SensorPins* sensor) {
    int startTick, endTick;

    // Send trigger signal
    gpioWrite(sensor->trig, 1);
    usleep(10);
    gpioWrite(sensor->trig, 0);

    // Wait for Echo to go high (with timeout)
    int timeout = 0;
    while(gpioRead(sensor->echo) == 0 && timeout < 30000) {
        timeout++;
        usleep(1);
    }
    if (timeout >= 30000) return -1;

    startTick = gpioTick();

    // Wait for Echo to go low (with timeout)
    timeout = 0;
    while(gpioRead(sensor->echo) == 1 && timeout < 30000) {
        timeout++;
        usleep(1);
    }
    if (timeout >= 30000) return -1;

    endTick = gpioTick();
    
    // Calculate distance in cm
    double distance = ((endTick - startTick) * 0.0343) / 2.0;
    return distance;
}

// Poll sensors sequentially
static void* pollSensorsSequentially(void* arg) {
    while (isRunning) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            double distance = getDistance(&sensorPins[i]);
            
            pthread_mutex_lock(&distanceMutex);
            sensorDistances[i] = distance;
            pthread_mutex_unlock(&distanceMutex);
            
            usleep(50000); // 50ms delay between readings
        }
    }
    return NULL;
}

void printSensorDistances() {
    double distances[NUM_SENSORS];
    if (getCurrentDistances(distances) == 0) {
        printf("Distances: [");
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (distances[i] < 0) {
                printf("NaN");
            } else {
                printf("%.2f", distances[i]);
            }
            if (i < NUM_SENSORS - 1) {
                printf(", ");
            }
        }
        printf("] cm\n");
    } else {
        printf("Failed to read distances.\n");
    }
}
