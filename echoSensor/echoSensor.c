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
    {4, 5, 0},  
    {6, 13, 1},  
    {26, 12, 2}, 
    {7, 8, 3},    
    {25, 16, 4}   
};

// Global variables
static double sensorDistances[NUM_SENSORS] = {0};
static pthread_mutex_t distanceMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t threads[NUM_SENSORS];
static bool isRunning = false;

// Function declarations
static double getDistance(const SensorPins* sensor);
static void* sensorThread(void* arg);
static void getAllDistances(double* distances);

// Initialize the echo sensor system
int initEchoSensors() {
    if(isRunning) return 0;  
    
    if(gpioInitialise() < 0) {
        printf("pigpio initialization failed\n");
        return -1;
    }
    printf("pigpio initialized successfully\n");
    
    // Initialize GPIO pins for all sensors
    for(int i = 0; i < NUM_SENSORS; i++) {
        gpioSetMode(sensorPins[i].echo, PI_INPUT);
        gpioSetMode(sensorPins[i].trig, PI_OUTPUT);
        printf("Initialized Sensor %d: Trig=%d, Echo=%d\n", 
               i, sensorPins[i].trig, sensorPins[i].echo);
        // Ensure trigger is low
        gpioWrite(sensorPins[i].trig, 0);
    }
    
    // Create threads for each sensor
    for(int i = 0; i < NUM_SENSORS; i++) {
        if(pthread_create(&threads[i], NULL, sensorThread, (void*)&sensorPins[i]) != 0) {
            printf("Failed to create thread for sensor %d\n", i);
            gpioTerminate();
            return -1;
        }
    }
    
    isRunning = true;
    return 0;
}

// Get the current distances from all sensors
// Returns 0 on success, -1 on failure
int getCurrentDistances(double distances[NUM_SENSORS]) {
    if(!isRunning) return -1;
    
    getAllDistances(distances);
    return 0;
}

// Cleanup and stop the echo sensor system
void cleanupEchoSensors() {
    if(!isRunning) return;
    
    isRunning = false;
    
    // Wait for threads to finish
    for(int i = 0; i < NUM_SENSORS; i++) {
        pthread_cancel(threads[i]);
        pthread_join(threads[i], NULL);
    }
    
    gpioTerminate();
}

static double getDistance(const SensorPins* sensor) {
    int startTick, endTick;
    
    printf("Sensor %d: Starting measurement (Trig=%d, Echo=%d)\n", 
           sensor->sensorId, sensor->trig, sensor->echo);
    
    //Send out a trigger signal
    gpioWrite(sensor->trig, 1);
    usleep(10);
    gpioWrite(sensor->trig, 0);
    
    //Wait for Echo to go up (with timeout)
    int timeout = 0;
    while(gpioRead(sensor->echo) == 0 && timeout < 30000) {
        timeout++;
        usleep(1);
    }
    if (timeout >= 30000) {
        printf("Sensor %d: Echo pin never went high (timeout) - Check Echo pin %d connection\n", 
               sensor->sensorId, sensor->echo);
        return -1;
    }
    
    //Calculate time
    startTick = gpioTick();
    
    //Wait for Echo to go down (with timeout)
    timeout = 0;
    while(gpioRead(sensor->echo) == 1 && timeout < 30000) {
        timeout++;
        usleep(1);
    }
    if (timeout >= 30000) {
        printf("Sensor %d: Echo pin never went low (timeout)\n", sensor->sensorId);
        return -1;
    }
    
    //Calculate time
    endTick = gpioTick();
    int timeElapsed = endTick - startTick;
    
    //Calculate distance
    double distance = (timeElapsed * 0.0343)/2.0;
    printf("Sensor %d: Time elapsed: %d microseconds, Distance: %.2f cm\n", 
           sensor->sensorId, timeElapsed, distance);
    return distance;
}

static void* sensorThread(void* arg) {
    SensorPins* sensor = (SensorPins*)arg;
    
    while(isRunning) {
        double dist = getDistance(sensor);
        
        pthread_mutex_lock(&distanceMutex);
        sensorDistances[sensor->sensorId] = dist;
        pthread_mutex_unlock(&distanceMutex);
        
        sleep(1);
    }
    return NULL;
}

static void getAllDistances(double* distances) {
    pthread_mutex_lock(&distanceMutex);
    for(int i = 0; i < NUM_SENSORS; i++) {
        distances[i] = sensorDistances[i];
    }
    pthread_mutex_unlock(&distanceMutex);
}

int main() {
    // Initialize the sensors
    if(initEchoSensors() < 0) {
        printf("Failed to initialize echo sensors\n");
        return 1;
    }
    
    // Main loop to read and display distances
    double distances[NUM_SENSORS];
    while(1) {
        if(getCurrentDistances(distances) == 0) {
            printf("Distances: [");
            for(int i = 0; i < NUM_SENSORS; i++) {
                if(distances[i] < 0) {
                    printf("NaN");
                } else {
                    printf("%.2f", distances[i]);
                }
                if(i < NUM_SENSORS - 1) printf(", ");
            }
            printf("] cm\n");
        }
        sleep(1);
    }
    cleanupEchoSensors();
    return 0;
}
