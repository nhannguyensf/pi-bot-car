#include <stdio.h>
#include <pigpio.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>

// GPIO Pins for 5 sensors
#define NUM_SENSORS 5
#define OBJECT_THRESHOLD 20.0  // Distance in cm to detect object
#define SIDE_CLEARANCE 15.0    // Minimum side clearance needed

typedef struct {
    int trig;
    int echo;
    int sensorId;
} SensorPins;

// Array of sensor pins
static const SensorPins sensorPins[NUM_SENSORS] = {
    {14, 15, 0},  // Left sensor
    {13, 21, 1},  // Left-center sensor
    {6, 26, 2},   // Center sensor
    {19, 20, 3},  // Right-center sensor
    {16, 12, 4}   // Right sensor
};

// Global variables
static double sensorDistances[NUM_SENSORS] = {0};
static pthread_mutex_t distanceMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t threads[NUM_SENSORS];
static bool isRunning = false;

// Global variables for object detection
static bool object_detected = false;
static int object_direction = 0;  // -1 for left, 1 for right
static pthread_mutex_t object_mutex = PTHREAD_MUTEX_INITIALIZER;

// Function declarations
static double getDistance(const SensorPins* sensor);
static void* sensorThread(void* arg);
static void getAllDistances(double* distances);
bool checkForObject(double distances[NUM_SENSORS], int* avoid_direction);
bool isObjectDetected();
int getAvoidanceDirection();

// Initialize the echo sensor system
int initEchoSensors() {
    if(isRunning) return 0;  
    
    if(gpioInitialise() < 0) {
        printf("pigpio initialization failed\n");
        return -1;
    }
    
    // Initialize GPIO pins for all sensors
    for(int i = 0; i < NUM_SENSORS; i++) {
        gpioSetMode(sensorPins[i].echo, PI_INPUT);
        gpioSetMode(sensorPins[i].trig, PI_OUTPUT);
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
    if (timeout >= 30000) return -1;
    
    //Calculate time
    startTick = gpioTick();
    
    //Wait for Echo to go down (with timeout)
    timeout = 0;
    while(gpioRead(sensor->echo) == 1 && timeout < 30000) {
        timeout++;
        usleep(1);
    }
    if (timeout >= 30000) return -1;
    
    //Calculate time
    endTick = gpioTick();
    int timeElapsed = endTick - startTick;
    
    //Calculate distance
    double distance = (timeElapsed * 0.0343)/2.0;  
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

// Check if there's an object in the path and determine best avoidance direction
bool checkForObject(double distances[NUM_SENSORS], int* avoid_direction) {
    pthread_mutex_lock(&object_mutex);
    
    bool found_object = false;
    
    // Check center sensors for objects
    if (distances[2] < OBJECT_THRESHOLD) {  // Center sensor detects object
        found_object = true;
        
        // Determine which direction has more clearance
        double left_space = distances[0];   // Left sensor
        double right_space = distances[4];  // Right sensor
        
        if (left_space > right_space && left_space > SIDE_CLEARANCE) {
            *avoid_direction = -1;  // Go left
        } else if (right_space > SIDE_CLEARANCE) {
            *avoid_direction = 1;   // Go right
        } else {
            *avoid_direction = 0;   // No clear path
        }
    }
    
    object_detected = found_object;
    object_direction = *avoid_direction;
    
    pthread_mutex_unlock(&object_mutex);
    return found_object;
}

// Get object detection status
bool isObjectDetected() {
    bool status;
    pthread_mutex_lock(&object_mutex);
    status = object_detected;
    pthread_mutex_unlock(&object_mutex);
    return status;
}

// Get recommended avoidance direction
int getAvoidanceDirection() {
    int direction;
    pthread_mutex_lock(&object_mutex);
    direction = object_direction;
    pthread_mutex_unlock(&object_mutex);
    return direction;
}

int main() {
    // Initialize the sensors
    if(initEchoSensors() < 0) {
        printf("Failed to initialize echo sensors\n");
        return 1;
    }
    
    // Main loop to read and display distances
    double distances[NUM_SENSORS];
    int avoid_direction;
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
            
            if(checkForObject(distances, &avoid_direction)) {
                printf("Object detected! Avoidance direction: %d\n", avoid_direction);
            }
        }
        sleep(1);
    }
    cleanupEchoSensors();
    return 0;
}
