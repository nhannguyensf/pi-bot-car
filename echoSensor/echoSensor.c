#include <stdio.h>
#include <pigpio.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>

// GPIO Pins for 5 sensors
#define NUM_SENSORS 4

// Sensor positions
#define LEFT_SENSOR 0
#define FRONT_LEFT_SENSOR 1
#define FRONT_CENTER_SENSOR 2
#define FRONT_RIGHT_SENSOR 3
#define RIGHT_SENSOR 4

// Distance thresholds
#define OBJECT_DETECT_THRESHOLD 20.0   // Distance to detect object (cm)
#define SAFE_FOLLOW_DISTANCE 15.0      // Distance to maintain from object (cm)
#define MIN_SIDE_DISTANCE 10.0         // Minimum safe side distance (cm)

typedef struct {
    int trig;
    int echo;
    int sensorId;
} SensorPins;

// Array of sensor pins
static const SensorPins sensorPins[NUM_SENSORS] = {
    {4, 5, LEFT_SENSOR},          // Left side sensor
    {6, 13, FRONT_LEFT_SENSOR},    // Front-left sensor
    {26, 12, FRONT_CENTER_SENSOR},   // Front-center sensor
    {20, 21, FRONT_RIGHT_SENSOR},   // Front-right sensor
    // {25, 16, RIGHT_SENSOR}          // Right side sensor
};

// Global variables
static double sensorDistances[NUM_SENSORS] = {0};
static pthread_mutex_t distanceMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t threads[NUM_SENSORS];
static bool isRunning = false;

// Global variables for object detection
typedef struct {
    bool object_detected;
    bool front_blocked;
    double front_distances[3];  // Left, Center, Right front sensors
    double side_distances[2];   // Left and Right side sensors
    int recommended_direction;  // -1 for left, 1 for right
} ObjectDetectionState;

static ObjectDetectionState detection_state = {0};
static pthread_mutex_t object_mutex = PTHREAD_MUTEX_INITIALIZER;

// Function declarations
static double getDistance(const SensorPins* sensor);
static void* sensorThread(void* arg);
static void getAllDistances(double* distances);
void updateObjectDetection(double distances[NUM_SENSORS]);
void getObjectDetectionState(ObjectDetectionState* state);

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

// Check surroundings and update object detection state
void updateObjectDetection(double distances[NUM_SENSORS]) {
    pthread_mutex_lock(&object_mutex);
    
    // Update front sensor readings
    detection_state.front_distances[0] = distances[FRONT_LEFT_SENSOR];
    detection_state.front_distances[1] = distances[FRONT_CENTER_SENSOR];
    detection_state.front_distances[2] = distances[FRONT_RIGHT_SENSOR];
    
    // Update side sensor readings
    detection_state.side_distances[0] = distances[LEFT_SENSOR];
    detection_state.side_distances[1] = distances[RIGHT_SENSOR];
    
    // Check if any front sensor detects an object
    detection_state.front_blocked = false;
    for (int i = 0; i < 3; i++) {
        if (detection_state.front_distances[i] < OBJECT_DETECT_THRESHOLD) {
            detection_state.front_blocked = true;
            break;
        }
    }
    
    // Determine if we're in object avoidance mode
    detection_state.object_detected = detection_state.front_blocked;
    
    // Determine best direction to avoid
    if (detection_state.object_detected) {
        // Choose direction with more space
        if (detection_state.side_distances[0] > detection_state.side_distances[1] && 
            detection_state.side_distances[0] > MIN_SIDE_DISTANCE) {
            detection_state.recommended_direction = -1;  // Go left
        } else if (detection_state.side_distances[1] > MIN_SIDE_DISTANCE) {
            detection_state.recommended_direction = 1;   // Go right
        } else {
            detection_state.recommended_direction = 0;   // No clear path
        }
    }
    
    pthread_mutex_unlock(&object_mutex);
}

// Get current object detection state
void getObjectDetectionState(ObjectDetectionState* state) {
    pthread_mutex_lock(&object_mutex);
    *state = detection_state;
    pthread_mutex_unlock(&object_mutex);
}

// int main() {
//     // Initialize the sensors
//     if(initEchoSensors() < 0) {
//         printf("Failed to initialize echo sensors\n");
//         return 1;
//     }
    
//     // Main loop to read and display distances
//     double distances[NUM_SENSORS];
//     ObjectDetectionState state;
//     while(1) {
//         if(getCurrentDistances(distances) == 0) {
//             printf("Distances: [");
//             for(int i = 0; i < NUM_SENSORS; i++) {
//                 if(distances[i] < 0) {
//                     printf("NaN");
//                 } else {
//                     printf("%.2f", distances[i]);
//                 }
//                 if(i < NUM_SENSORS - 1) printf(", ");
//             }
//             printf("] cm\n");
            
//             updateObjectDetection(distances);
//             getObjectDetectionState(&state);
            
//             if(state.object_detected) {
//                 printf("Object detected! Recommended direction: %d\n", state.recommended_direction);
//             }
//         }
//         sleep(1);
//     }
//     cleanupEchoSensors();
//     return 0;
// }
