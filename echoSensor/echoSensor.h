#ifndef ECHO_SENSOR_H
#define ECHO_SENSOR_H

#include <stdbool.h>

#define NUM_SENSORS 5

typedef struct {
    bool object_detected;
    bool front_blocked;
    double front_distances[3];
    double side_distances[2];
    int recommended_direction; // -1 for left, 1 for right, 0 for no clear path
} ObjectDetectionState;

int initEchoSensors();
void cleanupEchoSensors();
int getCurrentDistances(double distances[NUM_SENSORS]);
void updateObjectDetection(double distances[NUM_SENSORS]);
void getObjectDetectionState(ObjectDetectionState* state);
void printSensorDistances();

#endif