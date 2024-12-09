#ifndef ECHO_SENSOR_H
#define ECHO_SENSOR_H

// Number of sensors in the array
#define NUM_SENSORS 5

// Initialize the echo sensor system
// Returns 0 on success, -1 on failure
int initEchoSensors(void);
int getCurrentDistances(double distances[NUM_SENSORS]);

// Cleanup and stop the echo sensor system
void cleanupEchoSensors(void);

#endif // ECHO_SENSOR_H
