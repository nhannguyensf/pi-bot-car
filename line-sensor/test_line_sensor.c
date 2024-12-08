#include "line_sensor.h"
#include <bcm2835.h>
#include <stdio.h>

int main() {
    // Initialize the line sensors
    line_sensors_init();

    // Run the line sensors test
    test_line_sensors();

    // Terminate GPIO safely
    bcm2835_close();
    printf("Program terminated successfully. Stored %d readings.\n", reading_index);
    fflush(stdout);
    return 0;
}
