#include "car.h"
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <pigpio.h>
#include <bcm2835.h> // Include bcm2835 library for SPI
#include "motor/PCA9685.h"
#include "motor/DEV_Config.h"
#include "motor/MotorDriver.h"
#include "motor/Debug.h"
#include "ls7336r.h"
#include "motor.h"
#include <signal.h>

// Define missing constants
#define SPI0_CE0 8 // GPIO pin for Chip Enable (adjust as needed)
#define SPI0_CE1 7
#define READ_STATUS		0x70
#define WRITE_MODE0		0x88
#define WRITE_MODE1		0x90
#define READ_MODE0		0x48
#define READ_MODE1		0x50

/*  Modes  */
#define FOURX_COUNT		0x03

#define FOURBYTE_COUNTER	0x00
#define THREEBYTE_COUNTER	0x01
#define TWOBYTE_COUNTER		0x02
#define ONEBYTE_COUNTER		0x03

volatile sig_atomic_t stop = 0; // Flag to indicate program termination (ctrl + c)

// Signal handler to stop the motor safely and set stop flag
void Handler(int signo)
{
    // System Exit
    printf("\r\nHandler: Motor Stop\r\n");
    stop = 1;
    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);
}

void handleSignal(int signo) {
    printf("\nHandler: Stopping motors and exiting program.\n");
    stop = 1;
    stopMotors();
}


/*int main(void)
{
    int ret;

    // Handle ctrl + c signal
    if (signal(SIGINT, Handler) == SIG_ERR)
    {
        fprintf(stderr, "Error: Cannot handle SIGINT\n");
        return 1;
    }

    printf("Initializing motor system...\n");

    // 1. System Initialization
    if (DEV_ModuleInit() != 0)
    {
        fprintf(stderr, "Error: Failed to initialize system module\n");
        return 1;
    }

    // Initialize pigpio library
    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "Error: Failed to initialize pigpio\n");
        return 1;
    }


    // 2. Motor Initialization (PCA9685 PWM controller setup)
    Motor_Init(); // Initialize TS-25GA370H motor

    // Motor A and Motor B running forward at 50% speed (lower speed for testing)
    printf("Running motors forward at 50%% speed\n");
    Motor_Run(MOTORA, FORWARD, 50);
    Motor_Run(MOTORB, FORWARD, 50);

    ret = initLS7336RChip (SPI0_CE0);
    if (ret != 0)
    	{
    	printf ("Error initializing the LS7336R chip.  Error code: %d\n", ret);
    	return 1;
    	}
    else
    	{
    	printf ("LS7336R chip initialized successfully.\n");
    	}
    
    // Additional debug information for SPI communication
    char readModeData[2] = {0};
    ret = bbSPIXfer(SPI0_CE0, (char[]){WRITE_MODE0, FOURX_COUNT}, readModeData, 2);

    // Read back MDR0 to verify
    ret = bbSPIXfer(SPI0_CE0, (char[]){READ_MODE0, 0x00}, readModeData, 2);
    
    int lastCountA = 0;
    int countsPerRevolution = 540; 
    double wheelCircumference = 3.1415926 * 5; // Assuming 6.5 cm diameter wheel
    
    for (int j = 0; j < 20; j++)
    	{
    	// Explicitly toggle CE pin before SPI read
    	gpioWrite(SPI0_CE0, 0); // Set CE low to start communication
    	sleep(0.001); // Small delay to ensure proper state

    	int resultA = readLS7336RCounter(SPI0_CE0);
    	printf("Debug: CE Pin 8 State before SPI read operation: %d\n", gpioRead(SPI0_CE0));
    	
    	gpioWrite(SPI0_CE0, 1); // Set CE high after communication
    
    	if (resultA < 0)
    		{
    		printf("Error reading counter. ResultA: %d\n", resultA);
    		}
    	else
    		{
    		printf("Raw data from chip (read counter): %08X\n", resultA);
    		double speedA = ((resultA - lastCountA) / (double)countsPerRevolution) * wheelCircumference;
    		double revsPerSecA = (resultA - lastCountA) / (double)countsPerRevolution;
    		
    		printf ("Count: %d, Revolutions: %f, Speed: %f, delta: %d\n", 
                            resultA, revsPerSecA, speedA, resultA - lastCountA);
    		lastCountA = resultA;
    		}
    	
    	sleep (1);
    }


    ret = initLS7336RChip (SPI0_CE1);
    if (ret != 0)
    	{
    	printf ("Error initializing the LS7336R chip.  Error code: %d\n", ret);
    	return 1;
    	}
    else
    	{
    	printf ("LS7336R chip initialized successfully.\n");
    	}
    
    // Additional debug information for SPI communication
    // char readModeData[2] = {0};
    ret = bbSPIXfer(SPI0_CE1, (char[]){WRITE_MODE1, FOURX_COUNT}, readModeData, 2);

    // Read back MDR0 to verify
    ret = bbSPIXfer(SPI0_CE1, (char[]){READ_MODE1, 0x00}, readModeData, 2);
    
    int lastCountB = 0;
        
    for (int k = 0; k < 20; k++)	
    {
    	// Explicitly toggle CE pin before SPI read
    	gpioWrite(SPI0_CE1, 0); // Set CE low to start communication
    	sleep(0.001); // Small delay to ensure proper state

    	int resultB = readLS7336RCounter(SPI0_CE1);
    	printf("Debug: CE Pin 7 State before SPI read operation: %d\n", gpioRead(SPI0_CE1));
    	
    	gpioWrite(SPI0_CE1, 1); // Set CE high after communication
    
    	if (resultB < 0)
    		{
    		printf("Error reading counter. ResultB: %d\n", resultB);
    		}
    	else
    		{
    		printf("Raw data from chip (read counter): %08X\n", resultB);
    		double speedB = ((resultB - lastCountB) / (double)countsPerRevolution) * wheelCircumference;
    		double revsPerSecB = (resultB - lastCountB) / (double)countsPerRevolution;
    		
    		printf ("Count: %d, Revolutions: %f, Speed: %f, delta: %d\n", 
                            resultB, revsPerSecB, speedB, resultB - lastCountB);
    		lastCountB = resultB;
    		}
    	
    	sleep (1);
    }

    for (int i = 0; i<10; i++)
    {
        usleep(10000); // Poll every 100ms
    }

    // Stop the motors
    printf("Stopping motors\n");
    Motor_Stop(MOTORA);
    Motor_Stop(MOTORB);

    // Program finished, exit gracefully
    printf("Exiting program\n");
    gpioTerminate();
    DEV_ModuleExit();
    return 0;
}*/

int main(void) {
    if (signal(SIGINT, handleSignal) == SIG_ERR) {
        fprintf(stderr, "Error: Cannot handle SIGINT\n");
        return 1;
    }

    initializeMotorSystem();

    // Start motors
    printf("Running motors forward at 50%% speed\n");
    Motor_Run(MOTORA, FORWARD, 50);
    Motor_Run(MOTORB, FORWARD, 50);

    // Initialize encoders
    if (initializeEncoder(SPI0_CE0, "Motor A") != 0) return 1;
    if (initializeEncoder(SPI0_CE1, "Motor B") != 0) return 1;

    int lastCountA = 0, lastCountB = 0;

    for (int i = 0; i < 20 && !stop; i++) {
        readEncoder(SPI0_CE0, &lastCountA, "Motor A");
        readEncoder(SPI0_CE1, &lastCountB, "Motor B");
        sleep(1); // 1-second interval
    }

    // Stop motors and cleanup
    stopMotors();
    gpioTerminate();
    DEV_ModuleExit();
    printf("Program exited successfully.\n");

    return 0;
}