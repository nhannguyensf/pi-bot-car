/***********************************************************
* Class: CSC-615-01 Spring 2022
* Name: Robert Bierman
* Student ID: 
* GitHub ID: bierman
* Project: CSC615 Motor Encoder Board with LS7366R
*
* File: ls7336r.c
*
* Description: This file contains routine and a testbed for
*    using the LS7336 Quadrature Encode chip.
***********************************************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pigpio.h>

// Commands  
#define	CLEAR_COUNTER	0x20		// 00 (WR) 100 (CNTR)
#define CLEAR_STATUS	0x30		// 00 (WR) 110 (STR)
#define READ_COUNTER	0x60
#define READ_STATUS		0x70
#define WRITE_MODE0		0x88
#define WRITE_MODE1		0x90
#define READ_MODE0		0x48
#define READ_MODE1		0x50

//  Modes  
#define FOURX_COUNT		0x03

#define FOURBYTE_COUNTER	0x00
#define THREEBYTE_COUNTER	0x01
#define TWOBYTE_COUNTER		0x02
#define ONEBYTE_COUNTER		0x03

unsigned char BYTE_MODE[] = {ONEBYTE_COUNTER, TWOBYTE_COUNTER, 
			     THREEBYTE_COUNTER, FOURBYTE_COUNTER};



#define GPIO00	0		//Physical Pin 27 (ID_SD, I2C ID - Reserved)
#define GPIO01	1		//Physical Pin 28 (ID_SC, I2C ID - Reserved)
#define GPIO02	2		//Physical Pin 3 (SDA1 I2C)
#define GPIO03	3		//Physical Pin 5 (SCL1 I2C)
#define GPIO04	4		//Physical Pin 7
#define GPIO05	5		//Physical Pin 29
#define GPIO06	6		//Physical Pin 31
#define GPIO07	7		//Physical Pin 26 (SPI0_CE1)
#define GPIO08	8		//Physical Pin 24 (SPI0_CE0)
#define GPIO09	9		//Physical Pin 21 (SPI0_MISO)
#define GPIO10	10		//Physical Pin 19 (SPI0_MOSI)
#define GPIO11	11		//Physical Pin 23 (SPI0_SCLK)
#define GPIO12	12		//Physical Pin 32
#define GPIO13	13		//Physical Pin 33
#define GPIO14	14		//Physical Pin 8 (UART0_TX)
#define GPIO15	15		//Physical Pin 10 (UART0_RX)
#define GPIO16	16		//Physical Pin 36
#define GPIO17	17		//Physical Pin 11
#define GPIO18	18		//Physical Pin 12 ((PCM_CLK))
#define GPIO19	19		//Physical Pin 35
#define GPIO20	20		//Physical Pin 38
#define GPIO21	21		//Physical Pin 40
#define GPIO22	22		//Physical Pin 15
#define GPIO23	23		//Physical Pin 16
#define GPIO24	24		//Physical Pin 18
#define GPIO25	25		//Physical Pin 22
#define GPIO26	26		//Physical Pin 37
#define GPIO27	27		//Physical Pin 13

#define SPI0_MOSI	GPIO10		//Physical Pin 19
#define SPI0_MISO	GPIO09		//Physical Pin 21
#define SPI0_SCLK	GPIO11		//Physical Pin 23

#define SPI0_CE0	GPIO08		//Physical Pin 24
#define SPI0_CE1	GPIO07		//Physical Pin 26



unsigned char setMDR0[] = {WRITE_MODE0, FOURX_COUNT};
unsigned char setMDR1[] = {WRITE_MODE1, FOURBYTE_COUNTER};
unsigned char clearStatus[] = {CLEAR_STATUS};
unsigned char clearCounter[] = {CLEAR_COUNTER};
unsigned char readCounterMsg[] = { READ_COUNTER, 0, 0, 0, 0};

/*************************************************************************
 *   LS7336R Read Counter
 *
 *  int readLS7336RCounter (int ChipEnable);
 *
 *  Parameters:
 *  	ChipEnable: is the pin number of the chip enable (chip select)
 *
 *  Return:
 *      Integer value (4 byte) of the counter
 *
 *  Note that initLS7336RChip must be called prior to reading the counter
 *************************************************************************/
 
int readLS7336RCounter (int ChipEnable)
	{
	char dataFromChip[20];
    bbSPIXfer(ChipEnable, readCounterMsg, dataFromChip, 5);
    int result = (((int)dataFromChip[1]) << 24) + (((int)dataFromChip[2]) << 16) + 
    	                        (((int)dataFromChip[3]) << 8) + ((int)dataFromChip[4]);
    return (result);
	}

/*************************************************************************
 *   LS7336R Clear Counter
 *
 *  int clearLS7336RCounter (int ChipEnable);
 *
 *  Parameters:
 *  	ChipEnable: is the pin number of the chip enable (chip select)
 *
 *  Return:
 *      Integer value 0 if success, otherwise pigpio error number
 *
 *  Note that initLS7336RChip must be called prior to reading the counter
 *************************************************************************/
 
int clearLS7336RCounter (int ChipEnable)
    {
    int ret;
    char dataFromChip[20];
    
    // Clear the counter
    ret = bbSPIXfer(ChipEnable, clearCounter, dataFromChip, 1);
    if (ret < 0) {
        printf("Error clearing counter. Error code: %d\n", ret);
    } else {
        printf("Counter cleared successfully.\n");
    }
    if (ret >= 0) // xfer succeeded
        {
        ret = 0;
        }
    return (ret);
    }
 
/*************************************************************************
 *   LS7336R init
 *
 *  int initLS7336RChip (int ChipEnable);
 *
 *  Parameters:
 *  	ChipEnable: is the pin number of the chip enable (chip select)
 *
 *  Return:
 *      Integer value 0 if success, otherwise pigpio error number
 *
 *  initLS7336RChip initializes the pigpio SPI interface,
 *      it initializes the LS7336R chip by setting MDR0 to 4x Count Mode,
 *      setting MDR1 to 4 byte counter mode, clearing the status register,
 *      and clearing the counter.
 *************************************************************************/
 
int initLS7336RChip (int ChipEnable)
	{
	int ret;
	char dataFromChip[20];
	
    ret = bbSPIOpen(ChipEnable, SPI0_MISO, SPI0_MOSI, SPI0_SCLK, 100000, 0); //open SPI
    if (ret < 0) {
        printf("Error opening SPI connection. Error code: %d\n", ret);
        return ret;
    } else {
        printf("SPI connection opened successfully.\n");
    }
    if (ret == 0)  // open succeeded
        {
    	usleep (10000);
    	//set MDR0 to 4x counter mode
        ret = bbSPIXfer(ChipEnable, setMDR0, dataFromChip, 2);  //Set MDR0 
    		if (ret < 0) {
        		printf("Error setting MDR0. Error code: %d\n", ret);
        		return ret;
    		} else {
        		printf("MDR0 set to 4x counter mode successfully.\n");
    		}
        if (ret >= 0)  //xfer succeeded
            {
            usleep (10000);
            // set MDR1 to 4 byte counter mode
            ret = bbSPIXfer(ChipEnable, setMDR1, dataFromChip, 2);  //Set MDR1 
                if (ret < 0) {
                        printf("Error setting MDR0. Error code: %d\n", ret);
                        return ret;
                } else {
                        printf("MDR0 set to 4x counter mode successfully.\n");
                }

            if (ret >= 0)  //xfer succeeded
                {
                // Clear status
                ret = bbSPIXfer(ChipEnable, clearStatus, dataFromChip, 1);
                if (ret < 0) {
                        printf("Error setting MDR0. Error code: %d\n", ret);
                        return ret;
                } else {
                        printf("MDR0 set to 4x counter mode successfully.\n");
                }
                if (ret >= 0)  //xfer succeeded
                    {
                    // Clear the counter
                    ret = clearLS7336RCounter (ChipEnable);
                if (ret < 0) {
                        printf("Error setting MDR0. Error code: %d\n", ret);
                        return ret;
                } else {
                        printf("MDR0 set to 4x counter mode successfully.\n");
                }
                    }
                }
            }
        }
        
    return (ret);
	}	
	
