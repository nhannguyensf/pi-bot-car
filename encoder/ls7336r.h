/***********************************************************
* Class: CSC-615-01 Spring 2022
* Name: Robert Bierman
* Student ID: 
* GitHub ID: bierman
* Project: CSC615 Motor Encoder Board with LS7366R
*
* File: ls7336r.h
*
* Description: This file contains routine and a testbed for
*    using the LS7336 Quadrature Encode chip.
***********************************************************/
#ifndef LS7336R_H
#define LS7336R_H

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

// Declare the global variables as extern
extern unsigned char BYTE_MODE[];
extern unsigned char setMDR0[];
extern unsigned char setMDR1[];
extern unsigned char clearStatus[];
extern unsigned char clearCounter[];
extern unsigned char readCounterMsg[];

// Function prototypes
int readLS7336RCounter(int ChipEnable);
int clearLS7336RCounter(int ChipEnable);
int initLS7336RChip(int ChipEnable);

#endif // LS7336R_H
