#ifndef LS7336R_H
#define LS7336R_H

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
