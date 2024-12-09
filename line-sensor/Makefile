# Makefile for Line Sensor Testing with BCM2835 Library

CC = gcc
CFLAGS = -g -Wall -I. -lbcm2835

# Executable name
EXEC = test_line_sensor.out

# Source files
SRC = test_line_sensor.c line_sensor.c
HDR = line_sensor.h

# Object files
OBJ = $(SRC:.c=.o)

all: $(EXEC)

$(EXEC): $(OBJ)
	$(CC) -o $(EXEC) $(OBJ) $(CFLAGS)

%.o: %.c $(HDR)
	$(CC) -c $< $(CFLAGS)

run: $(EXEC)
	sudo ./$(EXEC)  # Add sudo to run the executable with root privileges

clean:
	rm -f $(OBJ) $(EXEC)  # Remove object files and the .out file
