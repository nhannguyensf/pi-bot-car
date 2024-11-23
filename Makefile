CC = gcc
CFLAGS = -Wall -O2 -DUSE_BCM2835_LIB
LDFLAGS = -lpigpio -lpthread -lbcm2835 -lm -lrt

SRC = \
	motor/DEV_Config.c \
	motor/MotorDriver.c \
	motor/PCA9685.c \
    ls7336r.c \
	motor.c \
	car.c

OBJ = $(SRC:.c=.o)

INCLUDES = \
	-I./motor

LIBS = \
	-lbcm2835 -lm -lpthread -lpigpio -lrt

TARGET = car

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

ls7336r: ls7336r.c
	$(CC) $(CFLAGS) $(INCLUDES) -o ls7336r ls7336r.c $(LDFLAGS)
