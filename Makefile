CC = gcc
CFLAGS = -Wall -O2 -DUSE_BCM2835_LIB
LDFLAGS = -lpigpio -lpthread -lbcm2835 -lm -lrt

SRC = \
	motor/DEV_Config.c \
	motor/MotorDriver.c \
	motor/PCA9685.c \
	encoder/ls7336r.c \
	encoder/motor.c \
	car.c \
	pid.c  # Add PID source file

BIN_DIR = bin
OBJ = $(SRC:%.c=$(BIN_DIR)/%.o)

INCLUDES = \
	-I./motor \
	-I./encoder \
	-I./pid  # Include directory for PID header

LIBS = \
	-lbcm2835 -lm -lpthread -lpigpio -lrt

TARGET = car

all: $(BIN_DIR)/motor $(BIN_DIR)/encoder $(BIN_DIR)/pid $(TARGET)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BIN_DIR)/motor:
	mkdir -p $(BIN_DIR)/motor

$(BIN_DIR)/encoder:
	mkdir -p $(BIN_DIR)/encoder

$(BIN_DIR)/pid:
	mkdir -p $(BIN_DIR)/pid

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

$(BIN_DIR)/%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(BIN_DIR) $(TARGET)

ls7336r: encoder/ls7336r.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ encoder/ls7336r.c $(LDFLAGS)

run:
	sudo ./car
