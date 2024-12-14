CC = gcc
CFLAGS = -Wall -O2 -DUSE_BCM2835_LIB
LDFLAGS = -lpigpio -lpthread -lbcm2835 -lm -lrt

# List of source files
SRC = \
    motor/DEV_Config.c \
    motor/MotorDriver.c \
    motor/PCA9685.c \
    line-sensor/line_sensor.c \
    pid/pid.c \
    car.c

# Directory structure
BIN_DIR = bin
OBJ = $(SRC:%.c=$(BIN_DIR)/%.o)

# Include directories
INCLUDES = \
    -I./motor \
    -I./line-sensor \
    -I./pid

# Libraries
LIBS = \
    -lbcm2835 -lm -lpthread -lpigpio -lrt

# Target binary
TARGET = car

# Default target
all: $(BIN_DIR)/motor $(BIN_DIR)/line-sensor $(BIN_DIR)/pid $(TARGET)

# Create necessary directories
$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BIN_DIR)/motor:
	mkdir -p $(BIN_DIR)/motor

$(BIN_DIR)/line-sensor:
	mkdir -p $(BIN_DIR)/line-sensor

$(BIN_DIR)/pid:
	mkdir -p $(BIN_DIR)/pid

# Link object files into the final binary
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

# Compile object files
$(BIN_DIR)/%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(BIN_DIR) $(TARGET)

# Run the final binary with root permissions
run:
	sudo ./$(TARGET)
