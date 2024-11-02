DIR_BIN = ./bin
DIR_Config = ./
DIR_MotorDriver = ./
DIR_PCA9685 = ./

OBJ_C = $(wildcard *.c)
OBJ_O = $(patsubst %.c,${DIR_BIN}/%.o,$(notdir ${OBJ_C}))

TARGET = assignment3

CC = gcc

DEBUG = -g -O0 -Wall
CFLAGS += $(DEBUG)

USELIB = USE_BCM2835_LIB
DEBUG = -D $(USELIB)
ifeq ($(USELIB), USE_BCM2835_LIB)
    LIB = -lbcm2835 -lm
endif

${TARGET}:${OBJ_O}
	$(CC) $(CFLAGS) $(OBJ_O) -o $@ $(LIB)

${DIR_BIN}/%.o : %.c
	$(CC) $(CFLAGS) -c  $< -o $@ $(LIB) -I $(DIR_Config) -I $(DIR_MotorDriver) -I $(DIR_PCA9685)

mkdir:
	@mkdir -p $(DIR_BIN)

run: ${TARGET}
	./${TARGET}
	
.PHONY: clean
clean :
	rm -f $(DIR_BIN)/*.o 
	rm -f $(TARGET)
