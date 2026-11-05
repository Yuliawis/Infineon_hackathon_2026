# Host test for the decide module. Needs only gcc, no ModusToolbox and no board.
#
#     make        build and run
#     make clean  remove the binary

MODULE_DIR = ../../proj_cm4/modules/decide

CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -Werror -I$(MODULE_DIR)
TARGET = test_decide

.PHONY: all clean

all: $(TARGET)
	./$(TARGET)

$(TARGET): test_decide.c $(MODULE_DIR)/decide.c $(MODULE_DIR)/decide.h
	$(CC) $(CFLAGS) -o $@ test_decide.c $(MODULE_DIR)/decide.c

clean:
	rm -f $(TARGET)
