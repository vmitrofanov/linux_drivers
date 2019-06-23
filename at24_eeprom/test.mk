TARGET := test_eeprom.elf
SRCS := test_eeprom.c
OBJS := $(SRCS:.c=.o)
FLAGS := -Wall -Werror
CC := gcc

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(FLAGS) $^ -o $(TARGET)

.c.o:
	$(CC) -c $^

.PHONY: clean
clean:
	rm -rf $(TARGET) $(OBJS)
