obj-m += eeprom.o

all:
	make -C ../../linux M=$(shell pwd) modules

clean:
	make -C ../../linux M=$(shell pwd) clean
