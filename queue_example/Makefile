TARGET := queue.o

obj-m += $(TARGET)

all:
	make -C ../../linux M=$(shell pwd) modules

clean: 
	make -C ../../linux M=$(shell pwd) clean
	
