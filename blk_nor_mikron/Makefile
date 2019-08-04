TARGET := micron_25q256a

obj-m += micron_25q256a.o
micron_25q256a-objs := main.o

all:
	make -C ../../linux M=$(shell pwd) modules

clean:
	make -C ../../linux M=$(shell pwd) clean

