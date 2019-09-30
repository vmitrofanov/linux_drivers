KDIR=/home/mvv/projects/linux/
MDIR=$(shell pwd)

obj-m += mem.o

all:
	make -C $(KDIR) M=$(MDIR) modules

clean:
	make -C $(KDIR) M=$(MDIR) clean
	
