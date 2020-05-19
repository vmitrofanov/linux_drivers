#
# author: vmitrofanov
#
obj-m += x100_kernel.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.order *.symvers *.mod.c
