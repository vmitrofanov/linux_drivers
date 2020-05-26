obj-m += eth.o

eth-objs := bb_eth.o bb_drv.o bb_mdio.o

ARCH := arm
CROSS_COMPILE := arm-linux-gnueabi-

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C ../../linux M=$(PWD) /*C=1*/ modules

clean:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C ../../linux M=$(PWD) clean
