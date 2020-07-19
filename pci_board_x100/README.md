# Structure
/driver_kernel - Linux kernel driver. This is a UIO driver with access to PCI mapped resources.
/include - headers
/src - Wrapper to get access to UIO object. Examples/Tests 

# Quick start
1. Call make from /driver_kernel directory
2. Call insmod x100_kernel.ko
3. In user application call ```x100_init("path_to_uio");```
4. Joy

# How to
You can see an example in /src/x100_test.c
