/**
 * author: vmitrofanov
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "x100_driver.h"
#include "x100_map.h"

int x100_memory_test(unsigned int start_address, unsigned int size);

int main(void)
{
    if (x100_init( "/dev/uio0")) {
        printf("X100 :: [ERROR] initialization\n");
        return -1;
    } else {
        printf("X100 :: [SUCESS] initialization\n");
    }
    
    /* 4 is the last word of CRAM and it is used as entry point address */
    if (x100_memory_test( 0, CRAM_LEN - 4)) {
        printf("memtest :: [ERROR]\n");
    } else {
        printf("memtest :: [OK]\n");
    }
    
    x100_deinit();

    return 0;
}

int x100_memory_test(unsigned int start_address, unsigned int size)
{
    unsigned int i = 0;
    unsigned int val = 0;

    for (i = 0; i < size; i += 4)
        if (x100_write(0, start_address + i, i))
            return -1;

    for (i = 0; i < size; i += 4) {
        if (x100_read(0, start_address + i, &val))
            return -1;

        if(val != i)
            return -1;
    }

    return 0;
}
