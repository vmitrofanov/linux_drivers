/**
 * author: vmitrofanov
 */
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include "x100_driver.h"
#include "x100_map.h"

#define FRAME_SIZE                                      0x04000000
#define PHY_ADDR_MASK                                   0xE0000000
#define REG_MAPPING_VAL                                 0x002F0000
#define X100_RUN_ADDRESS_CELL                           0xB801FFFC
#define BAR_COUNT                                       3
#define PCI_ADDRESS_SPACE_LEN                           0x04000000
#define PCI_CONTROLLER_ADDR                             0x001F0000

static void x100_set_bar1_frame_pointer(unsigned int physical_address);

static pci_t pci_device[BAR_COUNT];         /* x100 access points */
static unsigned int bar1_frame_pointer = 0; /* Used for inner usage */
static int is_driver_init = -1;
static int device_file_disc = -1;

int x100_init( char *uio_path )
{
    unsigned int i = 0;
    
    if( is_driver_init == 0 )
    {
        return 0;
    }

    for( i = 0; i < BAR_COUNT; ++i )
    {
        device_file_disc = open( uio_path, O_RDWR | O_SYNC );
        if( device_file_disc < 0 )
        {
            return -1;
        }

        pci_device[i].addr = mmap( i,
                                   PCI_ADDRESS_SPACE_LEN,
                                   PROT_READ | PROT_WRITE, 
                                   MAP_SHARED, 
                                   device_file_disc, 
                                   0 );
        
        if( pci_device[i].addr == MAP_FAILED )
        {
            return -2;
        }
    }
    is_driver_init = 0;
    
    return 0;
}

int x100_deinit( void )
{
    unsigned int i = 0;
    int unmap_result = -1;
    int close_result = -1;
    
    for( i = 0; i < BAR_COUNT; ++i )
    {
        unmap_result = munmap( pci_device[i].addr, PCI_ADDRESS_SPACE_LEN );
        close_result = close( device_file_disc );
        if( ( unmap_result < 0 ) || ( close_result < 0 ) )
        {
            return -1;
        }

        pci_device[i].addr = 0;
    }  
    is_driver_init = -1;
    
    return 0;
}

int x100_read( unsigned int bar, unsigned int addr, unsigned int *value )
{
    if( bar > BAR_COUNT )
    {
        return -1;
    }

    *value = *( unsigned int * ) ( pci_device[bar].addr + addr );

    return 0;
}

int x100_write( unsigned int bar, unsigned int addr, unsigned int value )
{
    if( bar > BAR_COUNT )
    {
        return -1;
    }

    *( unsigned int * ) ( pci_device[bar].addr + addr ) = value;

    return 0;
}

int x100_read_pci_cntrl( unsigned int addr, unsigned int *value )
{
    return x100_read( 0, PCI_CONTROLLER_ADDR | addr, value );
}

int x100_write_pci_cntrl( unsigned int addr, unsigned int value )
{
    return x100_write( 0, PCI_CONTROLLER_ADDR | addr, value );
}

void x100_run_program( unsigned int run_address )
{
    unsigned int phy_addr = 0;

    if( x100_bar1_translate_addr( X100_RUN_ADDRESS_CELL, &phy_addr ) )
    {
        printf( "x100 :: [ERROR] can't translate \n" );
        return;
    }
    x100_write( 1, phy_addr, run_address );
}

int x100_bar1_translate_addr( unsigned int virt, unsigned int *phy )
{
    unsigned int addr = ~PHY_ADDR_MASK & virt;
    unsigned int frame_pointer = 0;
    
    if( addr % 4 )
    {
        return -1;
    }

    if( ( addr > bar1_frame_pointer + FRAME_SIZE ) || ( addr < bar1_frame_pointer ) )
    {
        if( addr <= FRAME_SIZE / 2 )
        {
            frame_pointer = 0;
        }
        else
        {
            frame_pointer = addr - FRAME_SIZE / 2;
        }
        
        x100_set_bar1_frame_pointer( frame_pointer );
    }

    *phy = addr - bar1_frame_pointer;

    return 0;
}



void x100_set_bar1_frame_pointer( unsigned int physical_address )
{

    x100_write( 0, REG_MAPPING_VAL | PCI_CTRL_IR_TARGET, ~PHY_ADDR_MASK & physical_address );

    /* Used for inner usage */
    bar1_frame_pointer = ~PHY_ADDR_MASK & physical_address;
}

int x100_enable_interrupt( Interrupt_t interrupt )
{
    unsigned int maskr = 0;
    
    if( ( is_driver_init < 0 ) || ( interrupt & ~int_all ) )
    {
        return -1;
    }
    
    if( x100_read_pci_cntrl( PCI_CTRL_MASKR_PCI, &maskr ) )
    {
        return -1;
    }
    
    return x100_write_pci_cntrl( PCI_CTRL_MASKR_PCI, maskr | interrupt );
}

int x100_disable_interrupt( Interrupt_t interrupt )
{
    unsigned int maskr = 0;
    
    if( ( is_driver_init < 0 ) || ( interrupt & ~int_all ) )
    {
        return -1;
    }
    
    if( x100_read_pci_cntrl( PCI_CTRL_MASKR_PCI, &maskr ) )
    {
        return -1;
    }
    
    return x100_write_pci_cntrl( PCI_CTRL_MASKR_PCI, maskr & ~interrupt );
}
