/**
 * author: mvv
 */
/* Include -------------------------------------------------------------------*/
#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>
#include <asm/io.h>
#include "../include/x100_driver.h"
#include "../include/x100_map.h"

/* Defines -------------------------------------------------------------------*/
#define VENDOR_ID                       0xA842
#define DEVICE_ID                       0xC902
#define PCI_CTRL_REGS_OFFSET            0x001F0000
#define MOD_NAME                        "x100"
#define MOD_VERSION                     "0.0.3"  
#define MOD_LICENSE                     "GPL"
#define MOD_AUTHOR                      "mvv"

/* Macro functions -----------------------------------------------------------*/
#define PCI_CTRL( reg_offset ) (unsigned int *)((unsigned char *) \
                                dev_info->mem[0].internal_addr +  \
                                PCI_CTRL_REGS_OFFSET + ( reg_offset ) )

/* Prototypes ----------------------------------------------------------------*/
static int x100_pci_probe( struct pci_dev *dev, const struct pci_device_id *id );
static void x100_pci_remove( struct pci_dev *dev );
static int irq_enable_disable_handler( struct uio_info *dev_info, s32 irq_on );
static irqreturn_t irq_handler( int irq, struct uio_info *dev_info );

/* Variables -----------------------------------------------------------------*/
static struct pci_device_id pci_ids[] = {
    { PCI_DEVICE( VENDOR_ID, DEVICE_ID ), },
    { 0, }
};

static struct pci_driver x100_pci_device_structure = {
    .probe = x100_pci_probe,
    .name = MOD_NAME,
    .id_table = pci_ids,
    .remove = x100_pci_remove,
};

/* Module macros -------------------------------------------------------------*/
MODULE_DEVICE_TABLE( pci, pci_ids );
MODULE_LICENSE( MOD_LICENSE );
MODULE_AUTHOR( MOD_AUTHOR );

/* Functions -----------------------------------------------------------------*/
static int irq_enable_disable_handler( struct uio_info *dev_info, s32 irq_on )
{
    unsigned int *maskr = PCI_CTRL( PCI_CTRL_MASKR_PCI );
    
    if( irq_on == 0 )
    {
        /* Disable all IRQs */
        *maskr = 0x0;
    }
    else
    {
        /* Enable all IRQs */
        *maskr = int_all;
    }

    return 0;
}

static irqreturn_t irq_handler( int irq, struct uio_info *dev_info )
{
   volatile unsigned int *irq_status = PCI_CTRL( PCI_CTRL_QSTR_PCI );
   volatile unsigned int *mbr_cpu = PCI_CTRL( PCI_CTRL_MBR_CPU );     
    
    if( ( *irq_status ) == 0 )
    {
        return IRQ_NONE;
    }
    
    /* Read MBR_CPU to clear interrupt */
    (void)*mbr_cpu;  
    
    return IRQ_HANDLED;
}

static int x100_pci_probe( struct pci_dev *pci_dev, const struct pci_device_id *id )
{
    struct uio_info *info = NULL;

    info = kzalloc( sizeof (struct uio_info ), GFP_KERNEL );
    if( !info )
    {
        return -ENOMEM;
    }

    if( pci_enable_device( pci_dev ) )
    {
        dev_err( &pci_dev->dev, "[FAIL] enable device\n" );
        goto out_free;
    }

    dev_info( &pci_dev->dev, "vendor - %x\n", pci_dev->vendor );
    dev_info( &pci_dev->dev, "device - %x\n", pci_dev->device );
    if( pci_request_regions( pci_dev, MOD_NAME ) )
    {
        dev_err( &pci_dev->dev, "[FAIL] request regio\n" );
        goto out_disable;
    }
    
    info->mem[0].addr = pci_resource_start( pci_dev, 0 );
    if( !info->mem[0].addr )
    {
        dev_err( &pci_dev->dev, "[FAIL] get start address\n" );
        goto out_release;
    }

    info->mem[0].internal_addr = pci_ioremap_bar( pci_dev, 0 );
    if( !info->mem[0].internal_addr )
    {
        dev_err( &pci_dev->dev, "[FAIL] mmap\n" );
        goto out_release;
    }

    info->mem[0].addr = pci_resource_start( pci_dev, 0 );
    info->mem[0].size = pci_resource_len( pci_dev, 0 );
    info->mem[0].memtype = UIO_MEM_PHYS;
    info->mem[1].addr = pci_resource_start( pci_dev, 1 );
    info->mem[1].size = pci_resource_len( pci_dev, 1 );
    info->mem[1].memtype = UIO_MEM_PHYS;

    info->name = MOD_NAME;
    info->version = MOD_VERSION;
    info->irq = pci_dev->irq;
    info->irq_flags = IRQF_SHARED;
    info->handler = irq_handler;
    info->irqcontrol = irq_enable_disable_handler;

    if( uio_register_device( &pci_dev->dev, info ) )
    {
        dev_err( &pci_dev->dev, "[FAIL] register device\n" );
        goto out_unmap;
    }

    pci_set_drvdata( pci_dev, info );

    dev_info( &pci_dev->dev, "[OK] module install\n" );
    
    return 0;

out_unmap:
    iounmap( info->mem[0].internal_addr );

out_release:
    pci_release_regions( pci_dev );

out_disable:
    pci_disable_device( pci_dev );

out_free:
    kfree( info );

    dev_err( &pci_dev->dev, "[FAIL] module install\n" );
    
    return -ENODEV;
}

static void x100_pci_remove( struct pci_dev *pci_dev )
{
    struct uio_info *info = NULL;
    
    info = pci_get_drvdata( pci_dev );
    uio_unregister_device( info );
    pci_release_regions( pci_dev );
    pci_disable_device( pci_dev );

    pci_set_drvdata( pci_dev, NULL );
    iounmap( info->mem[0].internal_addr );

    kfree( info );
    
    dev_info( &pci_dev->dev, "[OK] module remove\n" );
}

static int __init x100_init_module( void )
{
    return pci_register_driver( &x100_pci_device_structure );
}

static void __exit x100_exit_module( void )
{
    pci_unregister_driver( &x100_pci_device_structure );
}

module_init( x100_init_module );
module_exit( x100_exit_module );
