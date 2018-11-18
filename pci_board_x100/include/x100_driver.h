/**
 * author: mvv
 */
#if !defined( X100_DRIVER_H_ )
#define X100_DRIVER_H_

/* Defines -------------------------------------------------------------------*/
#define PCI_DEVICE_RESOURCE_LEN     512

/* Types ---------------------------------------------------------------------*/
typedef enum
{
    int_sdhc_dma = 0x08000000,
    int_spi = 0x04000000,
    int_i2c = 0x02000000,
    int_video = 0x01000000,
    int_transf_to_pci_done = 0x80000000,
    int_transf_to_pci_error = 0x40000000,
    int_transf_to_pci_wmark = 0x20000000,
    int_cpu_mbr = 0x10000000,
    int_all = 0xFF000000
}Interrupt_t;

typedef struct
{
    unsigned char *addr;
} pci_t;

/* Prototypes ----------------------------------------------------------------*/
/**
 * Initialize x100 device
 * @param path to uio, usually /dev/uio0
 * @return 0 if success, -1 if failure
 */
int x100_init(char *uio_path);

/**
 * Deinitialize x100 device
 * @return 0 if success, -1 if failure
 */
int x100_deinit(void);

/**
 * Read value. Access to x100 memory.
 * @param bar Bar number: 0, 1, 2
 * @param addr memory address
 * @param value 32-bit value
 * @return 0 if success, -1 if failure
 */
int x100_read(unsigned int bar, unsigned int addr, unsigned int *value);

/**
 * Write value. Access to x100 memory.
 * @param bar Bar number: 0, 1, 2
 * @param addr memory address
 * @param value 32-bit value
 * @return 0 if success, -1 if failure
 */
int x100_write(unsigned int bar, unsigned int addr, unsigned int value);

/**
 * Read PMSC register
 * @param addr Register address
 * @param value 32-bit value
 * @return 0 if success, -1 if failure
 */
int x100_read_pmsc(unsigned int addr, unsigned int *value);

/**
 * Write to PMSC register
 * @param addr Register address
 * @param value 32-bit value
 * @return 0 if success, -1 if failure
 */
int x100_write_pmsc(unsigned int addr, unsigned int value);

/**
 * Translate virtual address to physical lay into bar1 frame
 * @param virt Virtual address
 * @param phy Physical address rely on bar1 frame pointer
 * @return 0 if success, -1 if failure
 */
int x100_bar1_translate_addr(unsigned int virt, unsigned int *phy);

/**
 * Enable x100 interrupt
 * @param an interrupt to enable
 * @return 1 - success, 0 - failure
 */
int x100_enable_interrupt( Interrupt_t interrupt );

/**
 * Disable x100 interrupt
 * @param an interrupt to disable
 * @return 1 - success, 0 - failure
 */
int x100_disable_interrupt( Interrupt_t interrupt );

#endif
