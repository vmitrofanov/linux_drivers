/**
 * author: vmitrofanov
 */
#if !defined( X100_MEMORY_MAP_H_ )
#define X100_MEMORY_MAP_H_

#define CRAM                                    0x18000000
#define CRAM_LEN                                0x00020000

/* PCI_CTRL */
#define PCI_CTRL_DEVICE_VENDOR_ID               0x00000000
#define PCI_CTRL_STATUS_COMMAND                 0x00000004
#define PCI_CTRL_CLASS_REVISION                 0x00000008
#define PCI_CTRL_LATENCY_TIMER                  0x0000000C
#define PCI_CTRL_BAR0                           0x00000010
#define PCI_CTRL_BAR1                           0x00000014
#define PCI_CTRL_SUB_ID_SUB_VEND_ID             0x0000002C
#define PCI_CTRL_INTERRUPT_LINE                 0x0000003C
#define PCI_CTRL_IR_TARGET                      0x00000040
#define PCI_CTRL_SEM                            0x00000044
#define PCI_CTRL_MBR_PCI                        0x00000044
#define PCI_CTRL_CSR_PCI                        0x00000048
#define PCI_CTRL_CSR_MASTER                     0x00000050
#define PCI_CTRL_IR_MASTER                      0x00000054
#define PCI_CTRL_AR_PCI                         0x00000058
#define PCI_CTRL_QSTR_PCI                       0x0000005C
#define PCI_CTRL_MASKR_PCI                      0x00000060
#define PCI_CTRL_STATUS_MASTER                  0x00000064
#define PCI_CTRL_TMR_PCI                        0x00000068
#define PCI_CTRL_CSR_WIN                        0x0000006C
#define PCI_CTRL_MBR_CPU                        0x00000070

#endif
