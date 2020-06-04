#ifndef _BB_DRV_H
#define _BB_DRV_H

#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/if_vlan.h>
#include <linux/etherdevice.h>
#include "bb_eth.h"

/* Descriptor mode bits */
#define CPDMA_DESC_SOP		BIT(31)
#define CPDMA_DESC_EOP		BIT(30)
#define CPDMA_DESC_OWNER	BIT(29)
#define CPDMA_DESC_EOQ		BIT(28)
#define CPDMA_DESC_TD_COMPLETE	BIT(27)
#define CPDMA_DESC_PASS_CRC	BIT(26)
#define CPDMA_DESC_TO_PORT_EN	BIT(20)
#define CPDMA_TO_PORT_SHIFT	16
#define CPDMA_DESC_PORT_MASK	(BIT(18) | BIT(17) | BIT(16))
#define CPDMA_DESC_CRC_LEN	4

#define CPDMA_EOI_RX		1
#define CPDMA_MAX_CHAN		8
#define CPDMA_PKT_MAX_LEN	0x7FF

/* Helper macros */
#define NEXT_DESC(cur_desc)	((cur_desc + 1) % 512)
#define MTU_TO_FRAME_SIZE(x) ((x) + VLAN_ETH_HLEN)
#define MTU_TO_BUF_SIZE(x) (MTU_TO_FRAME_SIZE(x) + NET_IP_ALIGN + 4)
#define ALE_TABLE_WRITE		BIT(31)

/**
 * GEMAC NAPI rx polling method
 */
int poll_rx(struct napi_struct *napi, int weight);

/**
 * GEMAC NAPI tx polling method
 */
int poll_tx(struct napi_struct *napi, int weight);

irqreturn_t rx_interrupt(int irq, void *dev_id);
void bb_enable_interrupts(struct gemac_private *gemac);
int bb_alloc_ring(struct gemac_private *gemac, struct ring *ring,
		   int alloc_buffers);
int bb_init_rings(struct gemac_private *gemac);
void bb_start_dma_engine(struct gemac_private *gemac);
void bb_stop_dma_engine(struct gemac_private *gemac);
netdev_tx_t bb_gemac_start_xmit(struct sk_buff *sk, struct net_device *ndev);

#endif
