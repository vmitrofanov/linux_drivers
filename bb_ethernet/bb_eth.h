#ifndef _BB_ETH_H
#define _BB_ETH_H

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/phy.h>
#include "bb_mdio.h"

//#define DEBUG
#ifdef DEBUG
#define DBG(...)	pr_err(__VA_ARGS__)
#else
#define DBG(...)	while(0)
#endif

#define BB_GEMAC_CLK			"fck"

#define BB_MDIO_NAME            	"bb_mdio"
#define BB_PHY_NAME             	"bb_mdiobus:0:00"

#define BB_DMA_RING_DEFAULT_SIZE	512
#define BB_DMA_BUF_DEFAULT_SIZE		1600
#define BB_DMA_CHANNELS_NUMBER		8
#define BB_INT_CHAN_MASK		0xFF


#define BB_MDIO_OFFSET			0x1000
#define BB_WR_OFFSET			0x1200
#define BB_SWITCH_OFFSET		0x0000
#define BB_PORT_OFFSET			0x0100
#define BB_PORT0_OFFSET			(BB_PORT_OFFSET + 0x0000)
#define BB_PORT1_OFFSET			(BB_PORT_OFFSET + 0x0100)
#define BB_PORT2_OFFSET			(BB_PORT_OFFSET + 0x0200)
#define BB_DMA_OFFSET			0x0800
#define BB_ETH_STATS_OFFSET		0X0900
#define BB_DMA_STREAM_OFFSET		0x0A00
#define BB_TIME_SYNC_OFFSET		0x0C00
#define BB_ADDR_LOOKUP_OFFSET		0x0D00
#define BB_SLIVER_PORT1			0x0D80
#define BB_SLIVER_PORT2			0x0DC0


#define BB_MAC_LO_OFFSET		0x20
#define BB_MAC_HI_OFFSET		0x24

#define BB_GET_MAC_HI(mac)		(((mac)[0] << 0) | ((mac)[1] << 8) | \
			 	 	 ((mac)[2] << 16) | ((mac)[3] << 24))
#define BB_GET_MAC_LO(mac)		(((mac)[4] << 0) | ((mac)[5] << 8))

//TODO: add BB prefix
#define RX_PRIORITY_MAPPING		0x76543210
#define TX_PRIORITY_MAPPING		0x33221100
#define CPDMA_TX_PRIORITY_MAP		0x76543210

#define CPSW_VERSION_1			0x19010a
#define CPSW_VERSION_2			0x19010c
#define CPSW_VERSION_3			0x19010f
#define CPSW_VERSION_4			0x190112

struct bb_sliver_regs
{
	u32	id_ver;
	u32	mac_control;
	u32	mac_status;
	u32	soft_reset;
	u32	rx_maxlen;
	u32	__reserved_0;
	u32	rx_pause;
	u32	tx_pause;
	u32	__reserved_1;
	u32	rx_pri_map;
};

struct bb_wr_regs
{
	u32	id_ver;			//0
	u32	soft_reset;		//4
	u32	control;		//8
	u32	int_control;		//C
	u32	c0_rx_thresh_en;	//10
	u32	c0_rx_en;		//14
	u32	c0_tx_en;		//18
	u32	c0_misc_en;		//1C
	u32	c1_rx_thresh_en;	//20
	u32	c1_rx_en;		//24
	u32	c1_tx_en;		//28
	u32	c1_misc_en;		//2C
	u32	c2_rx_thresh_en;	//30
	u32	c2_rx_en;		//34
	u32	c2_tx_en;		//38
	u32	c2_misc_en;		//3C
	u32	c0_rx_thresh_stat;	//40
	u32	c0_rx_stat;		//44
	u32	c0_tx_stat;		//48
	u32	c0_misc_stat;		//4C
};

struct bb_switch_regs
{
	u32	id_ver;
	u32	control;
	u32	soft_reset;
	u32	stat_port_en;
	u32	ptype;
	u32	soft_idle;
	u32	thru_rate;
	u32	gap_thresh;
	u32	tx_start_wds;
	u32	flow_control;
	u32	vlan_ltype;
	u32	ts_ltype;
	u32	dlr_ltype;
};

struct bb_port_regs
{
	u32	control;	//0
	u32	_reserved1;	//4
	u32	max_blks;	//8
	u32	blk_cnt;
	u32	tx_in_ctl;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	ts_seq_mtype;
	u32	mac_lo;
	u32	mac_hi;
	u32	send_percent;
	u32	_reserved2;
	u32	rx_dscp_pri_map[8];

};

struct bb_dma_regs
{
	u32	tx_idver;		//0
	u32	tx_control;		//4
	u32	tx_teardown;		//8
	u32	__reserve1;		//C
	u32	rx_idver;		//10
	u32	rx_control;		//14
	u32	rx_teardown;		//18
	u32	cpdma_soft_reset;	//1C
	u32	dma_control;		//20
	u32	dma_status;		//24
	u32	rx_buffer_offset;	//28
	u32	em_control;		//2C
	u32	tx_pri_rate[8];		//30
	u32	__reserve2[12];		//50
	u32	tx_intstat_raw;		//80
	u32	tx_intstat_masked;	//84
	u32	tx_intmask_set;		//88
	u32	tx_intmask_clear;	//8C
	u32	cpdma_in_vector;	//90
	u32	cpdma_eoi_vector;	//94
	u32	__reserve3[2];		//98
	u32	rx_intstat_raw;		//A0
	u32	rx_intstat_masked;	//A4
	u32	rx_intmask_set;		//A8
	u32	rx_intmask_clear;	//AC
	u32	dma_intstat_raw;	//B0
	u32	dma_intstat_masked;	//B4
	u32	dma_intmask_set;	//B8
	u32	dma_intmask_clear;	//BC
	u32	rx_pendthresh[8];	//C0
	u32	rx_freebuffer[8];	//E0
};

struct bb_dma_stream_regs
{
	u32	tx_hdp[8];		//A00
	u32	rx_hdp[8];		//A20
	u32	tx_cp[8];		//A40
	u32	rx_cp[8];		//A60
};

struct bb_ale
{
	u32	idver;			//0
	u32	_reserved1;		//4
	u32	control;		//8
	u32	_reserved2;		//C
	u32	prescale;		//10
	u32	_reserved3;		//14
	u32	unknown_vlan;		//18
	u32	_reserved4;		//1C
	u32	tblctl;			//20
	u32	_reserved5[4];		//24 + 28 + 2C + 30
	u32	tblw2;			//34
	u32	tblw1;			//38
	u32	tblw0;			//3C
	u32	portctl[6];
};

struct bb_stats_reg
{
	int i;
};

struct bb_stats
{
	struct bb_stats_reg __iomem *regs;
};

struct bb_sliver
{
	struct bb_sliver_regs __iomem *regs;
};

struct bb_dma
{
	struct bb_dma_regs __iomem *regs;
	struct bb_dma_stream_regs __iomem *stream;
};

struct bb_switch
{
	struct bb_switch_regs __iomem *regs;
	u32 version;
};

struct bb_time_sync
{
	//TODO: Implement HW time sync support
	void __iomem *regs;
};

struct bb_addr_lookup
{
	//TODO: Implement address lookup support for switch fabric
	struct bb_ale __iomem *regs;
};

struct bb_port
{
	struct bb_port_regs __iomem *regs;
};

struct bb_wr
{
	struct bb_wr_regs __iomem *regs;
};

struct hw_desc
{
	u32 next;
	u32 buf;
	u32 len;
	u32 opt;
};

struct dma_desc
{
	/* Pointers to descriptor in the ring */
	dma_addr_t desc_dma;
	void *desc_cpu;
	/* Pointers to the buffer in descriptor ring */
	dma_addr_t buf_dma;
	void *buf_cpu;
	u32 buf_size;
	struct sk_buff *skb;
};

struct ring
{
	struct gemac_private *gemac;
	struct napi_struct napi;
	struct dma_desc *desc_ring;
	u32 cur;
	u32 dirty;
	u8 idx;
};

/**
 * struct gemac_private - a common structure to store all net device information
 */
struct gemac_private
{
	/* Common driver resources */
	struct net_device *ndev;
	struct platform_device *pdev;
	struct phy_device *phy;
	/* Memory mapped resources */
	void __iomem *gemac_regs;
	struct bb_switch eth_switch;
	struct bb_port port0;
	struct bb_port port1;
	struct bb_port port2;
	struct bb_dma dma;
	struct bb_stats stats;
	struct bb_time_sync eth_ts;
	struct bb_addr_lookup ale;
	struct bb_sliver sliver_port1;
	struct bb_sliver sliver_port2;
	struct bb_mdio mdio;
	struct bb_wr eth_wr;

	/* DT resources */
	struct resource *dt_gemac_regs;
	struct device_node *dt_phy_node;
	struct clk *dt_clk;
	char dt_mac[ETH_ALEN];
	int dt_irq_rx;
	int dt_irq_tx;
	int dt_phy_id;
	phy_interface_t dt_phy_interface;

	/* Software device resources */
	struct ring tx_ring[BB_DMA_CHANNELS_NUMBER];
	struct ring rx_ring[BB_DMA_CHANNELS_NUMBER];

	/* Driver settings */
	size_t ring_size;
	size_t buf_size;
};

#endif /* _BB_ETH_H */
