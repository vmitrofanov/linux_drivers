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

#define DEBUG
#ifdef DEBUG
#define DBG(...)	pr_err(__VA_ARGS__)
#else
#define DBG(...)	while(0)
#endif

#define DT_MAC_SIZE				6
#define BB_GEMAC_CLK			"fck"
#define BB_MDIO_NAME            "mdio"
#define BB_PHY_NAME             "bb_mdiobus:0:00"

#define BB_MDIO_OFFSET			0x1000
#define BB_SWITCH_OFFSET		0x0000
#define BB_PORT_OFFSET			0x0100
#define BB_DMA_OFFSET			0x0800
#define BB_TIME_SYNC_OFFSET		0x0C00
#define BB_ADDR_LOOKUP_OFFSET	0x0D00

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
	u32	id_ver;
	u32	soft_reset;
	u32	control;
	u32	int_control;
	u32	rx_thresh_en;
	u32	rx_en;
	u32	tx_en;
	u32	misc_en;
	u32	mem_allign1[8];
	u32	rx_thresh_stat;
	u32	rx_stat;
	u32	tx_stat;
	u32	misc_stat;
	u32	mem_allign2[8];
	u32	rx_imax;
	u32	tx_imax;
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
	u32	max_blks;
	u32	blk_cnt;
	u32	tx_in_ctl;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	cpdma_tx_pri_map;
	u32	cpdma_rx_chan_map;
};

struct bb_dma_regs
{

};

struct bb_sliver
{
	struct bb_sliver_regs __iomem *regs;
};

struct bb_dma
{
	struct bb_dma_regs __iomem *regs;
};

struct bb_switch
{
	//TODO: Implement switch fabric support
};

struct bb_time_sync
{
	//TODO: Implement HW time sync support
};

struct bb_addr_lookup
{
	//TODO: Implement address lookup support for switch fabric
};

struct bb_port
{

};

struct bb_wr
{

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
    struct bb_mdio mdio;
    struct bb_dma dma;
    struct bb_sliver sliver_port1;
    struct bb_sliver sliver_port2;
    struct bb_switch eth_switch;
    struct bb_port port;
    struct bb_addr_lookup ale;
    struct bb_wr eth_wr;
    struct bb_time_sync eth_ts;

	/* DT resources */
	struct resource *dt_gemac_registers;
	struct device_node *dt_phy_node;
	struct clk *dt_clk;
	const char *dt_mac;
	int dt_irq_rx;
	int dt_irq_tx;
	int dt_phy_id;
	phy_interface_t dt_phy_interface;

	/* Hardware resources */
	void __iomem *gemac_regs;

	/* Software device resources */
	struct napi_struct napi_rx;
	struct napi_struct napi_tx;
};

#endif /* _BB_ETH_H */
