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
#define BB_MDIO_NAME            "mdio"//"4a101000.mdio"
#define BB_PHY_NAME             "bb_mdiobus:0:00"//"4a101000.mdio:00"

#define BB_MDIO_OFFSET			0x1000

struct gemac_private
{
	/* Common driver resources */
    struct net_device *ndev;
    struct platform_device *pdev;
    struct phy_device *phy;
    struct bb_mdio mdio;

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
