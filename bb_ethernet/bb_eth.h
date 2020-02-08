#ifndef _BB_ETH_H
#define _BB_ETH_H

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
#include <linux/device.h>

#define DEBUG
#ifdef DEBUG
#define DBG(...)	pr_err(__VA_ARGS__)
#else
#define DBG(...)	while(0)
#endif

#define BB_DT_MAC_SIZE		6
#define BB_GEMAC_CLK	"fck"
#define BB_PHY_NAME		"4a101000.mdio"

struct gemac_private
{
	/* Common driver resources */
	struct net_device *ndev;
	struct platform_device *pdev;
	struct platform_device *phy_platform;
	struct phy_device *phy;

	/* DT resources */
	struct resource *dt_gemac_registers;
	struct device_node *dt_phy_node;
	struct clk *dt_clk;
	const char *dt_mac;
	int dt_irq_rx;
	int dt_irq_tx;
	int dt_phy_id;
	phy_interface_t	dt_phy_interface;

	/* Hardware resources */
	void __iomem *gemac_regs;

	/* Software device resources */
	struct napi_struct napi_rx;
	struct napi_struct napi_tx;
};

#endif /* _BB_ETH_H */
