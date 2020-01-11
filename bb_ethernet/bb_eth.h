#ifndef _BB_ETH_H
#define _BB_ETH_H

#define DEBUG
#ifdef DEBUG
#define DBG(...)	pr_err(__VA_ARGS__)
#else
#define DBG(...)	while(0)
#endif

#define DT_MAC_SIZE		6
#define BB_GEMAC_CLK	"fck"

struct gemac_private
{
	/* Common driver resources */
	struct net_device *ndev;
	struct platform_device *pdev;

	/* DT resources */
	struct resource *dt_gemac_registers;
	struct device_node *dt_phy_node;
	struct clk *dt_clk;
	const char *dt_mac;
	int dt_irq_rx;
	int dt_irq_tx;
	int dt_phy_id;

	/* Hardware resources */
	void __iomem *gemac_regs;

	/* Software device resources */
	struct napi_struct napi_rx;
	struct napi_struct napi_tx;
};

#endif /* _BB_ETH_H */
