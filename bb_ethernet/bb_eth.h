#ifndef _BB_ETH_H
#define _BB_ETH_H

#define DT_MAC_SIZE		32
#define BB_GEMAC_CLK	"fck"

struct gemac_private
{
	/* Common driver resources */
	struct net_device *ndev;
	struct platform_device *pdev;

#if 1
	/* DT resources */
	struct resource *dt_ss;
	struct resource *dt_ws;
	struct resource *dt_mdio;
	const char *dt_mac;
	int dt_irq;
	struct clk *dt_clk;
#endif

	/* Hardware resources */
	void __iomem *ss;		/* Switch Subsystem */
	void __iomem *ws;		/* RGMII/GMII Subsystem */
	void __iomem *mdio;		/* MDIO controller */

	/* Software device resources */
};

#endif /* _BB_ETH_H */
