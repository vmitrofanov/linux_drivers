#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/clk.h>

#include "bb_eth.h"
#include "bb_drv.h"

/**
 * UP interface
 * @param gemac associated net-device
 */
static int bb_gemac_open(struct net_device *gemac)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

/**
 * DOWN interface
 * @param gemac associated net-device
 */
static int bb_gemac_stop(struct net_device *gemac)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

/**
 * Start transferring net-data gotten from the stack
 * @param sk net-data
 * @gemac gemac associated net-device
 */
static netdev_tx_t bb_gemac_start_xmit(struct sk_buff *sk, struct net_device *gemac)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);

	return NETDEV_TX_OK;
}

/**
 * Acquire resources from DT
 * @param pdata storage for resources
 */
static int bb_gemac_get_resources(struct gemac_private *pdata)
{
	struct platform_device *pdev = pdata->pdev;

	DBG("-->%s\n", __FUNCTION__);

	if (!pdev)
		return -1;

	/* Get all GEMAC register range */
	pdata->dt_gemac_registers = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pdata->dt_gemac_registers)
		return -2;

	/* MAC address. It is possible if there is no default MAC in DT */
	pdata->dt_mac = of_get_mac_address(pdev->dev.of_node);

	/* Interrupts. 0-rx_thresh, 1-rx, 2-tx, 3-misc */
	pdata->dt_irq_rx = platform_get_irq(pdev, 1);
	if (pdata->dt_irq_rx < 0)
			return -3;

	pdata->dt_irq_tx = platform_get_irq(pdev, 2);
	if (pdata->dt_irq_tx < 0)
				return -3;

	/* GEMAC clock */
	pdata->dt_clk = devm_clk_get(&pdev->dev, BB_GEMAC_CLK);
	if (IS_ERR(pdata->dt_clk))
		return -4;

	/* phy-handle */
	pdata->dt_phy_node = of_parse_phandle(pdev->dev.of_node, "phy-handle", 0);
	if (pdata->dt_phy_node)
		return -5;

	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

/**
 * Apply resources gotten from DT
 * @param pdata private data containing DT resources
 */
static int bb_gemac_apply_resources(struct gemac_private *pdata)
{
	int result;

	DBG("-->%s\n", __FUNCTION__);

	/* Map GEMAC registers */
	pdata->gemac_regs = devm_ioremap_resource(&pdata->pdev->dev, pdata->dt_gemac_registers);
	if (IS_ERR(pdata->gemac_regs))
		return PTR_ERR(pdata->gemac_regs);

	/* Enable clock */
	result = clk_prepare_enable(pdata->dt_clk);
	if (result)
		return -1;

	/* Setup MAC */
	if (pdata->dt_mac) {
		ether_addr_copy(pdata->ndev->dev_addr, pdata->dt_mac);
	} else {
		netdev_info(pdata->ndev, "Use random MAC\n");
		eth_hw_addr_random(pdata->ndev);
	}

	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

static const struct net_device_ops gemac_net_ops = {
		.ndo_open = bb_gemac_open,
		.ndo_stop = bb_gemac_stop,
		.ndo_start_xmit = bb_gemac_start_xmit,
};

/**
 * Probe the device. Acquire resource, map.
 * @param bb_gemac_dev platform device
 */
static int bb_gemac_probe(struct platform_device *bb_gemac_dev)
{
	int result;
	struct net_device *gemac;
	struct gemac_private *pdata;

	DBG("-->%s\n", __FUNCTION__);

	gemac = alloc_etherdev(sizeof(struct gemac_private));
	if (!gemac)
		return -ENOMEM;

	/* Embed private data into platform device */
	pdata = netdev_priv(gemac);
	pdata->ndev = gemac;
	pdata->pdev = bb_gemac_dev;
	platform_set_drvdata(bb_gemac_dev, pdata);

	/* Setup required net device fields */
	gemac->netdev_ops = &gemac_net_ops;

	result = bb_gemac_get_resources(pdata);
	if (result)
			goto disable_sth;

	/* Apply gotten resources to driver */
	result = bb_gemac_apply_resources(pdata);
	if (result)
			goto disable_sth;

	/* Setup NAPI */
	netif_napi_add(pdata->ndev, &pdata->napi_rx, poll_rx, NAPI_POLL_WEIGHT);
	netif_tx_napi_add(pdata->ndev, &pdata->napi_tx, poll_tx, NAPI_POLL_WEIGHT);

	/*
	 * NOTE: Access to configuration PHY is implemented by build in
	 *       MDIO controller. MDIO driver is in davinci_mdio.c
	 *       Driver create MDIO bus with capability to access controller
	 *       registers.
	 *
	 *       This driver use MDIO bus to scan all PHYs connected to it.
	 *       By default Beagle Bone Black use PHY LAN8710A which places
	 *       in drivers/net/phy/smsc.c
	 */

	/* Scan MDIO bus to get PHYs and attach to the ETH device */
	//TODO:
	//phy = phy_find_first(bus);

	/* Initialize locks */
	//TODO:

	/* Initialize service works */
	//TODO:

	/* Setup ethtool callback */
	//TODO:

	/* Complete registration */
	result = register_netdev(gemac);
	if (result)
		goto disable_sth;

	netif_carrier_off(gemac);

	DBG("<--%s\n", __FUNCTION__);
	return 0;

disable_sth:
	DBG("RESULT: %d\n", result);
	return -1;
}

/**
 * Remove bb ethernet device
 * @param bb_gemac_dev platform device
 */
static int bb_gemac_remove(struct platform_device *bb_gemac_dev)
{
	struct net_device *gemac;
	struct gemac_private *pdata;

	DBG("-->%s\n", __FUNCTION__);

	pdata = platform_get_drvdata(bb_gemac_dev);
	gemac = pdata->ndev;

	unregister_netdev(gemac);
	free_netdev(gemac);

	DBG("<--%s\n", __FUNCTION__);
	return 0;
}

static struct of_device_id dt_compatible_table[] = {
		{.compatible = "mvv,bb-gmac"},
		{/* sentinel */}
};

static struct platform_driver bb_gmac_driver = {
		.probe = bb_gemac_probe,
		.remove = bb_gemac_remove,
		.driver = {
				.name = "bb_gmac_driver",
				.owner = THIS_MODULE,
				.of_match_table = of_match_ptr(dt_compatible_table),
		},
};

module_platform_driver(bb_gmac_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Beagle Bone Black GEMAC driver");
MODULE_AUTHOR("vmitrofanov <mitrofanovmailbox@gmail.com>");
