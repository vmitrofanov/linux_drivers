#include <linux/module.h>

#include "bb_eth.h"
#include "bb_drv.h"
#include <linux/of_mdio.h>

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
		return -4;

	/* GEMAC clock */
	pdata->dt_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pdata->dt_clk))
		return -5;

	/* phy-handle */
	pdata->dt_phy_node = of_parse_phandle(pdev->dev.of_node, "phy-handle", 0);
	if (pdata->dt_phy_node)
		return -6;

	/* MDIO mode */
	pdata->dt_phy_interface = PHY_INTERFACE_MODE_RGMII;
	//TODO: check!
//	pdata->dt_phy_interface = of_get_phy_mode(pdev->dev.of_node);
//	if (pdata->dt_phy_interface < 0)
//		return -7;

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
#if 0
	{
		struct clk *clock;
		clock = pdata->dt_clk;

		while (clock) {
			clock = clk_get_parent(clock);
			clk_prepare_enable(clock);
		}

	}
#endif

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

static void gemac_link_handler(struct net_device *ndev)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);
}

/**
 * Probe the device. Acquire resource, map.
 * @param bb_gemac_dev platform device
 */
static int bb_gemac_probe(struct platform_device *bb_gemac_dev)
{
	int result;
	struct net_device *gemac;
	struct device *phy_device;
	struct gemac_private *pdata;
	struct device_node *child = NULL;
	struct mii_bus *bus;
	int (*phy_warm_probe)(void);
	//const char *phy_name = "4a101000.mdio:00";

	DBG("-->%s\n", __FUNCTION__);

	gemac = alloc_etherdev(sizeof(struct gemac_private));
	if (!gemac)
		return -ENOMEM;

	/* Embed private data into platform device */
	pdata = netdev_priv(gemac);
	pdata->ndev = gemac;
	pdata->pdev = bb_gemac_dev;
	platform_set_drvdata(bb_gemac_dev, pdata);
	SET_NETDEV_DEV(gemac, &bb_gemac_dev->dev);

	/* Setup required net device fields */
	gemac->netdev_ops = &gemac_net_ops;

	result = bb_gemac_get_resources(pdata);
	if (result)
			goto disable_sth;

	/* Apply gotten resources to driver */
	result = bb_gemac_apply_resources(pdata);
	if (result)
			goto disable_sth;

    /* This may be required here for child devices */
    pm_runtime_enable(&bb_gemac_dev->dev);

    /* Select default pin state */
    pinctrl_pm_select_default_state(&bb_gemac_dev->dev);

    /* Need to enable clocks with runtime PM api to access regs */
    result = pm_runtime_get_sync(&bb_gemac_dev->dev);
    if (result < 0) {
            pm_runtime_put_noidle(&bb_gemac_dev->dev);
            goto disable_sth;
    }

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
	phy_device = bus_find_device_by_name(&platform_bus_type, NULL, BB_PHY_NAME);
	if (!phy_device)
		goto phy_err;

	/* Make PHY probe again with enabled clock */
	pdata->phy_platform = container_of(phy_device, struct platform_device, dev);

	/* Warm probe side effect - it rewrite driver data with 'struct davinci_mdio_data' */
	phy_warm_probe = dev_get_drvdata(&pdata->phy_platform->dev);
	phy_warm_probe();

	child = of_get_next_child(pdata->phy_platform->dev.of_node, NULL);
	if (child == NULL) {
		pr_err("child == NULL\n");
		return -1;
	} else {
		pr_err("child != NULL %s\n", child->name);

	}
//
////	pdata->phy = of_phy_connect(gemac,
////			child,
////			gemac_link_handler,
////			0,
////			pdata->dt_phy_interface);
//
//	pr_err("try to get owner\n");
//	if(gemac->dev.parent != NULL)
//		pr_err("owner exists\n");
//	else
//		pr_err("owner not exists\n");
//
////	pdata->phy = of_phy_find_device(child);
////	bus = pdata->phy->mdio.bus;
////
////	if (bus)
////		pr_err("bus found\n");
////	else
////		pr_err("bus not found\n");
//
//	/* Exit */
//	return 0;

//	pdata->phy = phy_connect(gemac,
//			"4a101000.mdio:00",
//			gemac_link_handler,
//			pdata->dt_phy_interface);
//
//	if (IS_ERR(pdata->phy)) {
//		pr_err("cant conncet to phy\n");
//		return -1;
//	}

		pdata->phy = of_phy_connect(gemac,
				child,
				gemac_link_handler,
				0,
				pdata->dt_phy_interface);

//	pdata->phy = of_phy_find_device(child);


////	result = phy_attach_direct(gemac, pdata->phy, 0, pdata->dt_phy_interface);
////	pr_err("result: %d\n", result);
//
//	result = phy_init_hw(pdata->phy);
//	pr_err("phy_init_hw: %d\n", result);
//
//	result = pdata->phy->drv->config_init(pdata->phy);
//	pr_err("config init: %d\n", result);

	if (!pdata->phy) {
		pr_err("can not find phy\n");
		goto phy_connect_err;
	} else {
		pr_err("PHY got successfully\n");
	}

	//pr_err("phy_id: %u\n", pdata->phy->phy_id);
	//pr_info("phy->irq: %d\n", pdata->phy->irq);
	phy_attached_info(pdata->phy);
	phy_start(pdata->phy);



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

phy_err:
	pr_err("Can not find Ethernet PHY\n");

phy_connect_err:
	pr_err("Can not connect to Ethernet PHY\n");

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
		{.compatible = "mvv,bb-eth"},
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
