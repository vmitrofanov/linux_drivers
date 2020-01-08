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

#define DEBUG
#ifdef DEBUG
#define DBG(...)	pr_err(__VA_ARGS__)
#else
#define DBG(...)	while(0)
#endif

static int bb_gemac_open(struct net_device *gemac)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

static int bb_gemac_stop(struct net_device *gemac)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

static netdev_tx_t bb_gemac_start_xmit(struct sk_buff *sk, struct net_device *gemac)
{
	DBG("-->%s\n", __FUNCTION__);
	DBG("<--%s\n", __FUNCTION__);

	return NETDEV_TX_OK;
}


static int bb_gemac_get_resources(struct gemac_private *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	struct device_node *child_node;

	DBG("-->%s\n", __FUNCTION__);

	if (!pdev)
		return -1;

	/* CPSW_SS. Ethernet Switch Subsystem */
	pdata->dt_ss = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pdata->dt_ss)
		return -2;

	/* CPSW_WR. Ethernet Subsystem for RGMII/GMII interface */
	pdata->dt_ws = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pdata->dt_ws)
		return -3;

	/* MAC address. It is possible if there is no default MAC in DT */
	pdata->dt_mac = of_get_mac_address(pdev->dev.of_node);

	/* Interrupt */
	pdata->dt_irq = platform_get_irq(pdev, 0);
	if (pdata->dt_irq < 0)
			return -5;

	/* GEMAC clock */
	pdata->dt_clk = devm_clk_get(&pdev->dev, BB_GEMAC_CLK);
	if (IS_ERR(pdata->dt_clk))
		return -6;

	/* MDIO */
	for_each_available_child_of_node(pdev->dev.of_node, child_node) {
		if (!strcmp(child_node->name, "mdio")) {
			DBG("found MDIO\n");
//			of_property_read_u32(child_node, "", &pdata->dt_mdio);
		}
	}
//	if (pdata->dt_mdio )

	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

static int bb_gemac_apply_resources(struct gemac_private *pdata)
{
	DBG("-->%s\n", __FUNCTION__);

	pdata->ws = devm_ioremap_resource(&pdata->pdev->dev, pdata->dt_ws);
	if (IS_ERR(pdata->ws))
		return PTR_ERR(pdata->ws);

	pdata->ss = devm_ioremap_resource(&pdata->pdev->dev, pdata->dt_ss);
	if (IS_ERR(pdata->ss))
		return PTR_ERR(pdata->ss);

//	pdata->mdio = devm_ioremap_resource(&pdata->pdev->dev, pdata->dt_mdio);
//	if (IS_ERR(pdata->mdio))
//		return PTR_ERR(pdata->mdio);

//	if (!pdata->dt_mac)


	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

static const struct net_device_ops gemac_net_ops = {
		.ndo_open = bb_gemac_open,
		.ndo_stop = bb_gemac_stop,
		.ndo_start_xmit = bb_gemac_start_xmit,
};

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
MODULE_DESCRIPTION("Beagle Bone Black GMAC driver");
MODULE_AUTHOR("vmitrofanov <mitrofanovmailbox@gmail.com>");
