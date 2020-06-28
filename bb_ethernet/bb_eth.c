#include <linux/module.h>
#include <linux/moduleparam.h>
#include "bb_eth.h"
#include "bb_drv.h"
#include "bb_mdio.h"
#include "bb_ale.h"

static int ring_size = BB_DMA_RING_DEFAULT_SIZE;
module_param(ring_size, int, 0660);

static int buf_size = BB_DMA_BUF_DEFAULT_SIZE;
module_param(buf_size, int, 0660);

/**
 * NOW IT IS NOT NECESSARY. I fixed DT and not it always powered on!
 * DELETE WHEN FINISH DIRIVER. OBSOLETE!
 */
//#define BB_POWER_ON_SUPPORT
#ifdef BB_POWER_ON_SUPPORT
static int bb_power_up(struct device *dev)
{
	int result;

	pm_runtime_enable(dev);
	result = pm_runtime_get_sync(dev);
	if (result < 0) {
		pm_runtime_put_noidle(dev);
	}

	return result;
}
#endif

static void gemac_link_handler(struct net_device *ndev)
{
	struct gemac_private *gemac = netdev_priv(ndev);
	struct phy_device *phy = gemac->phy;
	u32 mac_control = 0;

        DBG("-->%s\n", __FUNCTION__);

        if ((phy->speed == 1000) && phy->duplex) {
        	mac_control |= BIT(7);
        	DBG("gemac: speed 1000 MBit/s\n");
        } else if (phy->speed == 100) {
        	mac_control |= BIT(15);
        	DBG("gemac: speed 100 MBit/s\n");
        } else
		DBG("gemac: speed 10 MBit/s\n");

        if (phy->duplex) {
        	mac_control |= BIT(0);
        	DBG("gemac: full duplex\n");
        }

        if (phy->link) {
        	DBG("gemac: link up\n");
        	/* Release Rx & Tx */
		mac_control |= BIT(5);
        	netif_carrier_on(ndev);
        	netif_tx_wake_all_queues(ndev);
        } else {
        	DBG("gemac: link down\n");
        	netif_carrier_off(ndev);
        	netif_tx_stop_all_queues(ndev);
        }

	__raw_writel(mac_control, &gemac->sliver_port1.regs->mac_control);

        DBG("<--%s\n", __FUNCTION__);
}

/**
 * bb_gemac_open() - UP interface
 * @gemac: associated net-device
 */
static int bb_gemac_open(struct net_device *ndev)
{
	struct gemac_private *gemac = netdev_priv(ndev);
	int ret;


	DBG("-->%s\n", __FUNCTION__);

	ret = devm_request_irq(&gemac->pdev->dev, gemac->dt_irq_rx,
			       rx_interrupt, 0, dev_name(&gemac->pdev->dev),
			       gemac);
	if (ret < 0) {
		dev_err(&gemac->pdev->dev, "can't attach rx irq (%d)\n", ret);
		goto err_request_rx_int;
	}

	ret = devm_request_irq(&gemac->pdev->dev, gemac->dt_irq_tx,
			       tx_interrupt, 0, dev_name(&gemac->pdev->dev),
			       gemac);
	if (ret < 0) {
		dev_err(&gemac->pdev->dev, "can't attach tx irq (%d)\n", ret);
		goto err_request_tx_int;
	}

	napi_enable(&gemac->rx_ring.napi);
	napi_enable(&gemac->tx_ring.napi);

	netif_start_queue(ndev);

	bb_alloc_ring(gemac, &gemac->rx_ring, 1);
	bb_alloc_ring(gemac, &gemac->tx_ring, 0);
	bb_init_rings(gemac);
	bb_start_dma_engine(gemac);

	bb_enable_interrupts(gemac, CPDMA_INT_CHANNEL0);

	DBG("<--%s\n", __FUNCTION__);

	return 0;

err_request_rx_int:
err_request_tx_int:
	return -1;
}

/**
 * bb_gemac_stop() - DOWN interface
 * @gemac: associated net-device
 */
static int bb_gemac_stop(struct net_device *ndev)
{
	struct gemac_private *gemac = netdev_priv(ndev);
	struct ring *rx_ring = &gemac->rx_ring;
	struct ring *tx_ring = &gemac->tx_ring;
	int i;

	DBG("-->%s\n", __FUNCTION__);

	bb_disable_interrupts(gemac, CPDMA_INT_CHANNEL0);

	napi_disable(&gemac->rx_ring.napi);
	napi_disable(&gemac->tx_ring.napi);

	bb_stop_dma_engine(gemac);
	netif_carrier_off(ndev);

	for (i = 0; i < gemac->ring_size; ++i) {
		/* Release Rx ring resources */
		dma_unmap_single(&gemac->pdev->dev,
				 rx_ring->desc_ring[i].buf_dma,
				 rx_ring->desc_ring[i].buf_size,
				 DMA_TO_DEVICE);

		kfree(rx_ring->desc_ring[i].buf_cpu);

		dma_free_coherent(&gemac->pdev->dev, sizeof(struct hw_desc),
				  rx_ring->desc_ring[i].desc_cpu,
				  rx_ring->desc_ring[i].desc_dma);

		/* Release Tx ring resources */
		dma_free_coherent(&gemac->pdev->dev, sizeof(struct hw_desc),
				  tx_ring->desc_ring[i].desc_cpu,
				  tx_ring->desc_ring[i].desc_dma);

	}

	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

/**
 * bb_mac_init() - initialize MAC (SS and WR) with applied resources
 * @gemac: store HW callbacks and parameters to setup
 */
static void bb_mac_init(struct gemac_private *gemac)
{
	DBG("-->%s\n", __FUNCTION__);

	/* Reset MAC */
	__raw_writel(1, &gemac->eth_switch.regs->soft_reset);

	/* Set up priority mapping */
	__raw_writel(0, &gemac->port1.regs->rx_dscp_pri_map[0]);

	/* Disable priority elevation and enable statistics on all ports */
	__raw_writel(0, &gemac->eth_switch.regs->ptype);

        /* Enable statistics collection only on the all ports */
        __raw_writel(7, &gemac->eth_switch.regs->stat_port_en);

	/* Setup MAC */
	__raw_writel(BB_GET_MAC_HI(gemac->dt_mac), &gemac->port1.regs->mac_hi);
	__raw_writel(BB_GET_MAC_LO(gemac->dt_mac), &gemac->port1.regs->mac_lo);

	/* Setup channel priorities. Set channel 0 as highest */
	__raw_writel(0x76543210, &gemac->sliver_port1.regs->rx_pri_map);
	__raw_writel(0x76543210, &gemac->sliver_port2.regs->rx_pri_map);
	__raw_writel(0x33221100, &gemac->port1.regs->tx_pri_map);
	__raw_writel(0x33221100, &gemac->port2.regs->tx_pri_map);
	__raw_writel(1 << 15 | 1 << 5 | 1,
		     &gemac->sliver_port1.regs->mac_control);

	__raw_writel(0, &gemac->eth_switch.regs->ptype);

	/* Max len */
	__raw_writel(BB_DMA_BUF_DEFAULT_SIZE,
		     &gemac->sliver_port1.regs->rx_maxlen);

	DBG("<--%s\n", __FUNCTION__);
}

/**
 * bb_gemac_get_resources() - acquire resources from DT
 * @pdata: storage for resources
 *
 * RETURN: 0 - ok negative - failure
 */
static int bb_gemac_get_resources(struct gemac_private *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	const void *dt;

	DBG("-->%s\n", __FUNCTION__);

	if (!pdev)
		return -1;

	/* Get all GEMAC register range */
	pdata->dt_gemac_regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pdata->dt_gemac_regs)
		return -2;

	/* MAC address. It is possible if there is no default MAC in DT */
	dt = of_get_mac_address(pdev->dev.of_node);
	if (dt)
		memcpy(pdata->dt_mac, dt, ETH_ALEN);
	else
		memset(pdata->dt_mac, 0, ETH_ALEN);

	/* Interrupts. 0 - rx_thresh, 1 - rx, 2 - tx, 3 - misc */
	pdata->dt_irq_rx = platform_get_irq(pdev, 1);
	if (pdata->dt_irq_rx < 0)
		return -3;

	pdata->dt_irq_tx = platform_get_irq(pdev, 2);
	if (pdata->dt_irq_tx < 0)
		return -3;

	/* GEMAC clock */
//	pdata->dt_clk = devm_clk_get(&pdev->dev, BB_GEMAC_CLK);
//	if (IS_ERR(pdata->dt_clk))
//		return -4;

	/* MDIO */
	pdata->mdio.parent = &pdev->dev;
	pdata->mdio.clk = devm_clk_get(&pdev->dev, BB_GEMAC_CLK);
	if (IS_ERR(pdata->mdio.clk))
		return -4;

	//TODO: get child by name because of many child are possible
	pdata->mdio.node = of_get_next_available_child(pdev->dev.of_node, NULL);
	if (of_property_read_u32(pdata->mdio.node, "bus_freq", &pdata->mdio.bus_freq) < 0) {
		pr_err("Missing bus_freq property in the DT.\n");
		return -5;
	}

	//TODO: un comment and replace
//	pdata->dt_phy_interface = of_get_phy_mode(pdata->mdio.node);
	pdata->dt_phy_interface = PHY_INTERFACE_MODE_RMII;
	if (pdata->dt_phy_interface < 0) {
		pr_err("Invalid PHY interface\n");
		return -6;
	}
	/* PHY */
//	pdata->dt_phy_node = of_parse_phandle(pdev->dev.of_node, "phy-handle", 0);
//	if (!pdata->dt_phy_node)
//		return -5;

	DBG("<--%s\n", __FUNCTION__);

	return 0;
}

/**
 * bb_gemac_apply_resources() - apply resources gotten from DT
 * @pdata: private data containing DT resources
 *
 * RETURN: 0 - ok negative - failure
 */
static int bb_gemac_apply_resources(struct gemac_private *pdata)
{
	int result;

	DBG("-->%s\n", __FUNCTION__);

	/* Map GEMAC registers */
	pdata->gemac_regs = devm_ioremap_resource(&pdata->pdev->dev,
						  pdata->dt_gemac_regs);
	if (IS_ERR(pdata->gemac_regs))
		return PTR_ERR(pdata->gemac_regs);

	pdata->eth_switch.regs = pdata->gemac_regs + BB_SWITCH_OFFSET;
	/* Read CPSW version to apply all remain resources */
	pdata->eth_switch.version = readl(&pdata->eth_switch.regs->id_ver);
	if (pdata->eth_switch.version != CPSW_VERSION_2)
		goto err_switch_version;

	/* Assign memory mapped resources to its handler structures */
	pdata->sliver_port1.regs = pdata->gemac_regs + BB_SLIVER_PORT1;
	pdata->sliver_port2.regs = pdata->gemac_regs + BB_SLIVER_PORT2;
	pdata->port0.regs = pdata->gemac_regs + BB_PORT0_OFFSET;
	pdata->port1.regs = pdata->gemac_regs + BB_PORT1_OFFSET;
	pdata->port2.regs = pdata->gemac_regs + BB_PORT2_OFFSET;
	pdata->eth_ts.regs = pdata->gemac_regs + BB_TIME_SYNC_OFFSET;
	pdata->ale.regs = pdata->gemac_regs + BB_ADDR_LOOKUP_OFFSET;
	pdata->mdio.regs = pdata->gemac_regs + BB_MDIO_OFFSET;
	pdata->eth_wr.regs = pdata->gemac_regs + BB_WR_OFFSET;
	pdata->dma.regs = pdata->gemac_regs + BB_DMA_OFFSET;
	pdata->dma.stream = pdata->gemac_regs + BB_DMA_STREAM_OFFSET;
	pdata->stats.regs = pdata->gemac_regs + BB_ETH_STATS_OFFSET;

	/* Enable clock */
	result = clk_prepare_enable(pdata->dt_clk);
	if (result)
		return -1;

	/* Setup MAC */
	if ((pdata->dt_mac[0] != 0) && (pdata->dt_mac[1] != 0) &&
	    (pdata->dt_mac[2] != 0) && (pdata->dt_mac[3] != 0) &&
	    (pdata->dt_mac[4] != 0) && (pdata->dt_mac[5] != 0) &&
	    is_valid_ether_addr(pdata->dt_mac)) {
		ether_addr_copy(pdata->ndev->dev_addr, pdata->dt_mac);
	} else {
		netdev_info(pdata->ndev, "Use random MAC\n");
		eth_hw_addr_random(pdata->ndev);
		memcpy(pdata->dt_mac, pdata->ndev->dev_addr, ETH_ALEN);
	}
	DBG("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", pdata->dt_mac[0],
						    pdata->dt_mac[1],
						    pdata->dt_mac[2],
						    pdata->dt_mac[3],
						    pdata->dt_mac[4],
						    pdata->dt_mac[5]);

	DBG("<--%s\n", __FUNCTION__);

	return 0;

err_switch_version:
	pr_err("unknown switch version\n");

	return -EINVAL;
}

static const struct net_device_ops gemac_net_ops = {
		.ndo_open = bb_gemac_open,
		.ndo_stop = bb_gemac_stop,
		.ndo_start_xmit = bb_gemac_start_xmit,
};

static void bb_setup_driver_settings(struct gemac_private *gemac)
{
	/* Read from global variables which are set as default or as params */
	gemac->ring_size = ring_size;
	gemac->buf_size = buf_size;
}

/**
 * bb_gemac_probe() - Probe the device. Acquire resource, map.
 * @bb_gemac_dev: platform device
 *
 * RETURN: 0 - ok negative - failure
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
	SET_NETDEV_DEV(gemac, &bb_gemac_dev->dev);

	/* Setup settings */
	bb_setup_driver_settings(pdata);

	/* Setup required net device fields */
	gemac->netdev_ops = &gemac_net_ops;

	result = bb_gemac_get_resources(pdata);
	if (result)
		goto err_get_resources;

	result = bb_gemac_apply_resources(pdata);
	if (result)
		goto err_apply_resources;

#ifdef BB_POWER_ON_SUPPORT
	bb_power_up(&bb_gemac_dev->dev);
#endif

	/* Select default pin state */
	result = pinctrl_pm_select_default_state(&bb_gemac_dev->dev);
	if (result)
		goto err_apply_resources;

#ifdef DEBUG
    	int reg = 0;
    	reg = readl(pdata->gemac_regs + 0x1000);
    	DBG("BB_MDIO version: %08x\n", reg);

    	reg = readl(pdata->gemac_regs + 0x800);
    	DBG("BB_CPDMA version: %08x\n", reg);

    	reg = readl(pdata->gemac_regs);
    	DBG("BB_SS version: %08x\n", reg);
#endif

	/* Create MDIO bus, scan and initialize PHYs  */
	result = bb_mdio_create(&pdata->mdio);
	if (result)
		goto err_mdio;

	pdata->phy = phy_connect(gemac, BB_PHY_NAME, gemac_link_handler,
			    	 pdata->dt_phy_interface);

	/* Initialize subsystems */
	bb_mac_init(pdata);
	bb_ale_init(pdata);

	/* Setup NAPI */
	netif_napi_add(pdata->ndev, &pdata->rx_ring.napi, poll_rx, NAPI_POLL_WEIGHT);
	netif_tx_napi_add(pdata->ndev, &pdata->tx_ring.napi, poll_tx, NAPI_POLL_WEIGHT);

	/* Initialize locks */
	//TODO:

	/* Initialize service works */
	//TODO:

	/* Setup ethtool callback */
	//TODO:

	/* Complete registration */
	result = register_netdev(gemac);
	if (result)
		goto err_complete_reg;

	netif_carrier_off(gemac);

	phy_attached_info(pdata->phy);
	phy_start(pdata->phy);

	DBG("<--%s\n", __FUNCTION__);
	return 0;

err_complete_reg:
	netif_napi_del(&pdata->rx_ring.napi);
	netif_napi_del(&pdata->tx_ring.napi);
	bb_mdio_destroy(&pdata->mdio);
err_mdio:
	clk_disable_unprepare(pdata->dt_clk);
err_apply_resources:
err_get_resources:
	/* Nothing to do */

	DBG("RESULT: %d\n", result);

	return result;
}

/**
 * bb_gemac_remove() - Remove bb ethernet device
 * @bb_gemac_dev: platform device
 *
 * RETURN: 0 - OK
 */
static int bb_gemac_remove(struct platform_device *bb_gemac_dev)
{
	struct net_device *gemac;
	struct gemac_private *pdata;

	DBG("-->%s\n", __FUNCTION__);

	pdata = platform_get_drvdata(bb_gemac_dev);
	gemac = pdata->ndev;

	bb_mdio_destroy(&pdata->mdio);

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
