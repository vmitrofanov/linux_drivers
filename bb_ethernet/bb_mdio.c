#define pr_fmt(fmt) "MDIO: " fmt
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/pm_runtime.h>
#include <linux/davinci_emac.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/pinctrl/consumer.h>
#include "bb_mdio.h"
#include "bb_eth.h"

#define MDIO_TIMEOUT		100 /* msecs */
#define PHY_REG_MASK		0x1f
#define PHY_ID_MASK			0x1f


static void bb_mdio_init_clk(struct bb_mdio *data)
{
	u32 mdio_in;
	u32 div;
	u32 mdio_out_khz;
	u32 access_time;

	mdio_in = clk_get_rate(data->clk);
	div = (mdio_in / data->bus_freq) - 1;
	if (div > CONTROL_MAX_DIV)
		div = CONTROL_MAX_DIV;

	data->clk_div = div;
	/*
	 * One mdio transaction consists of:
	 *	32 bits of preamble
	 *	32 bits of transferred data
	 *	24 bits of bus yield (not needed unless shared?)
	 */
	mdio_out_khz = mdio_in / (1000 * (div + 1));
	access_time  = (88 * 1000) / mdio_out_khz;

	/*
	 * In the worst case, we could be kicking off a user-access immediately
	 * after the mdio bus scan state-machine triggered its own read.  If
	 * so, our request could get deferred by one access cycle.  We
	 * defensively allow for 4 access cycles.
	 */
	data->access_time = usecs_to_jiffies(access_time * 4);
	if (!data->access_time)
		data->access_time = 1;
}

static void bb_mdio_enable(struct bb_mdio *data)
{
	/* set enable and clock divider */
	__raw_writel(data->clk_div | CONTROL_ENABLE, &data->regs->control);
}

/* wait until hardware is ready for another user access */
static inline int wait_for_user_access(struct bb_mdio *data)
{
	struct davinci_mdio_regs __iomem *regs = data->regs;
	unsigned long timeout = jiffies + msecs_to_jiffies(MDIO_TIMEOUT);
	u32 reg;

	while (time_after(timeout, jiffies)) {
		reg = __raw_readl(&regs->user[0].access);
		if ((reg & USERACCESS_GO) == 0)
			return 0;

		reg = __raw_readl(&regs->control);
		if ((reg & CONTROL_IDLE) == 0) {
			usleep_range(100, 200);
			continue;
		}

		/*
		 * An emac soft_reset may have clobbered the mdio controller's
		 * state machine.  We need to reset and retry the current
		 * operation
		 */
		pr_info("resetting idled controller\n");
		bb_mdio_enable(data);
		return -EAGAIN;
	}

	reg = __raw_readl(&regs->user[0].access);
	if ((reg & USERACCESS_GO) == 0)
		return 0;

	pr_err("timed out waiting for user access\n");
	return -ETIMEDOUT;
}

static int bb_mdio_reset(struct mii_bus *bus)
{
	struct bb_mdio *data = bus->priv;
	u32 phy_mask, ver;

	/* wait for scan logic to settle */
	msleep(PHY_MAX_ADDR * data->access_time);

	/* dump hardware version info */
	ver = __raw_readl(&data->regs->version);
	pr_info("davinci mdio revision %d.%d, bus freq %u\n",
		 (ver >> 8) & 0xff, ver & 0xff,
		 data->bus_freq);

	if (data->skip_scan)
		goto done;

	/* get phy mask from the alive register */
	phy_mask = __raw_readl(&data->regs->alive);
	if (phy_mask) {
		/* restrict mdio bus to live phys only */
		pr_info("detected phy mask %x\n", ~phy_mask);
		phy_mask = ~phy_mask;
	} else {
		/* desperately scan all phys */
		pr_info("no live phy, scanning all\n");
		phy_mask = 0;
	}
	data->bus->phy_mask = phy_mask;

done:
	return 0;
}

static int bb_mdio_write(struct mii_bus *bus, int phy_id,
			      int phy_reg, u16 phy_data)
{
	struct bb_mdio *data = bus->priv;
	u32 reg;
	int ret;

	//DBG("-->%s\n", __FUNCTION__);

	if (phy_reg & ~PHY_REG_MASK || phy_id & ~PHY_ID_MASK)
		return -EINVAL;

	reg = (USERACCESS_GO | USERACCESS_WRITE | (phy_reg << 21) |
		   (phy_id << 16) | (phy_data & USERACCESS_DATA));

	while (1) {
		ret = wait_for_user_access(data);
		if (ret == -EAGAIN)
			continue;
		if (ret < 0)
			break;

		__raw_writel(reg, &data->regs->user[0].access);

		ret = wait_for_user_access(data);
		if (ret == -EAGAIN)
			continue;
		break;
	}

	return ret;
}

static int bb_mdio_read(struct mii_bus *bus, int phy_id, int phy_reg)
{
	struct bb_mdio *data = bus->priv;
	u32 reg;
	int ret;

	//DBG("-->%s\n", __FUNCTION__);

	if (phy_reg & ~PHY_REG_MASK || phy_id & ~PHY_ID_MASK)
		return -EINVAL;

	reg = (USERACCESS_GO | USERACCESS_READ | (phy_reg << 21) |
	       (phy_id << 16));

	while (1) {
		ret = wait_for_user_access(data);
		if (ret == -EAGAIN)
			continue;
		if (ret < 0)
			break;

		__raw_writel(reg, &data->regs->user[0].access);

		ret = wait_for_user_access(data);
		if (ret == -EAGAIN)
			continue;
		if (ret < 0)
			break;

		reg = __raw_readl(&data->regs->user[0].access);
		ret = (reg & USERACCESS_ACK) ? (reg & USERACCESS_DATA) : -EIO;
		break;
	}

	//DBG("-->%s: ret: %08x\n", __FUNCTION__, ret);
	return ret;
}

int bb_mdio_create(struct bb_mdio *mdio)
{
	int result = -1;
	u32 addr;
	struct phy_device *phy;

	DBG("-->%s\n", __FUNCTION__);

	mdio->bus = mdiobus_alloc();
	if (!mdio->bus)
		goto err_bus_alloc;

	DBG("%s: [success] allocate bus\n", __FUNCTION__);

	snprintf(mdio->bus->id, MII_BUS_ID_SIZE, "bb_mdiobus:0");

	mdio->bus->name		= "bb_mdio";
	mdio->bus->read		= bb_mdio_read;
	mdio->bus->write	= bb_mdio_write;
	mdio->bus->reset	= bb_mdio_reset;
	mdio->bus->parent	= mdio->parent;
	mdio->bus->priv		= mdio;

	bb_mdio_init_clk(mdio);

	DBG("%s: mdio node name: %s\n", __FUNCTION__, mdio->node->name);
//	result = of_mdiobus_register(mdio->bus, mdio->node);
//	if (result)
//		goto err_bus_register;

	result = mdiobus_register(mdio->bus);
	if (result)
		goto err_bus_register;

	DBG("%s: [success] register bus\n", __FUNCTION__);

	for (addr = 0; addr < PHY_MAX_ADDR; addr++) {
		phy = mdiobus_get_phy(mdio->bus, addr);
		if (phy) {
			DBG("phy[%d]: device %s, driver %s\n",
				 phy->mdio.addr, phydev_name(phy),
				 phy->drv ? phy->drv->name : "unknown");
		} else {
			DBG("no phy at %u\n", addr);
		}

	}


//	result = devm_add_action_or_reset(dev, (void(*)(void *))mdiobus_unregister,
//				       mdio->bus);
//	if (result) {
//		pr_err("failed to add percpu stat free action %d", result);
//		return -1;
//	}
	DBG("<--%s\n", __FUNCTION__);

	return 0;

#if 0
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	data->dev = dev;
	data->regs = reg_base;
	data->bus = devm_mdiobus_alloc(dev);
	if (!data->bus)
		return ERR_PTR(-ENOMEM);

	if (of_property_read_u32(node, "bus_freq", &prop)) {
		dev_err(dev, "Missing bus_freq property in the DT.\n");
		return ERR_PTR(-EINVAL);
	}
	data->pdata.bus_freq = prop;

	snprintf(data->bus->id, MII_BUS_ID_SIZE, "k3-cpsw-mdio");

	data->bus->name		= dev_name(dev);
	data->bus->read		= davinci_mdio_read,
	data->bus->write	= davinci_mdio_write,
	data->bus->reset	= davinci_mdio_reset,
	data->bus->parent	= dev;
	data->bus->priv		= data;

	data->clk = devm_clk_get(dev, clk_name);
	if (IS_ERR(data->clk)) {
		dev_err(dev, "failed to get device clock\n");
		return ERR_CAST(data->clk);
	}

	davinci_mdio_init_clk(data);

	data->skip_scan = true;
	ret = of_mdiobus_register(data->bus, node);
	if (ret) {
		dev_err(dev, "mdio register err %d.\n", ret);
		goto bail_out;
	}

	ret = devm_add_action_or_reset(dev, (void(*)(void *))mdiobus_unregister,
				       data->bus);
	if (ret) {
		dev_err(dev, "failed to add percpu stat free action %d", ret);
		return ERR_PTR(ret);
	}

	return data;

bail_out:
	return ERR_PTR(ret);
#endif

err_bus_register:
	mdiobus_free(mdio->bus);

err_bus_alloc:
	return result;
}

#if 0
struct davinci_mdio_data *davinci_mdio_create(
			struct device *dev,
			struct device_node *node,
			void __iomem *reg_base,
			const char *clk_name)
{
	struct davinci_mdio_data *data;
	int ret;
	u32 prop;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	data->dev = dev;
	data->regs = reg_base;
	data->bus = devm_mdiobus_alloc(dev);
	if (!data->bus)
		return ERR_PTR(-ENOMEM);

	if (of_property_read_u32(node, "bus_freq", &prop)) {
		dev_err(dev, "Missing bus_freq property in the DT.\n");
		return ERR_PTR(-EINVAL);
	}
	data->pdata.bus_freq = prop;

	snprintf(data->bus->id, MII_BUS_ID_SIZE, "k3-cpsw-mdio");

	data->bus->name		= dev_name(dev);
	data->bus->read		= davinci_mdio_read,
	data->bus->write	= davinci_mdio_write,
	data->bus->reset	= davinci_mdio_reset,
	data->bus->parent	= dev;
	data->bus->priv		= data;

	data->clk = devm_clk_get(dev, clk_name);
	if (IS_ERR(data->clk)) {
		dev_err(dev, "failed to get device clock\n");
		return ERR_CAST(data->clk);
	}

	davinci_mdio_init_clk(data);

	data->skip_scan = true;
	ret = of_mdiobus_register(data->bus, node);
	if (ret) {
		dev_err(dev, "mdio register err %d.\n", ret);
		goto bail_out;
	}

	ret = devm_add_action_or_reset(dev, (void(*)(void *))mdiobus_unregister,
				       data->bus);
	if (ret) {
		dev_err(dev, "failed to add percpu stat free action %d", ret);
		return ERR_PTR(ret);
	}

	return data;

bail_out:
	return ERR_PTR(ret);
}
#endif
