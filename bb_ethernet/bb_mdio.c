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

	DBG("<--%s\n", __FUNCTION__);

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

	DBG("<--%s\n", __FUNCTION__);
}

static void bb_mdio_enable(struct bb_mdio *data)
{
	DBG("-->%s\n", __FUNCTION__);

	/* set enable and clock divider */
	__raw_writel(data->clk_div | CONTROL_ENABLE, &data->regs->control);

	DBG("<--%s\n", __FUNCTION__);
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
		pr_info("MDIO: Resetting idled controller\n");
		bb_mdio_enable(data);
		return -EAGAIN;
	}

	reg = __raw_readl(&regs->user[0].access);
	if ((reg & USERACCESS_GO) == 0)
		return 0;

	pr_err("MDIO: timed out waiting for user access\n");
	return -ETIMEDOUT;
}

static int bb_mdio_reset(struct mii_bus *bus)
{
	struct bb_mdio *data = bus->priv;
	u32 phy_mask, ver;

	DBG("-->%s\n", __FUNCTION__);

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

	DBG("<--%s\n", __FUNCTION__);

done:
	return 0;
}

static int bb_mdio_write(struct mii_bus *bus, int phy_id,
			      int phy_reg, u16 phy_data)
{
	struct bb_mdio *data = bus->priv;
	u32 reg;
	int ret;

	if ((phy_reg & ~PHY_REG_MASK) || phy_id & ~PHY_ID_MASK)
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

	if ((phy_reg & ~PHY_REG_MASK) || (phy_id & ~PHY_ID_MASK))
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

	return ret;
}

void bb_mdio_destroy(struct bb_mdio *mdio)
{
	DBG("-->%s\n", __FUNCTION__);

	mdiobus_unregister(mdio->bus);
	mdiobus_free(mdio->bus);
	pr_info("MDIO: mdio bus is destroyed\n");

	DBG("<--%s\n", __FUNCTION__);
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

	snprintf(mdio->bus->id, MII_BUS_ID_SIZE, "bb_mdiobus:0");

	mdio->bus->name		= "bb_mdio";
	mdio->bus->read		= bb_mdio_read;
	mdio->bus->write	= bb_mdio_write;
	mdio->bus->reset	= bb_mdio_reset;
	mdio->bus->parent	= mdio->parent;
	mdio->bus->priv		= mdio;

	bb_mdio_init_clk(mdio);

	result = mdiobus_register(mdio->bus);
	if (result)
		goto err_bus_register;

	for (addr = 0; addr < PHY_MAX_ADDR; addr++) {
		phy = mdiobus_get_phy(mdio->bus, addr);
		if (phy) {
			pr_info("phy[%d]: device %s, driver %s\n",
				 phy->mdio.addr, phydev_name(phy),
				 phy->drv ? phy->drv->name : "unknown");
		}
	}

	DBG("<--%s\n", __FUNCTION__);

	return 0;

err_bus_register:
	mdiobus_free(mdio->bus);

err_bus_alloc:
	return result;
}
