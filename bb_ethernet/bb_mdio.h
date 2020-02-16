#ifndef _BB_MDIO_H
#define _BB_MDIO_H

#include <linux/if_ether.h>
#include <linux/nvmem-consumer.h>
#include <linux/phy.h>

struct davinci_mdio_regs {
	u32	version;
	u32	control;
#define CONTROL_IDLE		BIT(31)
#define CONTROL_ENABLE		BIT(30)
#define CONTROL_MAX_DIV		(0xffff)

	u32	alive;
	u32	link;
	u32	linkintraw;
	u32	linkintmasked;
	u32	__reserved_0[2];
	u32	userintraw;
	u32	userintmasked;
	u32	userintmaskset;
	u32	userintmaskclr;
	u32	__reserved_1[20];

	struct {
		u32	access;
#define USERACCESS_GO		BIT(31)
#define USERACCESS_WRITE	BIT(30)
#define USERACCESS_ACK		BIT(29)
#define USERACCESS_READ		(0)
#define USERACCESS_DATA		(0xffff)

		u32	physel;
	}	user[0];
};

struct bb_mdio
{
	struct device_node *node;
	struct davinci_mdio_regs __iomem *regs;
	struct mii_bus *bus;
	struct device *parent;
	struct clk *clk;
	u32 bus_freq;
	unsigned long access_time; /* jiffies */
	u32	clk_div;
	bool skip_scan;
};

int bb_mdio_create(struct bb_mdio *mdio);

#if 0
struct davinci_mdio_regs {
	u32	version;
	u32	control;
#define CONTROL_IDLE		BIT(31)
#define CONTROL_ENABLE		BIT(30)
#define CONTROL_MAX_DIV		(0xffff)

	u32	alive;
	u32	link;
	u32	linkintraw;
	u32	linkintmasked;
	u32	__reserved_0[2];
	u32	userintraw;
	u32	userintmasked;
	u32	userintmaskset;
	u32	userintmaskclr;
	u32	__reserved_1[20];

	struct {
		u32	access;
#define USERACCESS_GO		BIT(31)
#define USERACCESS_WRITE	BIT(30)
#define USERACCESS_ACK		BIT(29)
#define USERACCESS_READ		(0)
#define USERACCESS_DATA		(0xffff)

		u32	physel;
	}	user[0];
};

struct davinci_mdio_data {
	struct mdio_platform_data pdata;
	struct davinci_mdio_regs __iomem *regs;
	struct clk	*clk;
	struct device	*dev;
	struct mii_bus	*bus;
	bool            active_in_suspend;
	unsigned long	access_time; /* jiffies */
	/* Indicates that driver shouldn't modify phy_mask in case
	 * if MDIO bus is registered from DT.
	 */
	bool		skip_scan;
	u32		clk_div;
};

struct davinci_mdio_data *davinci_mdio_create(
			struct device *dev,
			struct device_node *node,
			void __iomem *reg_base,
			const char *clk_name);

#endif
#endif
