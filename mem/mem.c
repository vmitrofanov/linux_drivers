#include <linux/module.h>
#include <linux/init.h>
#include <asm/io.h>

#define DBG(...) pr_info(__VA_ARGS__)

MODULE_LICENSE("GPL");

static char *addr = "0x0";
module_param(addr, charp, S_IRUGO);

static int __init init_mod(void)
{
	void *pa = NULL;
	long phy_addr = 0;
	long phy_value = 0;

	DBG("%s\n", __FUNCTION__);
	sscanf(addr, "%lx", &phy_addr);

	pr_info("phy_addr: 0x%lx\n", phy_addr);
	
	pa = ioremap(phy_addr, sizeof(long));
	if (pa == NULL)
		return -1;
	
	return 0;
	
	phy_value = readl(pa);
	pr_info("phy_value: 0x%lx\n", phy_value);
	iounmap(pa);
	
	return 0;
}

static void __exit exit_mod(void)
{
	DBG("%s\n", __FUNCTION__);
}

module_init(init_mod);
module_exit(exit_mod);
