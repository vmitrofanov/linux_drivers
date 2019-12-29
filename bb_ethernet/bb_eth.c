#include <linux/module.h>

static int __init bb_init(void)
{
	return 0;
}

static void __exit bb_exit(void)
{

}

module_init(bb_init);
module_exit(bb_exit);
