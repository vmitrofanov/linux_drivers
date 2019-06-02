#include <linux/module.h>
#include <linux/param.h>
#include <linux/init.h>

MODULE_AUTHOR("vmitrofanov");
MODULE_LICENSE("GPL");

int __init eeprom_init(void) 
{
	pr_info("init\n");
	return 0;
}

void __exit eeprom_exit(void)
{
	pr_info("exit\n");
}

module_init(eeprom_init);
module_exit(eeprom_exit);
