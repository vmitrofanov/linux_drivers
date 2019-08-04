#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#define MOD_NAME "25q256a"

MODULE_AUTHOR("vmitrofanov");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Block driver for Micron 25q256a");

static int debug = 1;
module_param(debug, int, S_IRUGO);

#define DBG(...) if(debug) pr_info(__VA_ARGS__)

static struct of_device_id compatible[] = {
	{ .compatible = "vm,25q256a" },
	{}
};

int probe(struct spi_device *spi)
{
	return 0;
}

int remove(struct spi_device *spi)
{
	return 0;
}

struct spi_driver spi_driver = {
	.probe = probe,
	.remove = remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = MOD_NAME,
		.of_match_table = compatible,
	},
};


module_spi_driver(spi_driver);
MODULE_DEVICE_TABLE(of, compatible);
