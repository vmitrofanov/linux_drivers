#include <linux/module.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>

/*
 * NOTE:
 * FM24CL16B has no way to change address, because of
 * absence address pins in the chip. Predefined address
 * is 1010, it means that only one device can present on
 * that i2c bus with this address.
 *
 * FM24 phy addressing:
 * [S] - [1010|A2|A1|A0|R/W] - [B7|B6|B5|B4|B3|B2|B1|B0]
 * A2-A0 - page address
 * B7-B0 - byte address
 * 2^3 * 2^8 = 2KB EEPROM size
 *
 * Driver API addressing:
 * Driver use linear addressing from 0 byte to EEPROM size,
 * there are no page and page offset.
 */

#define EEPROM_PAGE_NUMS		8
#define EEPROM_PAGE_SIZE		256
#define EEPROM_SIZE				(EEPROM_PAGE_NUMS * EEPROM_PAGE_SIZE)
#define EEPROM_NAME				"FM24"

MODULE_AUTHOR("vmitrofanov");
MODULE_LICENSE("GPL");

struct fm24 {
	struct i2c_client *client;
	struct cdev eeprom;
	dev_t dev;
};

DEFINE_MUTEX(dev_mtx);
static struct fm24 fm24_device;
static struct class *cls;
struct of_device_id compatible[] = {
	{ .compatible = "vm,fm24_eeprom"},
	{}
};

static int __read(struct i2c_client *client, u16 addr, u8 *buf, u16 buf_size)
{
	struct i2c_msg msg[2];
	u8 page = addr / EEPROM_PAGE_SIZE;
	u8 page_off = addr % EEPROM_PAGE_SIZE;
	int msg_executed = 0;

	if (!client)
			return -EFAULT;

	if (addr > EEPROM_SIZE)
		return -EFAULT;

	if (buf_size > EEPROM_SIZE)
		buf_size = EEPROM_SIZE;

	if (addr + buf_size > EEPROM_SIZE)
		buf_size = EEPROM_SIZE - addr;

	/* FM24 address byte include page number */
	msg[0].addr = client->addr | page;
	msg[0].buf = &page_off;
	msg[0].flags = 0;
	msg[0].len = sizeof(u8);

	msg[1].addr = client->addr | page;
	msg[1].buf = buf;
	msg[1].flags = I2C_M_RD;
	msg[1].len = buf_size;

	msg_executed = i2c_transfer(client->adapter, msg, sizeof(msg) / sizeof(struct i2c_msg));

	if (msg_executed < 0)
		return -EIO;
	else
		return buf_size;
}


static int __write(struct i2c_client *client, u16 addr, u8 *buf, u16 buf_size)
{
	struct i2c_msg msg[1];
	u8 page = addr / EEPROM_PAGE_SIZE;
	u8 page_off = addr % EEPROM_PAGE_SIZE;
	void *_buf = NULL;
	int msg_executed = 0;

	if (!client)
			return -EFAULT;

	if (addr > EEPROM_SIZE)
		return -EFAULT;

	if (buf_size > EEPROM_SIZE)
		buf_size = EEPROM_SIZE;

	if (addr + buf_size > EEPROM_SIZE)
		buf_size = EEPROM_SIZE - addr;

	_buf = vmalloc(buf_size + 1);
	if (!_buf)
		return -ENOMEM;

	((u8*)_buf)[0] = page_off;
	memcpy((u8*)_buf + 1, buf, buf_size);

	/* FM24 address byte include page number */
	msg[0].addr = client->addr | page;
	msg[0].buf = _buf;
	msg[0].flags = 0;
	msg[0].len = buf_size + 1;

	msg_executed = i2c_transfer(client->adapter, msg, sizeof(msg) / sizeof(struct i2c_msg));

	vfree(_buf);

	if (msg_executed < 0)
		return -EIO;
	else
		return buf_size;
}

int eeprom_open(struct inode *inode, struct file *file)
{
	file->private_data = &fm24_device;
	return 0;
}

int eeprom_release(struct inode *inode, struct file *file)
{
	return 0;
}

ssize_t eeprom_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	void *rbuf = NULL;
	int read_num = 0;
	struct fm24 *dev = filp->private_data;

	if (!buf)
		return -EIO;

	if (*f_pos > EEPROM_SIZE)
			return -EFAULT;

	if (count > EEPROM_SIZE)
		count = EEPROM_SIZE;

	if (count + *f_pos > EEPROM_SIZE)
		count = EEPROM_SIZE - *f_pos;

	rbuf = kmalloc(sizeof(char) * count, GFP_KERNEL);
	if (!rbuf)
		return -EIO;

	read_num = __read(dev->client, *f_pos, rbuf, count);
	if(copy_to_user(buf, rbuf, read_num))
		goto err;
	if (read_num < 0)
		goto err;

	kfree(rbuf);
	*f_pos += read_num;

	return (ssize_t)read_num;

err:
	kfree(rbuf);
	return -EIO;
}

ssize_t  eeprom_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	void *wbuf = NULL;
	int write_num = 0;
	struct fm24 *dev = filp->private_data;

	if (!buf)
		return -EIO;

	if (*f_pos > EEPROM_SIZE)
			return -EFAULT;

	if (count > EEPROM_SIZE)
		count = EEPROM_SIZE;

	if (count + *f_pos > EEPROM_SIZE)
		count = EEPROM_SIZE - *f_pos;

	wbuf = kmalloc(sizeof(char) * count, GFP_KERNEL);
	if (!wbuf)
		return -EIO;

	if (copy_from_user(wbuf, buf, count))
		goto err;

	write_num = __write(dev->client, *f_pos, wbuf, count);
	if (write_num < 0)
		goto err;

	kfree(wbuf);
	*f_pos += write_num;

	return (ssize_t)write_num;

err:
	kfree(wbuf);
	return -EIO;
}

loff_t eeprom_llseek(struct file *filp, loff_t off, int whence)
{
    loff_t newpos = 0;

    switch (whence) {
      case 0: /* SEEK_SET */
        newpos = off;
        break;

      case 1: /* SEEK_CUR */
        newpos = filp->f_pos + off;
        break;

      case 2: /* SEEK_END - Not supported */
        return -EINVAL;

      default: /* can't happen */
        return -EINVAL;
    }
    if (newpos < 0 || newpos > EEPROM_SIZE)
        return -EINVAL;

    filp->f_pos = newpos;

    return newpos;
}

static struct file_operations fops = {
		.open = eeprom_open,
		.release = eeprom_release,
		.read = eeprom_read,
		.write = eeprom_write,
		.llseek = eeprom_llseek
};

int probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	u8 buff[16] = {0};

	mutex_lock(&dev_mtx);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    	return -EIO;

	/* Read to check device */
	if (__read(client, 0, buff, sizeof(buff) / sizeof(u8)) < 0) {
		return -EIO;
	}

	cls = class_create(THIS_MODULE, EEPROM_NAME);
	if (IS_ERR(cls))
		goto out_cls;

	fm24_device.client = client;
	i2c_set_clientdata(client, &fm24_device);

	if(alloc_chrdev_region(&fm24_device.dev, 0, 1, EEPROM_NAME))
		goto out_cls;

	cdev_init(&fm24_device.eeprom, &fops);
	if (cdev_add(&fm24_device.eeprom, fm24_device.dev, 1) < 0)
		goto cdev_deinit;

	if (!device_create(cls, NULL, fm24_device.dev, NULL, EEPROM_NAME))
		goto destroy_device;

	mutex_unlock(&dev_mtx);

	return 0;

destroy_device:
	pr_info("destroy_device\n");
	device_destroy(cls, fm24_device.dev);
	cdev_del(&fm24_device.eeprom);

cdev_deinit:
	pr_info("cdev_deinit\n");
	class_destroy(cls);

out_cls:
	pr_info("out_cls\n");

	mutex_unlock(&dev_mtx);

	return -EIO;
};

int remove(struct i2c_client *client)
{
	pr_info("remove\n");
	device_destroy(cls, fm24_device.dev);
	cdev_del(&fm24_device.eeprom);
	class_destroy(cls);

	return 0;
};

struct i2c_driver drv = {
	.probe = probe,
	.remove = remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "fm24_eeprom_driver",
		.of_match_table = compatible
	}
};

module_i2c_driver(drv);
