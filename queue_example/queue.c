#include <linux/module.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

MODULE_AUTHOR("vmitrfanov");
MODULE_LICENSE("GPL");

#define DEV_NAME "queue_dev"
#define CLS_NAME "class_"DEV_NAME

static int debug = 0;
module_param(debug, int, S_IRUGO);
#define LOG(...) if(debug) pr_info(__VA_ARGS__)

#define DEFAULT_NUM_DEVICES	1
static int num_devices = DEFAULT_NUM_DEVICES;
module_param_named(num, num_devices, int, S_IRUGO); 

dev_t dev;
struct cdev chr_dev;
struct class *cls_dev;
static struct prv_data dev_data;

struct prv_data 
{
	struct list_head lst;
	int major;
	size_t sum_memory_used;
};

struct pub_data
{
	struct list_head lst;
	char *pdata;
	size_t data_len;
};

static ssize_t dummy_store( struct class *class, struct class_attribute *attr,
                        const char *buf, size_t count ) 
{
	return 0;
}

static ssize_t mem_show(struct class *class, 
			struct class_attribute *attr, 
			char *buf ) 
{
	char inner_buf[32] = {0};
	sprintf(inner_buf, "%d\n", dev_data.sum_memory_used);
	strncpy(buf, inner_buf, 32);

	return strnlen(inner_buf, 32);
}

static struct class_attribute class_attr[] =
{ __ATTR(mem_info, 0644, mem_show, dummy_store) };

int open_dev(struct inode *inp, struct file *fp)
{
	LOG("%s\n", __FUNCTION__);
	return 0;
}

int release_dev(struct inode *inp, struct file *fp)
{
	LOG("%s\n", __FUNCTION__);
	return 0;
}

ssize_t read_dev(struct file *fp, char __user *buf, size_t len, loff_t *off)
{
	struct pub_data *dptr = NULL;

	LOG("%s\n", __FUNCTION__);
	
	if (list_empty(&dev_data.lst))
		return 0;

	dptr = list_first_entry(&dev_data.lst, struct pub_data, lst);

	if (copy_to_user(buf, dptr->pdata, dptr->data_len) != 0)
		goto err;

	*off += 1;
	LOG("data_len: %u\n", dptr->data_len);
	vfree(dptr->pdata);
	list_del(&dptr->lst);
	dev_data.sum_memory_used -= dptr->data_len;

	return dptr->data_len;

err:
	return -EIO;
}

ssize_t write_dev(struct file *fp, char const __user *buf, size_t len, loff_t *off)
{	
	char *ptr = NULL;
	struct pub_data *dptr = NULL;

	LOG("%s, len: %u, sizeof: %u\n", __FUNCTION__, len, sizeof(struct pub_data));
	
	dptr = vmalloc(sizeof(struct pub_data));
	if (!dptr)
		return -ENOMEM;
	
	ptr = vmalloc(len);
	if (!ptr) {
		vfree(dptr);
		return -ENOMEM; 
	}

	if (copy_from_user(ptr, buf, len))
		goto free_data;
	
	dptr->pdata = ptr;
	dptr->data_len = len;
	list_add(&dptr->lst, &dev_data.lst);	
	dev_data.sum_memory_used += len;

	/* Each offset is an index of list structure */
	*off += 1;

	return len;

free_data:
	vfree(ptr);
	return -EIO;
}

unsigned int poll_dev (struct file *fp, struct poll_table_struct *ptp)
{
	LOG("%s\n", __FUNCTION__);
	return 0;
}

struct file_operations fs = {
	.open = open_dev,
	.release = release_dev,
	.read = read_dev,
	.write = write_dev,
	.poll = poll_dev
};

static int __init init_m(void)
{
	int res = -1;
	int i = 0;

	LOG("%s-->\n", __FUNCTION__);
	INIT_LIST_HEAD(&dev_data.lst);
	res = alloc_chrdev_region(&dev, 0, num_devices, DEV_NAME);
	if (res < 0)
		goto exit;

	dev_data.major = MAJOR(dev);	
	LOG("major: %d, minor: %d, dev numbers: %d\n", MAJOR(dev), MINOR(dev), num_devices); 	

	cdev_init(&chr_dev, &fs);
	chr_dev.owner = THIS_MODULE;
	res = cdev_add(&chr_dev, dev, num_devices);
	if (res < 0)	
		goto unreg_chrdev;
	
	cls_dev = class_create(THIS_MODULE, CLS_NAME);
	if (!cls_dev) 
		goto del_cdev;

	if (class_create_file(cls_dev, class_attr))
		goto del_class; 

	for (i = 0; i < num_devices; ++i) {
		dev = MKDEV(dev_data.major, i);
		device_create(cls_dev,
			NULL,
			dev,
			NULL,
			DEV_NAME"_%d",
			i);
	}
	
	LOG("%s<--\n", __FUNCTION__);
	return 0;

del_class:
	class_destroy(cls_dev);
del_cdev:	
	cdev_del(&chr_dev);
unreg_chrdev:
	unregister_chrdev_region(dev, num_devices);
exit:
	return -1;
}

static void __exit exit_m(void)
{
	int i = 0;
	struct pub_data *pos;

	list_for_each_entry(pos, &dev_data.lst, lst) {
		vfree(pos->pdata);
		vfree(pos);
	}

	class_remove_file(cls_dev, class_attr);	

	LOG("%s-->\n", __FUNCTION__);
	for (i = 0; i < num_devices; ++i) {
		dev = MKDEV(dev_data.major, i);
		device_destroy(cls_dev, dev);
	}
	class_destroy(cls_dev);
	cdev_del(&chr_dev);
	unregister_chrdev_region(dev, num_devices);
	LOG("%s<--\n", __FUNCTION__);
}

module_init(init_m);
module_exit(exit_m);
