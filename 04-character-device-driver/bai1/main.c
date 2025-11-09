#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR("NQH");
MODULE_DESCRIPTION("A Simple Character device");
MODULE_VERSION("0.01");

#define DEVICE		5
#define AUTHOR		"NQH nqhuy@gmail.com"
#define DESC		"A example character device driver"
#define DEVICE_NAME     "char_dd"

static dev_t dev_num;
struct class *my_class;
struct device *my_dev;
struct cdev my_cdev;
char data[100];
static int my_open(struct inode *inode, struct file *file)
{
    pr_info("open is called\n");
	return 0;
}

static int my_close(struct inode *inode, struct file *file)
{
    pr_info("close is called\n");
	return 0;
}

static ssize_t my_read(struct file *flip, char __user *user_buf, size_t len, loff_t *offs)
{
    dump_stack();
    printk("read\n");

	return len;
}

static ssize_t my_write(struct file *flip, const char __user *user_buf, size_t len, loff_t *offs)
{
	memset(data, 0, 100);
	copy_from_user(data, user_buf, len);
        printk("write: %s", data);

	return len;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = my_open,
	.release = my_close,
	.read = my_read,
	.write = my_write,
};

static int __init func_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&dev_num, 0, DEVICE, DEVICE_NAME);
	if (ret) {
		pr_info("Can not register major number\n");
		return ret;
	}
	pr_info("Register successfully major no is %d\n",MAJOR(dev_num));

	cdev_init(&my_cdev, &fops);
	my_cdev.owner = THIS_MODULE;
	my_cdev.dev = dev_num;
	ret = cdev_add(&my_cdev, dev_num, 1);

	if (ret < 0) {
		pr_info("cdev_add error\n");
		return ret;
	}

	my_class = class_create(DEVICE_NAME);
	if (IS_ERR(my_class)) {
		pr_info("create class failed\n");
	}
	pr_info("create successfully class\n");

	device_create(my_class, NULL, dev_num, NULL, DEVICE_NAME);

	return 0;
}

static void __exit func_exit(void)
{
	cdev_del(&my_cdev);
	device_destroy(my_class, dev_num);
	class_destroy(my_class);
	unregister_chrdev(dev_num, DEVICE_NAME);
}

module_init(func_init);
module_exit(func_exit);
