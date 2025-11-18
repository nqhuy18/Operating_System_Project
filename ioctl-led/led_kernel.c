#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>

#include "led_ioctl.h"
#include <linux/delay.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("GROUP_15");
MODULE_DESCRIPTION("A Simple Character device");
MODULE_VERSION("0.01");

static struct gpio_desc *led;
#define IO_LED 21
#define IO_OFFSET 0

#define DEVICE		5
#define AUTHOR		"GROUP_15"
#define DESC		"A example character device driver"
#define DEVICE_NAME     "led"

static dev_t dev_num;
struct class *my_class;
struct device *my_dev;
struct cdev my_cdev;


void led_on(void) {
	gpiod_set_value(led, 1);
	pr_info("Led: ON\n");
}

void led_off(void) {
        gpiod_set_value(led, 0);
        pr_info("Led: OFF\n");
}

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
    printk("read is called\n");

	return len;
}
int state = -1;
static long int my_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
        struct led_blink_cfg cfg;
        switch(cmd) {
                case LED_ON:
                            led_on();
                            state = 1;
                            break;
                case LED_OFF:
                            led_off();
                            state = 0;
                            break;
                case LED_GET_STATE:
                            if (copy_to_user((int __user *)arg, &state, sizeof(state))) 
                                printk("Error copying data state_led to user!\n");
                            break;
                case LED_SET_BLINK:
                        if(copy_from_user(&cfg, (struct mystruct *) arg, sizeof(cfg)))
                                printk("Error copying data from user!\n");
                        else {
                                printk("Update the number and timer to %d %d\n", cfg.number, cfg.interval_ms);
								int i;
                                for (i = 0; i < cfg.number; i++) {
                               		 led_on();
 					                 msleep(cfg.interval_ms);
 					                 led_off();
				 	                 msleep(cfg.interval_ms);	 
         				        }
                        }
                        break;
                default:
                        printk("Command not valdte!\n");
                        break;
        }
	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = my_open,
	.release = my_close,
	.read = my_read,
	.unlocked_ioctl = my_ioctl
};

static int __init func_init(void)
{	 
 	led = gpio_to_desc(IO_LED + IO_OFFSET);
	if (!led) {
   	   pr_err("Invalid GPIO pin: %d\n", IO_LED + IO_OFFSET);
	   return -EINVAL;
	}

	gpiod_direction_output(led, 0);
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

	my_class = class_create(THIS_MODULE, DEVICE_NAME);
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
