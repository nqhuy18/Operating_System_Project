#define pr_fmt(fmt) "%s %s: " fmt, KBUILD_MODNAME, __func__
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include "mpu6050.h"

dev_t devNr = 0;
static struct class *dev_class;
static struct cdev mpu_cdev;

static long mpu_ioctl(struct file *file, unsigned int cmd, unsigned long arg) { //NOLINT
  int aux;
  int16_t tmp;

  switch (cmd) {
  case READ_ACCELEROMETER:
    mpu_read_accelerometer_axis(&acc_read);
    if (copy_to_user((struct xyz_data*)arg, &acc_read, sizeof(xyz_data)) != 0) {
      pr_err("Failed READ_ACCELEROMETER");
    }
    break;

  case MPU_INFO:
    if (copy_to_user((struct mpu6050*)arg, &mpu_info, sizeof(mpu6050)) != 0) {
      pr_err("Failed MPU_INFO");
      break;
    } else {
      break;
    }

  case SET_AFS_SEL:
    if (copy_from_user(&aux, (int*)arg, sizeof(aux)) != 0) {
      pr_err("Failed to set AFS_SEL");
    } else {
      if ((aux == ACCEL_CONFIG_AFS_2G) || (aux == ACCEL_CONFIG_AFS_4G) ||
          (aux == ACCEL_CONFIG_AFS_8G) || (aux == ACCEL_CONFIG_AFS_16G)) {
        accel_config_g = aux;
        mpu_info.sensitivity = sensitivity_afssel[accel_config_g >> 3];
        pr_info("Setting accelerometer sensitivity: %d (%d)", accel_config_g,
                mpu_info.sensitivity);
        MPU_Write_Reg(ACCEL_CONFIG_ADDR, accel_config_g);
      } else {
        pr_err("Invalid AFS_SEL value");
      }
    }
    break;

  case READ_TEMPERATURE:
    mpu_read_temperature(&tmp);
    if (copy_to_user((int16_t*)arg, &tmp, sizeof(tmp)) != 0) {
      pr_err("Failed READ_TEMPERATURE");
      break;
    }
    break;

  case SET_SAMPLE_RATE:
    if (copy_from_user(&aux, (int*)arg, sizeof(aux))) {
      pr_err("Failed to IOCTL sample rate");
    } else {
      mpu_info.sample_rate = aux;
      mpu_set_sample_rate(mpu_info.sample_rate);
    }
    break;

  case FIFO_COUNT:
    mpu_read_fifo_count(&aux);
    if (copy_to_user((int*)arg, &aux, sizeof(aux)) != 0) {
      pr_err("Failed to IOCTL fifo count");
    }
    break;

  default:
    pr_info("IOCTL command defaulted");
    break;
  }
  return 0;
}



/* Create the i2c_device_id for your slave device and register that. */
static const struct of_device_id mpu_of_match[] = {
    { .compatible = "invensense,mpu6050" },
    { }
};
MODULE_DEVICE_TABLE(of, mpu_of_match);

/*
** This function getting called when the slave has been found
** Note : This will be called only once when we load the driver.
*/
static int mpu_probe(struct i2c_client *client,
                     const struct i2c_device_id *id) {
  unsigned char who_am_i;
  accel_config_g = ACCEL_CONFIG_AFS_4G;
  pr_info("Initializing driver");
  mpu_client = client;
  MPU_Read_Reg(WHO_AM_I_ADDR, &who_am_i);
  if (who_am_i != MPU6050_WHOAMI_VALUE) {
    pr_err("Bad device address: 0x%X", who_am_i);
    return -1;
  } else {
    pr_info("Found device on: 0x%X", who_am_i);
  }

  mpu_info.power_mgmt = PWR_MGMT_CLKSEL_PLL_X;
  mpu_info.sensitivity = sensitivity_afssel[accel_config_g >> 3];
  mpu_info.dlpf = CONFIG_DLPF_CFG_OFF;
  mpu_info.sample_rate = 150;

  MPU_Write_Reg(PWR_MGMT_ADDR, mpu_info.power_mgmt);
  MPU_Write_Reg(CONFIG, mpu_info.dlpf);
  MPU_Write_Reg(USER_CTRL, USER_CTL_FIFO_EN);
  MPU_Write_Reg(ACCEL_CONFIG_ADDR, accel_config_g);
  mpu_set_sample_rate(mpu_info.sample_rate);
  MPU_Write_Reg(FIFO_EN, FIFO_EN_ACCEL);
  pr_info("Done probing");
  return 0;
}

/*
** This function getting called when the slave has been removed
** Note : This will be called only once when we unload the driver.
*/

static int mpu_i2c_remove(struct i2c_client *client) {
  pr_info("Removing\n");
  MPU_Write_Reg(PWR_MGMT_ADDR, 0x80);
  return 0;
}


static struct i2c_driver mpu_driver = {
  .driver = {
    .name = MPU_NAME,
    .owner = THIS_MODULE,
    .of_match_table = mpu_of_match,
  },
  .probe = mpu_probe,
  .remove = mpu_i2c_remove,
};

static struct file_operations mpu_fops = {
  .owner = THIS_MODULE,
  .write = mpu_write,
  .unlocked_ioctl = mpu_ioctl,
  .read = mpu_read,
};

static int __init mpu_init(void) {
  if (alloc_chrdev_region(&devNr, 0, 1, MPU_NAME) < 0) {
    pr_err("Failed to allocate chr dev number");
    return -1;
  }

  cdev_init(&mpu_cdev, &mpu_fops);
  if (cdev_add(&mpu_cdev, devNr, 1) < 0) {
    pr_err("Could not add cdev.");
    goto r_class;
  }

  dev_class = class_create(THIS_MODULE, "mpu_class");
  if (dev_class == NULL) {
    pr_err("Failed to create device class");
    goto r_class;
  }

  if (device_create(dev_class, NULL, devNr, NULL, MPU_NAME) < 0) {
    pr_err("Failed to create the device");
    goto r_device;
  }
  return i2c_add_driver(&mpu_driver);

r_device:
  class_destroy(dev_class);
r_class:
  unregister_chrdev_region(devNr, 1);
  return -1;
}

static void __exit mpu_exit(void) {
  i2c_del_driver(&mpu_driver);

  device_destroy(dev_class, devNr);
  class_destroy(dev_class);
  unregister_chrdev_region(devNr, 1);
  
  pr_info("Driver removed\n");
}

module_init(mpu_init);
module_exit(mpu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Filipe do Ó Cavalcanti");
MODULE_DESCRIPTION("MPU6050 Kernel Module");
MODULE_VERSION("0.1.0");
