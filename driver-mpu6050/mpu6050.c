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

#define MPU_NAME "mpu6050"
#define MEM_SIZE 1024
#define MPU6050_WHOAMI_VALUE 0x68
#define WHO_AM_I_ADDR 0x75
#define CONFIG 0x1A             // Configure FSYNC pin and DLPF
#define USER_CTRL 0x6A          // User control register
#define PWR_MGMT_ADDR 0x6B      // Power Management Register
#define ACCEL_CONFIG_ADDR 0x1C  // Accelerometer Configuration (AFSEL)
#define TEMP_ADDR 0x41          // Temperature sensor address
#define ACC_XOUT0 0x3B          // First register for Accelerometer X
#define FIFO_R_W 0x74           // FIFO buffer
#define FIFO_COUNT_H 0x72       // FIFO Count Register 15:8 (0x73 for 7:0)
#define FIFO_EN 0x23            // Which sensor measurements are loaded
                                // into the FIFO buffer.
#define SMPRT_DIV 0x19          // divider from the gyroscope output rate
                                // used to generate the Sample Rate

const uint32_t sensitivity_afssel[4] = {16384, 8192, 4096, 2048};

static struct i2c_client *mpu_client;

dev_t devNr = 0;
static struct class *dev_class;
static struct cdev mpu_cdev;


static mpu6050 mpu_info;
struct xyz_data acc_read;
unsigned int fifo_count;

static DEFINE_MUTEX(mpu6050_mutex);
/**
 * MPU_Write_Reg() - Writes to a register.
 * @reg: Register that will be written.
 * @value: Value that will be written to the register.
 *
 * Return: Number of bytes written.
 */
static int MPU_Write_Reg(unsigned char reg, unsigned int value) {
  unsigned char send_buffer[2] = {reg, value};
  int ret;

  ret = i2c_master_send(mpu_client, send_buffer, 2);
  if (ret != 2) {
    pr_err("Failed writing register");
  }
  return ret;
}

/**
 * MPU_Read_Reg() - Read a single register.
 * @reg: Register that will be read.
 *
 * Return: bytes read.
 */

/**
 * MPU_Read_Reg_RACE() - Read a single register (UNSAFE).
 */
static int MPU_Read_Reg(unsigned char reg, unsigned char *rec_buf)
{
    int ret;
    /* Step 1: write register address */
    mutex_lock(&mpu6050_mutex);
    ret = i2c_master_send(mpu_client, &reg, 1);
    if (ret < 0) {
        pr_err("Error writing register address 0x%X\n", reg);
        return ret;
    }

    /* Step 2: read register data */
    ret = i2c_master_recv(mpu_client, rec_buf, 1);
    if (ret < 0) {
        pr_err("Error reading register 0x%X\n", reg);
        return ret;
    }
    mutex_unlock(&mpu6050_mutex);
    return ret;
}

/**
 * MPU_Burst_Read() - Read multiple registers in sequence.
 * @reg: First register of the reading sequence.
 * @length: Number of registers to be read.
 * @rec_buffer: Pointer to a buffer that will store the read data.
 *
 * Return: bytes sent and received
 */

/**
 * MPU_Burst_Read_RACE() - Read multiple registers (UNSAFE).
 */
unsigned char curr_reg;
static int MPU_Burst_Read(unsigned char start_reg,
                          unsigned int length,
                          unsigned char *rec_buffer)
{
    int ret;
    /* Step 1: write start register */
    mutex_lock(&mpu6050_mutex);
    curr_reg = start_reg;
    pr_info("[PID=%d] LOCKED MUTEX\n", current->pid);
    ret = i2c_master_send(mpu_client, &start_reg, 1);
    if (ret < 0) {
        pr_err("Error writing start register 0x%X\n", start_reg);
        return ret;
    }
    pr_info("[PID=%d] WRITE start_reg=0x%02X\n",
            current->pid, curr_reg);
    /* Step 2: read burst data */
    pr_info("[PID=%d] READ, start_reg=0x%02X\n",
            current->pid, curr_reg);
    ret = i2c_master_recv(mpu_client, rec_buffer, length);
    if (ret < 0) {
        pr_err("Error burst reading register 0x%X\n", start_reg);
        return ret;
    }
    mutex_unlock(&mpu6050_mutex);
    pr_info("[PID=%d] UNLOCKED MUTEX\n", current->pid);
    pr_info("\n");
    return ret;
}

void mpu_read_temperature(int16_t *temp) {
  unsigned char buf[2];
  MPU_Burst_Read(TEMP_ADDR, 2, buf);
  *temp = (buf[0] << 8) + buf[1];
  //pr_info("MPU6050 temperature raw = %d\n", *temp);
}

void mpu_read_accelerometer_axis(struct xyz_data *acc) {
  unsigned char test_buf[6];
  MPU_Burst_Read(FIFO_R_W, 6, test_buf);
  acc->x = (test_buf[0] << 8) + test_buf[1];
  acc->y = (test_buf[2] << 8) + test_buf[3];
  acc->z = (test_buf[4] << 8) + test_buf[5];
  //pr_info("MPU6050 accel raw: X=%d Y=%d Z=%d\n",
  //          acc->x, acc->y, acc->z);
}

void mpu_read_fifo_count(int *count) {
  unsigned char test_buf[2];
  MPU_Burst_Read(FIFO_COUNT_H, 2, test_buf);
  *count = (test_buf[0] << 8) + test_buf[1];
}

void mpu_set_sample_rate(int sample_rate) {
  unsigned char buf;
  if (sample_rate > 1000) {
    pr_info("Can not set sample rate over 1000 Hz");
  }
  mpu_info.sample_rate = sample_rate;
  if ((mpu_info.dlpf & 0x7) == CONFIG_DLPF_CFG_OFF) {
    mpu_info.sample_rate_divider = (8000/mpu_info.sample_rate) - 1;
  } else {
    mpu_info.sample_rate_divider = (1000/mpu_info.sample_rate) - 1;
  }
  MPU_Write_Reg(SMPRT_DIV, mpu_info.sample_rate_divider);
  MPU_Read_Reg(SMPRT_DIV, &buf);
  if (buf != mpu_info.sample_rate_divider) {
    pr_err("Failed to set sample rate. SMPRT_DIV: %d, got %d", SMPRT_DIV, buf);
  } else {
    pr_info("Sample rate set to %d Hz", mpu_info.sample_rate);
  }
}

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

static ssize_t mpu_read(struct file *filp, char __user *buf, size_t len,
                        loff_t *off) {
  int ret;
  unsigned char test_buf;
  ret = MPU_Read_Reg(0x75, &test_buf);
  if (copy_to_user(buf, &test_buf, 1)) {
          return -EFAULT;
      }
  return ret;
}

static ssize_t mpu_write(struct file *filp, const char *buf, size_t len,
                         loff_t *off) {
  uint8_t kbuf[2];
  int ret;
  if (len > 2) {
    pr_err("Too many fields written: only two permitted (reg, value)");
    return -1;
  }
  if (copy_from_user(kbuf, buf, len)) {
          return -EFAULT;
      }
  ret = MPU_Write_Reg(kbuf[0], kbuf[1]);
  return ret;
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
