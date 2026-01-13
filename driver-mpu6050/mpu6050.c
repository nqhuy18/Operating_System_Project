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

static DEFINE_MUTEX(mpu6050_mutex);
const uint32_t sensitivity_afssel[4] = {16384, 8192, 4096, 2048};

struct xyz_data acc_read = {
    .x = 0,
    .y = 0,
    .z = 0,
};
enum accel_config accel_config_g = ACCEL_CONFIG_AFS_2G;

/**
 * MPU_Write_Reg() - Writes to a register.
 * @reg: Register that will be written.
 * @value: Value that will be written to the register.
 *
 * Return: Number of bytes written.
 */
  int MPU_Write_Reg(unsigned char reg, unsigned int value) {
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
int MPU_Read_Reg(unsigned char reg, unsigned char *rec_buf)
{
    int ret;
    /* Step 1: write register address */
    mutex_lock(&mpu6050_mutex);
    ret = i2c_master_send(mpu_client, &reg, 1);
    if (ret < 0) {
        pr_err("Error writing register address 0x%X\n", reg);
        goto unlock_exit;
    }

    /* Step 2: read register data */
    ret = i2c_master_recv(mpu_client, rec_buf, 1);
    if (ret < 0) {
        pr_err("Error reading register 0x%X\n", reg);
        goto unlock_exit;
    }
unlock_exit:
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
 int MPU_Burst_Read(unsigned char start_reg,
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
        goto unlock_exit;
    }
    pr_info("[PID=%d] WRITE start_reg=0x%02X\n",
            current->pid, curr_reg);
    /* Step 2: read burst data */
    pr_info("[PID=%d] READ, start_reg=0x%02X\n",
            current->pid, curr_reg);
    ret = i2c_master_recv(mpu_client, rec_buffer, length);
    if (ret < 0) {
        pr_err("Error burst reading register 0x%X\n", start_reg);
        goto unlock_exit;
    }
unlock_exit:
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

ssize_t mpu_read(struct file *filp, char __user *buf, size_t len,
                        loff_t *off) {
  int ret;
  unsigned char test_buf;
  ret = MPU_Read_Reg(0x75, &test_buf);
  if (copy_to_user(buf, &test_buf, 1)) {
          return -EFAULT;
      }
  return ret;
}

  ssize_t mpu_write(struct file *filp, const char *buf, size_t len,
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
