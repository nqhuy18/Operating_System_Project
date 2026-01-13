#ifndef DRIVER_MPU6050_H_
#define DRIVER_MPU6050_H_

// PWR_MGMT_1
#define PWR_MGMT_CLKSEL_INT 0x0
#define PWR_MGMT_CLKSEL_PLL_X 0x1
#define PWR_MGMT_CLKSEL_PLL_Y 0x2
#define PWR_MGMT_CLKSEL_PLL_Z 0x3
#define PWR_MGMT_DEVICE_RESET 0x80
#define PWR_MGMT_DEVICE_SLEEP 0x40

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
typedef struct xyz_data {
  int16_t x;
  int16_t y;
  int16_t z;
}xyz_data;

typedef struct mpu6050 {
  uint8_t dlpf;
  uint8_t power_mgmt;
  uint32_t sensitivity;
  uint32_t sample_rate;
  uint8_t sample_rate_divider;
}mpu6050;

extern const uint32_t sensitivity_afssel[4];

static struct i2c_client *mpu_client;

static mpu6050 mpu_info;
extern struct xyz_data acc_read;
extern unsigned int fifo_count;

// CONFIG 0x1A
#define CONFIG_DLPF_CFG_OFF 0x00

// ACCEL_CONFIG 0x1C
enum accel_config {
    ACCEL_CONFIG_AFS_2G = 0x0,
    ACCEL_CONFIG_AFS_4G = 0x8,
    ACCEL_CONFIG_AFS_8G = 0x10,
    ACCEL_CONFIG_AFS_16G = 0x18
};
extern enum accel_config accel_config_g;

// USER_CTRL 0x6A
#define USER_CTL_FIFO_EN 0x40
#define USER_CTL_FIFO_RESET 0x4

// FIFO_EN 0x23
#define FIFO_EN_ACCEL 0x8
#define FIFO_EN_GYRO_Z 0x10
#define FIFO_EN_GYRO_Y 0x20
#define FIFO_EN_GYRO_X 0x40
#define FIFO_EN_TEMP 0x80


#define MPU6050_MAGIC 'a'

#define READ_ACCELEROMETER _IOR(MPU6050_MAGIC, 'a', struct xyz_data)
#define MPU_INFO _IOR(MPU6050_MAGIC, 'b', struct mpu6050)
#define READ_TEMPERATURE _IOR(MPU6050_MAGIC, 'c', struct xyz_data)
#define FIFO_COUNT _IOR(MPU6050_MAGIC, 'd', struct xyz_data)
#define SET_SAMPLE_RATE _IOW(MPU6050_MAGIC, 'e', int*)
#define SET_AFS_SEL _IOW(MPU6050_MAGIC, 'f', int*)

 int MPU_Write_Reg(unsigned char reg, unsigned int value);
 int MPU_Read_Reg(unsigned char reg, unsigned char *rec_buf);
 int MPU_Burst_Read(unsigned char start_reg, unsigned int length, unsigned char *rec_buffer);
void mpu_read_temperature(int16_t *temp);
void mpu_read_accelerometer_axis(struct xyz_data *acc);
void mpu_read_fifo_count(int *count);
void mpu_set_sample_rate(int sample_rate);
 ssize_t mpu_read(struct file *filp, char __user *buf, size_t len, loff_t *off); 
 ssize_t mpu_write(struct file *filp, const char *buf, size_t len, loff_t *off);
#endif  // DRIVER_MPU6050_H_