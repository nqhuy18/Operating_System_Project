#ifndef DRIVER_MPU6050_H_
#define DRIVER_MPU6050_H_

// PWR_MGMT_1
#define PWR_MGMT_CLKSEL_INT 0x0
#define PWR_MGMT_CLKSEL_PLL_X 0x1
#define PWR_MGMT_CLKSEL_PLL_Y 0x2
#define PWR_MGMT_CLKSEL_PLL_Z 0x3
#define PWR_MGMT_DEVICE_RESET 0x80
#define PWR_MGMT_DEVICE_SLEEP 0x40

// CONFIG 0x1A
#define CONFIG_DLPF_CFG_OFF 0x00

// ACCEL_CONFIG 0x1C
enum accel_config {ACCEL_CONFIG_AFS_2G = 0x0,
                   ACCEL_CONFIG_AFS_4G = 0x8,
                   ACCEL_CONFIG_AFS_8G = 0x10,
                   ACCEL_CONFIG_AFS_16G = 0x18} accel_config_g;

// USER_CTRL 0x6A
#define USER_CTL_FIFO_EN 0x40
#define USER_CTL_FIFO_RESET 0x4

// FIFO_EN 0x23
#define FIFO_EN_ACCEL 0x8
#define FIFO_EN_GYRO_Z 0x10
#define FIFO_EN_GYRO_Y 0x20
#define FIFO_EN_GYRO_X 0x40
#define FIFO_EN_TEMP 0x80

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

#define READ_ACCELEROMETER _IOR('a', 'a', struct xyz_data)
#define MPU_INFO _IOR('a', 'b', struct mpu6050)
#define READ_TEMPERATURE _IOR('a', 'c', struct xyz_data)
#define FIFO_COUNT _IOR('a', 'd', struct xyz_data)
#define SET_SAMPLE_RATE _IOW('a', 'e', int*)
#define SET_AFS_SEL _IOW('a', 'f', int*)

#endif  // DRIVER_MPU6050_H_