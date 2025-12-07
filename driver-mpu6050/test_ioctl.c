// 2025 Example - Userspace Menu Interface for MPU6050
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdlib.h>

#include "mpu6050.h"     // chứa READ_TEMPERATURE, MPU_INFO, READ_ACCELEROMETER, ...

#define DEV_PATH "/dev/mpu6050"

static void print_menu(void)
{
    printf("\n=========== MPU6050 CONTROL MENU ===========\n");
    printf("1. Read temperature\n");
    printf("2. Read accelerometer (1 time)\n");
    printf("3. Read accelerometer (N times)\n");
    printf("4. Read MPU information\n");
    printf("5. Set sample rate\n");
    printf("6. Set AFS_SEL (2g/4g/8g/16g)\n");
    printf("0. Exit\n");
    printf("Select: ");
}

static void clear_stdin(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {}
}

int main(void)
{
    int fd = open(DEV_PATH, O_RDWR);
    if (fd < 0) {
        perror("open " DEV_PATH);
        return 1;
    }

    int choice;
    int ret;

    while (1) {
        print_menu();

        if (scanf("%d", &choice) != 1) {
            printf("Invalid input.\n");
            clear_stdin();
            continue;
        }

        if (choice == 0) {
            break;
        }

        switch (choice)
        {
        case 1:   // ====================== READ TEMPERATURE =====================
        {
            int16_t temp_raw = 0;
            ret = ioctl(fd, READ_TEMPERATURE, &temp_raw);
            if (ret < 0) {
                perror("ioctl READ_TEMPERATURE");
            } else {
                float temp_c = (temp_raw / 340.0f) + 36.53f;
                printf("Temperature raw = %d, Temp = %.2f C\n", temp_raw, temp_c);
            }
            break;
        }

        case 2:   // ====================== READ ACCEL 1 TIME =====================
        {
            xyz_data acc;
            ret = ioctl(fd, READ_ACCELEROMETER, &acc);
            if (ret < 0) {
                perror("ioctl READ_ACCELEROMETER");
            } else {
                printf("Accel: X=%d, Y=%d, Z=%d\n", acc.x, acc.y, acc.z);
            }
            break;
        }

        case 3:   // ====================== READ ACCEL N TIMES =====================
        {
            int n;
            printf("Enter number of samples: ");
            if (scanf("%d", &n) != 1 || n <= 0) {
                printf("Invalid number.\n");
                clear_stdin();
                break;
            }

            xyz_data acc;
            for (int i = 0; i < n; i++) {
                ret = ioctl(fd, READ_ACCELEROMETER, &acc);
                if (ret < 0) {
                    perror("ioctl READ_ACCELEROMETER");
                    break;
                }
                printf("[%d] X=%d, Y=%d, Z=%d\n", i, acc.x, acc.y, acc.z);
                usleep(10000);
            }
            break;
        }

        case 4:   // ====================== READ MPU INFO =====================
        {
            mpu6050 info;
            ret = ioctl(fd, MPU_INFO, &info);
            if (ret < 0) {
                perror("ioctl MPU_INFO");
            } else {
                printf("MPU Info:\n");
                printf("  sensitivity = %.2f\n", info.sensitivity);
                printf("  sample_rate = %d\n", info.sample_rate);
            }
            break;
        }

        case 5:   // ====================== SET SAMPLE RATE =====================
        {
            int sr;
            printf("Enter sample rate (Hz): ");
            if (scanf("%d", &sr) != 1 || sr <= 0) {
                printf("Invalid sample rate.\n");
                clear_stdin();
                break;
            }

            ret = ioctl(fd, SET_SAMPLE_RATE, &sr);
            if (ret < 0) {
                perror("ioctl SET_SAMPLE_RATE");
            } else {
                printf("Sample rate set to %d Hz\n", sr);
            }
            break;
        }

        case 6:   // ====================== SET AFS_SEL =====================
        {
            printf("\nSelect AFS_SEL:\n");
            printf("1. 2g\n");
            printf("2. 4g\n");
            printf("3. 8g\n");
            printf("4. 16g\n");
            printf("Select: ");

            int sel;
            if (scanf("%d", &sel) != 1 || sel < 1 || sel > 4) {
                printf("Invalid selection.\n");
                clear_stdin();
                break;
            }

            uint32_t afs_table[4] = {
                ACCEL_CONFIG_AFS_2G,
                ACCEL_CONFIG_AFS_4G,
                ACCEL_CONFIG_AFS_8G,
                ACCEL_CONFIG_AFS_16G
            };

            uint32_t afs = afs_table[sel - 1];

            ret = ioctl(fd, SET_AFS_SEL, &afs);
            if (ret < 0) {
                perror("ioctl SET_AFS_SEL");
            } else {
                printf("AFS_SEL set to index %d (value = %u)\n", sel, afs);
            }
            break;
        }

        default:
            printf("Unknown choice: %d\n", choice);
            break;
        }
    }

    close(fd);
    return 0;
}
