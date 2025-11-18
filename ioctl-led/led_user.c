// Chức năng:
//   1. Bật / tắt LED
//   2. Đọc trạng thái LED
//   3. Cấu hình nhấp nháy (số lần + khoảng thời gian ms)

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>     
#include <fcntl.h>      
#include <sys/ioctl.h>  
#include <string.h>     

#include "led_ioctl.h"  

#define DEV_PATH "/dev/led"   
static void print_menu(void)
{
    printf("\n===== LED CONTROL MENU =====\n");
    printf("1. Turn LED ON\n");
    printf("2. Turn LED OFF\n");
    printf("3. Get LED state\n");
    printf("4. Configure blink (number + interval_ms)\n");
    printf("0. Exit\n");
    printf("Select: ");
}

static void clear_stdin(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {
        
    }
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
            printf("Invalid input. Please enter a number.\n");
            clear_stdin();
            continue;
        }

        if (choice == 0) {
            break;
        }

        switch (choice) {
        case 1:                                     // LED ON
            ret = ioctl(fd, LED_ON, NULL);
            if (ret < 0) {
                perror("ioctl LED_ON");
            } else {
                printf("LED turned ON\n");
            }
            break;

        case 2:                                     // LED OFF
            ret = ioctl(fd, LED_OFF, NULL);
            if (ret < 0) {
                perror("ioctl LED_OFF");
            } else {
                printf("LED turned OFF\n");
            }
            break;

        case 3: {                                   // GET STATE
            int state = -1;
            ret = ioctl(fd, LED_GET_STATE, &state);
            if (ret < 0) {
                perror("ioctl LED_GET_STATE");
            } else {
                printf("LED state: %s (%d)\n", state ? "ON" : "OFF", state);    // giả sử driver trả 0 = OFF, 1 = ON
            }
            break;
        }

        case 4: {                                    // SET BLINK
            struct led_blink_cfg cfg;
            printf("Enter number of blinks: ");
            if (scanf("%d", &cfg.number) != 1) {
                printf("Invalid number. Abort.\n");
                clear_stdin();
                break;
            }

            printf("Enter interval (ms): ");
            if (scanf("%d", &cfg.interval_ms) != 1) {
                printf("Invalid interval. Abort.\n");
                clear_stdin();
                break;
            }

            ret = ioctl(fd, LED_SET_BLINK, &cfg);
            if (ret < 0) {
                perror("ioctl LED_SET_BLINK");
            } else {
                printf("Blink config sent: number=%d, interval=%d ms\n",
                       cfg.number, cfg.interval_ms);
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
