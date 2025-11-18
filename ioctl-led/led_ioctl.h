#ifndef LED_IOCTL_H
#define LED_IOCTL_H

/*
 *
 * Dùng chung cho:
 *  - Driver kernel:  #include "led_ioctl.h"
 *  - User-space app: #include "led_ioctl.h"
 *
 * Các chức năng:
 *  - LED_ON        : bật LED
 *  - LED_OFF       : tắt LED
 *  - LED_GET_STATE : đọc trạng thái LED (0 = OFF, 1 = ON)
 *  - LED_SET_BLINK : cấu hình nhấp nháy (số lần + khoảng thời gian ms)
 */

#ifdef __KERNEL__
    #include <linux/ioctl.h>
#else
    #include <sys/ioctl.h>
#endif

// Cấu hình nhấp nháy LED 
struct led_blink_cfg {
    int number;                 //Số lần nhấp nháy           
    int interval_ms;            //Thời gian mỗi trạng thái (ms)
};

#define LED_MAGIC  'L'

/*
 * Định nghĩa các ioctl command
 * _IO  : không truyền dữ liệu kèm theo
 * _IOR : driver copy dữ liệu về user
 * _IOW : user truyền dữ liệu xuống driver
 */

// Bật LED (không kèm tham số) 
#define LED_ON         _IO(LED_MAGIC, 1)

// Tắt LED (không kèm tham số)
#define LED_OFF        _IO(LED_MAGIC, 2)

// Đọc trạng thái LED: driver ghi 0/1 vào int* ở arg
#define LED_GET_STATE  _IOR(LED_MAGIC, 3, int)

// Cấu hình nhấp nháy: user truyền struct led_blink_cfg xuống driver 
#define LED_SET_BLINK  _IOW(LED_MAGIC, 4, struct led_blink_cfg)

#endif
