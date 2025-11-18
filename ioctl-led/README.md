# BTL – User-space LED demo (ioctl)

Thư mục này chứa phần **chương trình user-space** dùng để điều khiển LED thông qua
device file `/dev/led` bằng lời gọi hệ thống **ioctl**.

## 1. Cấu trúc file

- `led_ioctl.h`  
  Định nghĩa giao diện điều khiển LED dùng chung cho driver và user:
  - `struct led_blink_cfg` – cấu hình nhấp nháy (số lần, khoảng thời gian ms)
  - Các lệnh ioctl:
    - `LED_ON`        – bật LED
    - `LED_OFF`       – tắt LED
    - `LED_GET_STATE` – đọc trạng thái LED (0 = OFF, 1 = ON)
    - `LED_SET_BLINK` – cấu hình nhấp nháy

- `led_user.c`  
  Chương trình C chạy trên **user-space**:
  - Mở device `/dev/led` bằng `open()`
  - Gọi `ioctl()` với các lệnh ở trên để:
    - Bật / tắt LED
    - Đọc trạng thái LED
    - Gửi cấu hình nhấp nháy (số lần, thời gian delay)
  - Hiển thị menu trên Terminal cho người dùng lựa chọn.

- `Makefile`  
  Dùng để biên dịch chương trình `led_user`:
  - `make` – biên dịch
  - `make clean` – xoá file thực thi

## 2. Yêu cầu trước khi chạy

- Kernel đã nạp module **driver LED** tương ứng (ví dụ module trong thư mục `05-ioctl-led`)
- Driver tạo ra device node **`/dev/led`**  
  (nếu device có tên khác, cần sửa macro `DEV_PATH` trong `led_user.c` cho khớp).

## 3. Cách biên dịch

```bash
make all

how to run?

Kernel_space: sudo insmod led_kernel.ko
              sudo chmod 666 /dev/led
Use_space: ./led_user

