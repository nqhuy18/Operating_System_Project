# BTL – User-space & Kernel-space MPU6050 demo (ioctl)

Thư mục này chứa phần **chương trình user-space** dùng để điều khiển MPU6050 thông qua
device file `/dev/mpu6050` bằng lời gọi hệ thống **ioctl**.

## 1. Cấu trúc file

- `mpu6050.h`  
  Định nghĩa giao diện điều khiển MPU6050 dùng chung cho driver và user:
  - `struct led_blink_cfg` – cấu hình nhấp nháy (số lần, khoảng thời gian ms)
  - Các lệnh ioctl:
    - `READ_ACCELEROMETER`        – Đọc giá trị góc
    - `READ_TEMPERATURE`       – Đọc giá trị nhiệt độ
    - `SET_SAMPLE_RATE` – Cấu hình tần số  
    - `MPU_INFO` – Đọc thông tin cảm biến

- `test_ioclt.c`  
  Chương trình C chạy trên **user-space**:
  - Mở device `/dev/mpu6050` bằng `open()`
  - Gọi `ioctl()` với các lệnh ở trên để:
    - Đọc giá trị cảm biến
    - Cáu hình tần số
  - Hiển thị menu trên Terminal cho người dùng lựa chọn.

- `Makefile`  
  Dùng để biên dịch chương trình `test_ioctl` `mpu6050.c` :
  - `make all` – biên dịch
  - `make clean` – xoá file thực thi

## 2. Yêu cầu trước khi chạy

- Load device-tree overlay vào kernel
- Kernel đã nạp module **driver MPU6050** tương ứng (ví dụ module trong thư mục `05-ioctl-led`)
- Driver tạo ra device node **`/dev/mpu6050`**  
  (nếu device có tên khác, cần sửa macro `DEV_PATH` trong `test_ioctl.c` cho khớp).

## 3. Cách biên dịch

```bash
make all

how to run?

Kernel_space: sudo insmod mpu6050.ko
              sudo chmod 666 /dev/mpu6050
Use_space: ./test_ioctl

Device-tree: sudo dtoverlay ./testoverlay.dtbo

