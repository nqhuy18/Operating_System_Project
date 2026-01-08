obj-m:= mpu6050.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

user:
		gcc test_ioctl.c -o test_ioctl -lm

kernel:
	make -C $(KDIR) M=$(PWD) modules

all: user kernel
clean:
	make -C $(KDIR) M=$(PWD) clean
	rm -rf test_ioctl
