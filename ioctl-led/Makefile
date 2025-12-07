obj-m:= led_kernel.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

user:
        gcc led_user.c -o led_user

kernel:
        make -C $(KDIR) M=$(PWD) modules

all: user kernel
clean:
        make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
        rm -rf led_user
