# # Makefile cho phần user-space điều khiển LED bằng ioctl

# CC      = gcc
# CFLAGS  = -Wall -Wextra -O2
# TARGET  = led_user
# SRCS    = led_user.c

# all: $(TARGET)

# $(TARGET): $(SRCS) led_ioctl.h
# 	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS)

# clean:
# 	rm -f $(TARGET)

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
	
