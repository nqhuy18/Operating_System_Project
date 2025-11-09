#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "ioctl_test.h"

int main() {
	int answer;
	struct mystruct test = {10, 500};
	int dev = open("/dev/char_dd", O_WRONLY);
	if(dev == -1) {
		printf("Opening was not possible!\n");
		return -1;
	}


	ioctl(dev, WR_SLEEP, &test);

	printf("Opening was successfull!\n");
	close(dev);
	return 0;
}
