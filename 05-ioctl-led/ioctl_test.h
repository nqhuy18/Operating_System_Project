#ifndef IOCTL_TEST_H
#define IOCTL_TEST_H

struct mystruct{
	int number;
	int timer;
};

#define WR_SLEEP _IOW('a', 'c', struct mystruct *)

#endif
