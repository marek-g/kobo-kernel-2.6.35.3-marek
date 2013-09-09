#include <sys/ioctl.h>
#include <iostream>
#include <fcntl.h>	
#include "mxcfb.h"
#include <linux/fb.h>

using namespace std;


int main()
{
	int mode = AUTO_UPDATE_MODE_REGION_MODE;
	int framebuffer = open("/dev/fb0", O_RDWR); /* 0_RDONLY */
	if (framebuffer != -1)
	{
		ioctl(framebuffer, MXCFB_SET_AUTO_UPDATE_MODE, &mode);
		close(framebuffer);

		cout << "E-ink display autoupdate disabled.\n";
	}
	return 0;
}

