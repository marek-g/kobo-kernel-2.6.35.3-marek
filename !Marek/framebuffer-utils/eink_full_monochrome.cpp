#include <sys/ioctl.h>
#include <iostream>
#include <fcntl.h>
#include "mxcfb.h"
#include <linux/fb.h>

using namespace std;


int main()
{
	mxcfb_update_data update_data = {
		0, 0, 800, 600,
		WAVEFORM_MODE_AUTO,
	};
	update_data.update_mode = UPDATE_MODE_PARTIAL;
	update_data.flags = EPDC_FLAG_FORCE_MONOCHROME;


	int framebuffer = open("/dev/fb0", O_RDWR); /* 0_RDONLY */
	if (framebuffer != -1)
	{
		ioctl(framebuffer, MXCFB_SET_UPDATE_SCHEME, UPDATE_SCHEME_QUEUE);
		ioctl(framebuffer, MXCFB_SEND_UPDATE, &update_data);
		close(framebuffer);

		cout << "E-ink display fully updated (monochrome).\n";
	}
	return 0;
}
