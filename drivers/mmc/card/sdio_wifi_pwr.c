#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>

extern void ntx_wifi_power_ctrl (int isWifiEnable);
static int __init sdio_wifi_pwr_init(void)
{
	ntx_wifi_power_ctrl(1);
	return 0;
}
module_init(sdio_wifi_pwr_init);

static void __exit sdio_wifi_pwr_exit(void)
{
	ntx_wifi_power_ctrl(0);
}
module_exit(sdio_wifi_pwr_exit);

MODULE_DESCRIPTION("sdio wifi power control driver");
MODULE_AUTHOR("Netronix");
MODULE_LICENSE("GPL");
