#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/usbplugevent.h>
#include <linux/delay.h>

#define SINGLE_SHOT_TIME (6*1000) //in ms
#define MXC_CCM_CCGR2 0xF7ED4070	// remapped IO addr for CCGR2 reg
#define USB_PHY_CTR_FUNC 0xF7E80808	// remapped IO addr for USB_PHY_CTR reg

struct usb_plug_priv {
	struct device    *ddev;
	int (*get_status) (void);
};

static struct usb_plug_priv *mxc_usbplug;
extern void set_pmic_dc_charger_state(int dccharger);

static void usb_plug_handler(void *dummy)
{
	int plugged = mxc_usbplug->get_status();
	if (plugged) {
		int charger;
		int ret;
		volatile unsigned long *ccm_ccgr2 = (unsigned long*)(MXC_CCM_CCGR2);
		volatile unsigned long *usb_phy = (unsigned long*)(USB_PHY_CTR_FUNC);
		unsigned long old_ccgr2;
		unsigned long old_usbphy;
		
		/* query the i.MX to test for D+/D- charger detection */
		old_ccgr2 = *ccm_ccgr2;
		*ccm_ccgr2 = (old_ccgr2 | 0x0C000000); /* enable usbotg clocks temporarily */
		mdelay(100);
		old_usbphy = *usb_phy;   
		*usb_phy = (old_usbphy | 0x01800000);  /* enable charge detection circuit  */
		mdelay(100);	
		charger = (int)(*usb_phy & (0x00000004)); /* bit 2 = charger detection */
	
		/* restore old vals */
		*usb_phy = old_usbphy;
		*ccm_ccgr2 = old_ccgr2;

		if (charger) {
			set_pmic_dc_charger_state(1);
			ret = kobject_rename(&mxc_usbplug->ddev->kobj, "usb_plug");
		} else {
			ret = kobject_rename(&mxc_usbplug->ddev->kobj, "usb_host");
		}
		pr_info("usb plugged %d-%d\n", plugged, charger);
		kobject_uevent(&mxc_usbplug->ddev->kobj, KOBJ_ADD);
	} else {
		kobject_uevent(&mxc_usbplug->ddev->kobj, KOBJ_REMOVE);
		kobject_set_name(&mxc_usbplug->ddev->kobj, "%s", "usb_state");
		set_pmic_dc_charger_state(0);
		pr_info("usb unplugged\n");
	}
}

static void single_shot_worker(struct work_struct *work)
{
	usb_plug_handler(NULL);
}
static DECLARE_DELAYED_WORK(single_shot_work, single_shot_worker);

static int __devinit usb_plug_probe(struct platform_device *pdev)
{
	int ret;
	struct usbplug_event_platform_data *pdata = pdev->dev.platform_data;

	mxc_usbplug = kzalloc(sizeof(struct usb_plug_priv), GFP_KERNEL);
	if (!mxc_usbplug) {
		dev_err(&pdev->dev, "Error: kzalloc\n");
		ret = -ENOMEM;
		goto err_allocate;
	}

	mxc_usbplug->ddev = &pdev->dev;
	kobject_set_name(&mxc_usbplug->ddev->kobj, "%s", "usb_state");
	mxc_usbplug->get_status = pdata->get_key_status;
	pdata->register_usbplugevent(usb_plug_handler);
	schedule_delayed_work(&single_shot_work,
		msecs_to_jiffies(SINGLE_SHOT_TIME));
	return 0;

err_allocate:

	return ret;
}

static int __devexit usb_plug_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&single_shot_work);
	kfree(mxc_usbplug);
	return 0;
}

static struct platform_driver usb_plug_driver = {
	.probe		= usb_plug_probe,
	.remove		= __devexit_p(usb_plug_remove),
	.driver		= {
		.name	= "usb_plug",
		.owner	= THIS_MODULE,
	},
};

static int __init usb_plug_init(void)
{
	return platform_driver_register(&usb_plug_driver);
}
module_init(usb_plug_init);

static void __exit usb_plug_exit(void)
{
	platform_driver_unregister(&usb_plug_driver);
}
module_exit(usb_plug_exit);
