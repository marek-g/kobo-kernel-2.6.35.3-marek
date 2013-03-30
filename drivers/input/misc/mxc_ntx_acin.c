#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#define USB_PLUG_EVENT	0x0C

struct usb_insert_priv {
	struct input_dev *dev;
	int (*get_status) (void);
};

static struct usb_insert_priv *mxc_usb_insert;

static void mxc_usb_insert_handler(char inserted)
{
	printk ("[%s-%d] usb %s\n",__func__,__LINE__,(inserted)?"connected":"disconnected");
	input_report_switch(mxc_usb_insert->dev, USB_PLUG_EVENT , inserted);
}

typedef void (*usb_insert_handler) (char inserted);

static int __devinit usb_insert_probe(struct platform_device *pdev)
{
	int ret;
	usb_insert_handler *pdata = pdev->dev.platform_data;

	mxc_usb_insert = kzalloc(sizeof(struct usb_insert_priv), GFP_KERNEL);
	if (!mxc_usb_insert) {
		dev_err(&pdev->dev, "Error: kzalloc\n");
		ret = -ENOMEM;
		goto err_allocate;
	}

	mxc_usb_insert->dev = input_allocate_device();
	if (!mxc_usb_insert->dev) {
		dev_err(&pdev->dev, "Error: input_allocate_device\n");
		ret = -ENOMEM;
		goto err_input_allocate;
	}

	*pdata = mxc_usb_insert_handler;

	mxc_usb_insert->dev->evbit[0] = BIT_MASK(EV_SW);
	mxc_usb_insert->dev->swbit[BIT_WORD(USB_PLUG_EVENT)] = BIT_MASK(USB_PLUG_EVENT);
	mxc_usb_insert->dev->name = "usb_insert";
	mxc_usb_insert->dev->dev.parent = &pdev->dev;
	ret = input_register_device(mxc_usb_insert->dev);
	if (ret) {
		dev_dbg(&pdev->dev, "Error: input_register_device\n");
		goto err_input_register;
	}
	return 0;

err_input_register:
	input_free_device(mxc_usb_insert->dev);
err_input_allocate:
	kfree(mxc_usb_insert);
err_allocate:

	return ret;
}

static int __devexit usb_insert_remove(struct platform_device *pdev)
{
	input_unregister_device(mxc_usb_insert->dev);
	input_free_device(mxc_usb_insert->dev);
	kfree(mxc_usb_insert);
	return 0;
}

static struct platform_driver usb_insert_driver = {
	.probe		= usb_insert_probe,
	.remove		= __devexit_p(usb_insert_remove),
	.driver		= {
		.name	= "usb_insert",
		.owner	= THIS_MODULE,
	},
};

static int __init usb_insert_init(void)
{
	return platform_driver_register(&usb_insert_driver);
}
module_init(usb_insert_init);

static void __exit usb_insert_exit(void)
{
	platform_driver_unregister(&usb_insert_driver);
}

module_exit(usb_insert_exit);
