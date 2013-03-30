/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/pmic_status.h>
#include <linux/pmic_external.h>

#define RTC_TIME_LSH		0
#define RTC_DAY_LSH		0
#define RTCALARM_TIME_LSH	0
#define RTCALARM_DAY_LSH	0
#define RTC_DISABLE_LSH	23
#define ISR1_1HZM_LSH	0
#define ISR1_TODAM_LSH	1


#define RTC_TIME_WID		17
#define RTC_DAY_WID		15
#define RTCALARM_TIME_WID	17
#define RTCALARM_DAY_WID	15
#define RTC_DISABLE_WID	1
#define ISR1_1HZM_WID	1
#define ISR1_TODAM_WID	1

extern void msp430_gettime( struct rtc_time *tm);
extern void msp430_settime( struct rtc_time *tm);
extern void msp430_getalarm( struct rtc_wkalrm *alrm);
extern void msp430_setalarm( struct rtc_wkalrm *alrm);
extern void msp430_clearalarm(void);

static struct rtc_wkalrm calrm;

static unsigned long rtc_status;
static unsigned long rtc_flag = 0;

extern int msp430_write(unsigned int reg, unsigned int value);
extern unsigned int msp430_read(unsigned int reg);
extern int g_wakeup_by_alarm;
extern int gIsCustomerUi;
extern int gIsMSP430IntTriggered;

static int mxc_rtc_open(struct device *dev)
{
	if (test_and_set_bit(1, &rtc_status))
		return -EBUSY;
	return 0;
}

static void mxc_rtc_release(struct device *dev)
{
	clear_bit(1, &rtc_status);
}

static void mxc_rtc_alarm_enable (int isEnable);

static int mxc_rtc_ioctl(struct device *dev, unsigned int cmd,
			 unsigned long arg)
{
	unsigned int tmp;
	switch (cmd) {
	case RTC_AIE_OFF:
		pr_debug("RTC alarm masked\n");
#if 0
		pmic_event_mask(EVENT_TODAI);
#else
		calrm.enabled = 0;
		mxc_rtc_alarm_enable (0);
#endif
		return 0;
	case RTC_AIE_ON:
		pr_debug("RTC alarm unmasked\n");
#if 0
		pmic_event_unmask(EVENT_TODAI);
#else
		calrm.enabled = 1;
		mxc_rtc_alarm_enable (1);
#endif
		return 0;
   case RTC_UIE_OFF:
		pr_debug("RTC update masked\n");
//		pmic_event_mask(EVENT_1HZI);
		return -ENOIOCTLCMD;
	case RTC_UIE_ON:
		pr_debug("RTC update unmasked\n");
//		pmic_event_unmask(EVENT_1HZI);
		return -ENOIOCTLCMD;
	/* RTC_PIE_x repurposed for RTC enable */
	case RTC_PIE_ON:
		pr_debug("RTC enabled\n");
//		CHECK_ERROR(pmic_write_reg(REG_RTC_ALARM, BITFVAL(RTC_DISABLE, 0), BITFMASK(RTC_DISABLE)));
		return -ENOIOCTLCMD;
	case RTC_PIE_OFF:
		pr_debug("RTC disabled\n");
//		CHECK_ERROR(pmic_write_reg(REG_RTC_ALARM, BITFVAL(RTC_DISABLE, 1), BITFMASK(RTC_DISABLE)));
		return -ENOIOCTLCMD;
	case RTC_WAKEUP_FLAG:
		tmp = msp430_read (0x60);
		printk ("[%s-%d] Micro P MSP430 status 0x%04X ....\n", __func__, __LINE__,tmp);
		put_user((0x8000 & tmp)?1:0, (unsigned long __user *)arg);
		gIsMSP430IntTriggered = 0;
		rtc_flag = 0;
		return 0;
	}

	return -ENOIOCTLCMD;
}

static int mxc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
#if 0
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0, day_reg_val2;
	unsigned int mask, value;
	unsigned long time;

	do {
		mask = BITFMASK(RTC_DAY);
		CHECK_ERROR(pmic_read_reg(REG_RTC_DAY, &value, mask));
		day_reg_val = BITFEXT(value, RTC_DAY);

		mask = BITFMASK(RTC_TIME);
		CHECK_ERROR(pmic_read_reg(REG_RTC_TIME, &value, mask));
		tod_reg_val = BITFEXT(value, RTC_TIME);

		mask = BITFMASK(RTC_DAY);
		CHECK_ERROR(pmic_read_reg(REG_RTC_DAY, &value, mask));
		day_reg_val2 = BITFEXT(value, RTC_DAY);
	} while (day_reg_val != day_reg_val2);

	time = (unsigned long)((unsigned long)(tod_reg_val &
					       0x0001FFFF) +
			       (unsigned long)(day_reg_val * 86400));

	rtc_time_to_tm(time, tm);
#else
	#if 1
	msp430_gettime(tm);
//	printk ("read_time: %d/%d/%d %d:%d:%d\n",tm->tm_year,tm->tm_mon,tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	#else
	unsigned int tmp;
//	printk ("[%s-%d]\n",__func__,__LINE__);
	tmp = msp430_read (0x20);
	tm->tm_year = ((tmp >> 8) & 0x0FF)+100;
	tm->tm_mon = (tmp & 0x0FF)-1;
	tmp = msp430_read (0x21);
	tm->tm_mday = (tmp >> 8) & 0x0FF;
	tm->tm_hour = tmp & 0x0FF;
	tmp = msp430_read (0x23);
	tm->tm_min = (tmp >> 8) & 0x0FF;
	tm->tm_sec = tmp & 0x0FF;
	#endif
#endif

	return 0;
}

static int mxc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val, day_reg_val2 = 0;
	unsigned int mask, value;
	unsigned long time;

	if (rtc_valid_tm(tm))
		return -1;

#if 0
	rtc_tm_to_time(tm, &time);

	tod_reg_val = time % 86400;
	day_reg_val = time / 86400;

	do {
		mask = BITFMASK(RTC_DAY);
		value = BITFVAL(RTC_DAY, day_reg_val);
		CHECK_ERROR(pmic_write_reg(REG_RTC_DAY, value, mask));

		mask = BITFMASK(RTC_TIME);
		value = BITFVAL(RTC_TIME, tod_reg_val);
		CHECK_ERROR(pmic_write_reg(REG_RTC_TIME, value, mask));

		mask = BITFMASK(RTC_DAY);
		CHECK_ERROR(pmic_read_reg(REG_RTC_DAY, &value, mask));
		day_reg_val2 = BITFEXT(value, RTC_DAY);
	} while (day_reg_val != day_reg_val2);
#else
//	printk ("[%s-%d] set %d/%d/%d %d:%d:%d\n",__func__,__LINE__,tm->tm_year,tm->tm_mon,tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	#if 1
	msp430_settime(tm);
	#else
	msp430_write (0x10, ((tm->tm_year-100)<<8));
	msp430_write (0x11, ((tm->tm_mon+1)<<8));
	msp430_write (0x12, (tm->tm_mday<<8));
	msp430_write (0x13, (tm->tm_hour<<8));
	msp430_write (0x14, (tm->tm_min<<8));
	msp430_write (0x15, (tm->tm_sec<<8));
	#endif
#endif
	return 0;
}

static int mxc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
#if 0	// Joseph 20110523, remove rtc
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;
	unsigned long time;

	mask = BITFMASK(RTCALARM_TIME);
	CHECK_ERROR(pmic_read_reg(REG_RTC_ALARM, &value, mask));
	tod_reg_val = BITFEXT(value, RTCALARM_TIME);

	mask = BITFMASK(ISR1_TODAM);
	CHECK_ERROR(pmic_read_reg(REG_INT_MASK1, &value, mask));
	alrm->enabled = 1 - BITFEXT(value, ISR1_TODAM);

	mask = BITFMASK(RTCALARM_DAY);
	CHECK_ERROR(pmic_read_reg(REG_RTC_DAY_ALARM, &value, mask));
	day_reg_val = BITFEXT(value, RTCALARM_DAY);

	time = (unsigned long)((unsigned long)(tod_reg_val &
					       0x0001FFFF) +
			       (unsigned long)(day_reg_val * 86400));
	rtc_time_to_tm(time, &(alrm->time));
#else
//	printk ("[%s-%d] %s()\n",__FILE__,__LINE__,__func__);
	msp430_getalarm(alrm);
	struct rtc_time *tm = &(alrm->time);
	printk ("read_alarm: %d %d/%d/%d %d:%d:%d\n",
		alrm->enabled,tm->tm_year,tm->tm_mon,tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
#endif
	return 0;
}

static void mxc_rtc_alarm_enable (int isEnable)
{
	if (isEnable) {
		printk ("[%s-%d] \n", __func__, __LINE__);
		return msp430_setalarm(&calrm);
	}
	else
		msp430_clearalarm();
}

static int mxc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
#if 0
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;
	unsigned long time;

	if (rtc_valid_tm(&alrm->time))
		return -1;

	rtc_tm_to_time(&alrm->time, &time);

	tod_reg_val = time % 86400;
	day_reg_val = time / 86400;

	mask = BITFMASK(RTCALARM_TIME);
	value = BITFVAL(RTCALARM_TIME, tod_reg_val);

	CHECK_ERROR(pmic_write_reg(REG_RTC_ALARM, value, mask));

	/* set the alarm interrupt mask appropriately */
	if(alrm->enabled){
		pmic_event_unmask(EVENT_TODAI);
	} else {
		pmic_event_mask(EVENT_TODAI);
	}

	mask = BITFMASK(RTCALARM_DAY);
	value = BITFVAL(RTCALARM_DAY, day_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_DAY_ALARM, value, mask));
#else
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	int ret;
	unsigned int tmp;
	
//	printk ("[%s-%d] set %d/%d/%d %d:%d:%d\n",__func__,__LINE__,alrm->time.tm_year,alrm->time.tm_mon,alrm->time.tm_mday,alrm->time.tm_hour,alrm->time.tm_min,alrm->time.tm_sec);
	if (rtc_valid_tm(&alrm->time))
		return -1;

	memcpy(&calrm, alrm, sizeof(struct rtc_wkalrm));
	if (calrm.enabled) 
		msp430_setalarm(&calrm);
#endif
	return 0;
}

struct rtc_drv_data {
	struct rtc_device *rtc;
	pmic_event_callback_t event;
};

static struct rtc_class_ops mxc_rtc_ops = {
	.open = mxc_rtc_open,
	.release = mxc_rtc_release,
	.ioctl = mxc_rtc_ioctl,
	.read_time = mxc_rtc_read_time,
	.set_time = mxc_rtc_set_time,
	.read_alarm = mxc_rtc_read_alarm,
	.set_alarm = mxc_rtc_set_alarm,
};

static void mxc_rtc_alarm_int(void *data)
{
#if 0
	struct rtc_drv_data *pdata = data;

	rtc_update_irq(pdata->rtc, 1, RTC_AF | RTC_IRQF);
	rtc_flag = 1;
#endif
}

static int mxc_rtc_probe(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = NULL;

	printk(KERN_INFO "mc13892 rtc probe start\n");

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);

	if (!pdata)
		return -ENOMEM;

	pdata->event.func = mxc_rtc_alarm_int;
	pdata->event.param = pdata;
#if 0
	CHECK_ERROR(pmic_event_subscribe(EVENT_TODAI, pdata->event));

	device_init_wakeup(&pdev->dev, 1);
#endif
	pdata->rtc = rtc_device_register(pdev->name, &pdev->dev,
					 &mxc_rtc_ops, THIS_MODULE);

	platform_set_drvdata(pdev, pdata);
	if (IS_ERR(pdata->rtc))
		return -1;

	printk(KERN_INFO "mc13892 rtc probe succeed\n");
	return 0;
}

static int __exit mxc_rtc_remove(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);

	rtc_device_unregister(pdata->rtc);
#if 0
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_TODAI, pdata->event));
#endif

	return 0;
}

static struct platform_driver mxc_rtc_driver = {
	.driver = {
		   .name = "pmic_rtc",
		   },
	.probe = mxc_rtc_probe,
	.remove = __exit_p(mxc_rtc_remove),
};

static int __init mxc_rtc_init(void)
{
	return platform_driver_register(&mxc_rtc_driver);
}

static void __exit mxc_rtc_exit(void)
{
	platform_driver_unregister(&mxc_rtc_driver);

}

module_init(mxc_rtc_init);
module_exit(mxc_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MC13892 Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
