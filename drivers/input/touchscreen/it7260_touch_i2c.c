/*
 *  IT7260 touchscreen driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
      
#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>

//#define _WITH_DELAY_WORK_
static const char IT7260_TS_NAME[]	= "IT7260-touch";
#define TS_POLL_PERIOD	msecs_to_jiffies(10) /* ms delay between samples */

extern int gTSC2004_exist;		// Joseph 20100726

#define tp_int_pin			183

#define IT7260_TS_X_MAX 		1500<<1
#define IT7260_TS_Y_MAX 		1000<<1
#define IDX_QUEUE_SIZE		100
#define IDX_PACKET_SIZE		14

enum {
	hello_packet 			= 0x55,
	idx_coordinate_packet 	= 0x5a,
};

enum {
	idx_finger_state = 7,
};

static struct workqueue_struct *it7260_wq;
static uint16_t g_touch_pressed;


static struct it7260_data {
	int intr_gpio;
	int use_irq;
	struct hrtimer timer;
#ifdef _WITH_DELAY_WORK_
	struct delayed_work	work;
#else
	struct work_struct work;
#endif
	struct i2c_client *client;
	struct input_dev *input;
	wait_queue_head_t wait;
} it7260_touch_data;

extern unsigned int msp430_read(unsigned int reg);
/*--------------------------------------------------------------*/
int i2cReadFromIt7260(struct i2c_client *client, unsigned char bufferIndex,
		unsigned char dataBuffer[], unsigned short dataLength) {
	int ret;
	struct i2c_msg msgs[2] = { { .addr = client->addr, .flags = I2C_M_NOSTART,
			.len = 1, .buf = &bufferIndex }, { .addr = client->addr, .flags =
			I2C_M_RD, .len = dataLength, .buf = dataBuffer } };

	memset(dataBuffer, 0xFF, dataLength);
	ret = i2c_transfer(client->adapter, msgs, 2);
	return (ret == 2) ? dataLength : ret;
}

int i2cWriteToIt7260(struct i2c_client *client, unsigned char bufferIndex,
		unsigned char const dataBuffer[], unsigned short dataLength) {
	unsigned char buffer4Write[256];
	int ret;
	struct i2c_msg msgs[1] = { { .addr = client->addr, .flags = 0, .len =
			dataLength + 1, .buf = buffer4Write } };

	buffer4Write[0] = bufferIndex;
	memcpy(&(buffer4Write[1]), dataBuffer, dataLength);
	ret = i2c_transfer(client->adapter, msgs, 1);
	return (ret == 1) ? dataLength : ret;
}

static int it7260_touch_detect_int_level(void)
{
	unsigned v;
#if 0
	v = (msp430_read(0x60) & 0x10)?0:1;
	printk ("[%s-%d] int status %d\n",__func__,__LINE__,v);
#else
	v = gpio_get_value(it7260_touch_data.intr_gpio);
#endif

	return v;
}

static int __it7260_touch_poll(struct i2c_client *client)
{
	int status = 0, retry = 20;

	do {
		status = it7260_touch_detect_int_level();
		retry--;
		mdelay(20);
	} while (status == 1 && retry > 0);

	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int it7260_touch_poll(struct i2c_client *client)
{
	return __it7260_touch_poll(client);
}

static inline int it7260_touch_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];
	
	*x <<= 1;
	*y <<= 1;

	return 0;
}

/*	__it7260_touch_init -- hand shaking with touch panel
 *
 *	1.recv hello packet
 */
static int __it7260_touch_init(struct i2c_client *client)
{
	int rc;
	uint8_t cmdbuf[4] = { 0 };
	unsigned char ucQuery = 0;
	unsigned char buffer[128];
	
	// Identify Cap Sensor
	do {
		if (1 != i2cReadFromIt7260(client, 0x80, &ucQuery, 1))
			return -EINVAL;		
	} while (ucQuery & 0x01);
	buffer[0] = 0x00;
	if (1 != i2cWriteToIt7260(client, 0x20, buffer, 1))
		return -EINVAL;		
	do {
		if (1 != i2cReadFromIt7260(client, 0x80, &ucQuery, 1))
			return -EINVAL;		
	} while (ucQuery & 0x01);

	memset(&buffer, 0, sizeof(buffer));
	i2cReadFromIt7260(client, 0xA0, buffer, 10);
	buffer[10] = 0;
	printk("=IT7260_Init -- %s=\n", &buffer[1]);
	if (buffer[1] != 'I' || buffer[2] != 'T' || buffer[3] != 'E') {
		printk ("[%s-%d] IT7260 not found!\n",__func__,__LINE__);
		return -EINVAL;
	}

#if 0
	// Get firmware information
	do {
		i2cReadFromIt7260(client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);
	buffer[0] = 0x01;
	buffer[1] = 0x00;
	i2cWriteToIt7260(client, 0x20, buffer, 2);
	do {
		i2cReadFromIt7260(client, 0x80, &ucQuery, 1);
	} while (ucQuery & 0x01);
	memset(&buffer, 0, sizeof(buffer));
	i2cReadFromIt7260(client, 0xA0, buffer, 9);
	printk("=IT7260_firmware_ver -- ROM -%x.%x.%x.%x Flash-%x.%x.%x.%x=\n", buffer[1], buffer[2], 
		buffer[3], buffer[4], buffer[5],buffer[6],buffer[7],buffer[8]);
#endif

	//To reset point queue.
	i2cWriteToIt7260(client, 0x20, cmdbuf, 1);
	mdelay(10);
	i2cReadFromIt7260(client, 0xA0, cmdbuf, 2);
//	printk("[%s-%d] 0x%02x 0x%02x\n",__func__,__LINE__,cmdbuf[0],cmdbuf[1]);
	mdelay(10);

	return 0;
}

static int it7260_touch_recv_data(struct i2c_client *client, uint8_t *buf)
{
	unsigned char ucQuery = 0;
	int rc, bytes_to_recv = IDX_PACKET_SIZE;

	if (buf == NULL) {
		printk ("= Null buffer pointer =\n");
		return -EINVAL;
	}

	i2cReadFromIt7260(client, 0x80, &ucQuery, 1);
	if (ucQuery < 0) {
		printk ("=error Read_Point=\n");
		return -EINVAL;
	} 
	memset(buf, 0, bytes_to_recv);
	if (ucQuery & 0x80) {
		rc = i2cReadFromIt7260(client, 0xE0, buf, bytes_to_recv);
		if (rc != bytes_to_recv) {
			printk ("== read data error ( %d )===\n",rc);
			return -EINVAL;
		}
	}
//	printk("=== flag 0x%02X Point1 x=%d y=%d ===\n",buf[0], ((buf[3] & 0x0F) << 8) + buf[2], ((buf[3] & 0xF0) << 4) + buf[4]);
	return rc;
}

static uint8_t git7260Buffer[IDX_QUEUE_SIZE][IDX_PACKET_SIZE];
static int gQueueRear, gQueueFront;

void it7260_touch_enqueue (void)
{
	int result;
	
	result = it7260_touch_recv_data(it7260_touch_data.client, git7260Buffer[gQueueRear]);
	if (0 > result)
		return;
	if (((gQueueRear+1)%IDX_QUEUE_SIZE) == gQueueFront)
		printk ("[%s-%d] touch queue full\n",__func__,__LINE__);
	else
		gQueueRear = (gQueueRear+1)%IDX_QUEUE_SIZE;
}

static void it7260_touch_report_data(struct i2c_client *client, uint8_t *pucPoint)
{
	static int x[2] = { (int) -1, (int) -1 };
	static int y[2] = { (int) -1, (int) -1 };
	static bool finger[2] = { 0, 0 };
	bool flag = 0;
	int ret = 0;
	int finger2_pressed = 0;
	int xraw, yraw, xtmp, ytmp;
	char pressure_point;
	int i = 0;
	
	if (pucPoint[0] & 0xF0) {
		// gesture
		printk("(pucPoint[0] & 0xF0) is true, it's a gesture\n") ;
		printk("pucPoint[0]=%x\n", pucPoint[0]);
		return;
	}
	if (pucPoint[1] & 0x01) {
		// palm
		printk("pucPoint 1 is 0x01, it's a palm\n") ;
		return;
	}
	
	if (pucPoint[0] & 0x01) {
		xraw = ((pucPoint[3] & 0x0F) << 8) + pucPoint[2];
		yraw = ((pucPoint[3] & 0xF0) << 4) + pucPoint[4];
		pressure_point = pucPoint[5] & 0x0f;
//		printk(">>> flag 0x%02X Point1 x=%d y=%d (%d) <<<\n",pucPoint[0],xraw,yraw,gQueueFront);

#if 0
		input_report_abs(it7260_touch_data.input, ABS_MT_TRACKING_ID, 1);
		input_report_abs(it7260_touch_data.input, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(it7260_touch_data.input, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(it7260_touch_data.input, ABS_MT_POSITION_X, xraw);
		input_report_abs(it7260_touch_data.input, ABS_MT_POSITION_Y, yraw);
#else
		input_report_abs(it7260_touch_data.input, ABS_X, xraw);
		input_report_abs(it7260_touch_data.input, ABS_Y, yraw);
#endif
//		input_report_abs(it7260_touch_data.input, ABS_PRESSURE, pressure_point);
		input_report_abs(it7260_touch_data.input, ABS_PRESSURE, 1024);
		input_report_key(it7260_touch_data.input, BTN_TOUCH, 1);
		//input_mt_sync(it7260_touch_data.input);
		x[0] = xraw;
		y[0] = yraw;
		finger[0] = 1;
		g_touch_pressed = 1;
		//pr_info("=input Read_Point1 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
	} else {
#if 0
		input_report_abs(it7260_touch_data.input, ABS_MT_TRACKING_ID, 1);
		input_report_abs(it7260_touch_data.input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(it7260_touch_data.input, ABS_MT_WIDTH_MAJOR, pressure_point);
		input_report_abs(it7260_touch_data.input, ABS_MT_POSITION_X, x[0]);
		input_report_abs(it7260_touch_data.input, ABS_MT_POSITION_Y, y[0]);
#else
		input_report_abs(it7260_touch_data.input, ABS_X, x[0]);
		input_report_abs(it7260_touch_data.input, ABS_Y, y[0]);
#endif
		input_report_key(it7260_touch_data.input, BTN_TOUCH, 0);
		input_report_abs(it7260_touch_data.input, ABS_PRESSURE, 0);
		finger[0] = 0;
		g_touch_pressed = 0;
	}

#if 0
	if (pucPoint[0] & 0x02) {
		char pressure_point, z, w;
		xraw = ((pucPoint[7] & 0x0F) << 8) + pucPoint[6];
		yraw = ((pucPoint[7] & 0xF0) << 4) + pucPoint[8];

		pressure_point = pucPoint[9] & 0x0f;

		//pr_info("=Read_Point2 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
		input_report_abs(it7260_touch_data.input, ABS_MT_TRACKING_ID, 2);
		input_report_abs(it7260_touch_data.input, ABS_MT_TOUCH_MAJOR, 1);
		//input_report_abs(it7260_touch_data.input, ABS_MT_WIDTH_MAJOR, pressure_point);
		input_report_abs(it7260_touch_data.input, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(it7260_touch_data.input, ABS_MT_POSITION_X, xraw);
		input_report_abs(it7260_touch_data.input, ABS_MT_POSITION_Y, yraw);
		input_mt_sync(it7260_touch_data.input);
		x[1] = xraw;
		y[1] = yraw;
		finger[1] = 1;
		//pr_info("input Read_Point2 x=%d y=%d p=%d=\n",xraw,yraw,pressure_point);
	} else {
		input_report_abs(it7260_touch_data.input, ABS_MT_TRACKING_ID, 2);
		input_report_abs(it7260_touch_data.input, ABS_MT_TOUCH_MAJOR, 0);
		//input_report_abs(it7260_touch_data.input, ABS_MT_WIDTH_MAJOR, pressure_point);
		input_report_abs(it7260_touch_data.input, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(it7260_touch_data.input, ABS_MT_POSITION_X, x[1]);
		input_report_abs(it7260_touch_data.input, ABS_MT_POSITION_Y, y[1]);
		input_mt_sync(it7260_touch_data.input);
		finger[1] = 0;
	}
#endif
	input_sync(it7260_touch_data.input);
	schedule();	// Joseph 20101023

#ifdef	_WITH_DELAY_WORK_
	if (g_touch_pressed || !it7260_touch_detect_int_level ())
		schedule_delayed_work(&it7260_touch_data.work, 1);
#endif			
}

static void it7260_touch_work_func(struct work_struct *work)
{
	it7260_touch_enqueue ();	
	while (gQueueRear != gQueueFront){
		it7260_touch_report_data(it7260_touch_data.client, git7260Buffer[gQueueFront]);
		gQueueFront = (gQueueFront+1)%IDX_QUEUE_SIZE;
	}
}

static irqreturn_t it7260_touch_ts_interrupt(int irq, void *dev_id)
{
	// printk ("[%s-%d] ...\n", __func__, __LINE__);
#ifdef	_WITH_DELAY_WORK_
	schedule_delayed_work(&it7260_touch_data.work, 0);
#else
//	disable_irq(it7260_touch_data.client->irq);
//	it7260_touch_enqueue ();	
//	enable_irq(it7260_touch_data.client->irq);
	queue_work(it7260_wq, &it7260_touch_data.work);
#endif			

	return IRQ_HANDLED;
}

void it7260_touch_ts_triggered(void)
{
//	it7260_touch_enqueue ();	
#ifdef	_WITH_DELAY_WORK_
	schedule_delayed_work(&it7260_touch_data.work, 0);
#else
	queue_work(it7260_wq, &it7260_touch_data.work);
#endif			
}

static enum hrtimer_restart it7260_touch_timer_func(struct hrtimer *timer)
{
	queue_work(it7260_wq, &it7260_touch_data.work);
	hrtimer_start(&it7260_touch_data.timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static int it7260_touch_register_interrupt(struct i2c_client *client)
{
	int err = 0;

	if (client->irq) {
		it7260_touch_data.use_irq = 1;
		err = request_irq(client->irq, it7260_touch_ts_interrupt, IRQF_TRIGGER_FALLING,
				  IT7260_TS_NAME, &it7260_touch_data);

		if (err < 0) {
			printk("%s(%s): Can't allocate irq %d\n", __FILE__, __func__, client->irq);
			it7260_touch_data.use_irq = 0;
		}
		enable_irq_wake (client->irq);
	}

	if (!it7260_touch_data.use_irq) {
		hrtimer_init(&it7260_touch_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		it7260_touch_data.timer.function = it7260_touch_timer_func;
		hrtimer_start(&it7260_touch_data.timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	printk("it7260 ts starts in %s mode.\n",	it7260_touch_data.use_irq == 1 ? "interrupt":"polling");
	
	return 0;
}

static int it7260_touch_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (g_touch_pressed || !it7260_touch_detect_int_level ()) 
	{
		it7260_touch_ts_triggered ();
		printk ("[%s-%d] it7260 touch event not processed.\n",__func__,__LINE__);
		return -1;
	}
	return 0;
}

static int it7260_touch_resume(struct platform_device *pdev)
{
#if 0
	if (!it7260_touch_detect_int_level ()) {
		it7260_touch_ts_triggered ();
	}
#endif
	return 0;
}

static int it7260_touch_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	it7260_touch_data.client = client;
	it7260_touch_data.intr_gpio = (client->dev).platform_data;
	err = __it7260_touch_init(client);
	if (err < 0) {
	    printk("__it7260_touch_init Failed\n");
	    return err;
	}

	strlcpy(client->name, IT7260_TS_NAME, I2C_NAME_SIZE);
#ifdef	_WITH_DELAY_WORK_
	INIT_DELAYED_WORK(&it7260_touch_data.work, it7260_touch_work_func);
#else
	it7260_wq = create_singlethread_workqueue("it7260_wq");
	if (!it7260_wq) {
		err = -ENOMEM;
		goto fail;
	}
	INIT_WORK(&it7260_touch_data.work, it7260_touch_work_func);
#endif			
	gTSC2004_exist = 1;
	
	it7260_touch_data.input = input_allocate_device();
	if (it7260_touch_data.input == NULL) {
		err = -ENOMEM;
		goto fail;
	}

	it7260_touch_data.input->name = IT7260_TS_NAME;
	it7260_touch_data.input->id.bustype = BUS_I2C;
	
	set_bit(EV_SYN, it7260_touch_data.input->evbit);
	
	set_bit(EV_KEY, it7260_touch_data.input->evbit);
	set_bit(BTN_TOUCH, it7260_touch_data.input->keybit);
	set_bit(BTN_2, it7260_touch_data.input->keybit);
	
	set_bit(EV_ABS, it7260_touch_data.input->evbit);
	set_bit(ABS_X, it7260_touch_data.input->absbit);
	set_bit(ABS_Y, it7260_touch_data.input->absbit);
	set_bit(ABS_PRESSURE, it7260_touch_data.input->absbit);
	set_bit(ABS_HAT0X, it7260_touch_data.input->absbit);
	set_bit(ABS_HAT0Y, it7260_touch_data.input->absbit);

    input_set_abs_params(it7260_touch_data.input, ABS_X, 0, IT7260_TS_X_MAX, 0, 0);
	input_set_abs_params(it7260_touch_data.input, ABS_Y, 0, IT7260_TS_Y_MAX, 0, 0);
	input_set_abs_params(it7260_touch_data.input, ABS_PRESSURE, 0, 2048, 0, 0);
	input_set_abs_params(it7260_touch_data.input, ABS_HAT0X, 0, IT7260_TS_X_MAX, 0, 0);
	input_set_abs_params(it7260_touch_data.input, ABS_HAT0Y, 0, IT7260_TS_Y_MAX, 0, 0);

	err = input_register_device(it7260_touch_data.input);
	if (err < 0) {
		goto fail;
	}

	it7260_touch_register_interrupt(it7260_touch_data.client);

	return 0;

fail:
	input_free_device(it7260_touch_data.input);
	if (it7260_wq)
		destroy_workqueue(it7260_wq);
	return err;
}

static int it7260_touch_remove(struct i2c_client *client)
{
	if (it7260_wq)
		destroy_workqueue(it7260_wq);

	input_unregister_device(it7260_touch_data.input);

	if (it7260_touch_data.use_irq)
		free_irq(client->irq, client);
	else
		hrtimer_cancel(&it7260_touch_data.timer);
	return 0;
}

/* -------------------------------------------------------------------- */
static const struct i2c_device_id it7260_touch_id[] = {
    {"it7260-touch", 0 },
	{ }
};

static struct i2c_driver it7260_touch_driver = {
	.probe		= it7260_touch_probe,
	.remove		= it7260_touch_remove,
	.suspend	= it7260_touch_suspend,
	.resume		= it7260_touch_resume,
	.id_table	= it7260_touch_id,
	.driver		= {
		.name = "it7260-touch",
		.owner = THIS_MODULE,
	},
};

static int __init it7260_touch_init(void)
{
	return i2c_add_driver(&it7260_touch_driver);
}

static void __exit it7260_touch_exit(void)
{
	i2c_del_driver(&it7260_touch_driver);
}

module_init(it7260_touch_init);
module_exit(it7260_touch_exit);

MODULE_AUTHOR("Netronix Inc.");
MODULE_DESCRIPTION("IT7260 Touch Screen driver");
MODULE_LICENSE("GPL");
