/*
 *  zForce touchscreen driver
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
#include "ntx_hwconfig.h"
extern volatile NTX_HWCONFIG *gptHWCFG;

static const char ZFORCE_TS_NAME[]	= "zForce-ir-touch";
#define TS_POLL_PERIOD	msecs_to_jiffies(10) /* ms delay between samples */

#define tp_int_pin			183

#define DEFAULT_PANEL_W		600
#define DEFAULT_PANEL_H		800
//#define ZFORCE_TS_WIDTH			600
//#define ZFORCE_TS_HIGHT			800

//#define ZFORCE_TS_X_MAX 		ZFORCE_TS_WIDTH<<1
//#define ZFORCE_TS_Y_MAX 		ZFORCE_TS_HIGHT<<1
#define IDX_QUEUE_SIZE		20
#define IDX_PACKET_SIZE		129

static struct workqueue_struct *zForce_wq;
static uint16_t g_touch_pressed, g_touch_triggered;
static int g_zforce_initial_step;


static unsigned long ZFORCE_TS_WIDTH=DEFAULT_PANEL_W;
static unsigned long ZFORCE_TS_HIGHT=DEFAULT_PANEL_H;
static unsigned long ZFORCE_TS_X_MAX=DEFAULT_PANEL_W<<1; 
static unsigned long ZFORCE_TS_Y_MAX=DEFAULT_PANEL_H<<1; 

static struct zForce_data {
	int intr_gpio;
	struct delayed_work	work;
	struct i2c_client *client;
	struct input_dev *input;
	wait_queue_head_t wait;
} zForce_ir_touch_data;

static uint8_t cmd_Resolution_v2[] = {0xEE, 0x05, 0x02, (DEFAULT_PANEL_W&0xFF), (DEFAULT_PANEL_W>>8), (DEFAULT_PANEL_H&0xFF), (DEFAULT_PANEL_H>>8)};
static const uint8_t cmd_TouchData_v2[] = {0xEE, 0x01, 0x04};
static const uint8_t cmd_Frequency_v2[] = {0xEE, 0x07, 0x08, 10, 00, 100, 00, 100, 00};
static const uint8_t cmd_getFirmwareVer_v2[] = {0xEE, 0x01, 0x1E};
static const uint8_t cmd_Active_v2[] = {0xEE, 0x01, 0x01};
static const uint8_t cmd_Deactive_v2[] = {0xEE, 0x01, 0x00};

static uint8_t cmd_Resolution[] = {0x02, (DEFAULT_PANEL_W&0xFF), (DEFAULT_PANEL_W>>8), (DEFAULT_PANEL_H&0xFF), (DEFAULT_PANEL_H>>8)};
static const uint8_t cmd_TouchData[] = {0x04};
static const uint8_t cmd_Frequency[] = {0x08,10,100};
static const uint8_t cmd_getFirmwareVer[] = {0x0A};
static const uint8_t cmd_Active[] = {0x01};
static const uint8_t cmd_Deactive[] = {0x00};

extern unsigned int msp430_read(unsigned int reg);
/*--------------------------------------------------------------*/
static int zForce_ir_touch_detect_int_level(void)
{
	unsigned v;
	v = gpio_get_value(zForce_ir_touch_data.intr_gpio);
	return v;
}

static int __zForce_read_data (struct i2c_client *client, char* buffer)
{
	uint8_t buf_recv[2];
	int rc;
		
	while (zForce_ir_touch_detect_int_level())
		schedule_timeout(2);
	rc = i2c_master_recv(client, buf_recv, 2);
	if (0xEE != buf_recv[0]) {
		printk (KERN_ERR "[%s-%d] Error , frame start not found !!\n",__func__,__LINE__);
		return 0;
	}
	
	while (zForce_ir_touch_detect_int_level())
		schedule_timeout(2);
	return i2c_master_recv(client, buffer, buf_recv[1]);
}

/*	__zForce_ir_touch_init -- hand shaking with touch panel
 *
 *	1.recv hello packet
 */
static int __zForce_ir_touch_init(struct i2c_client *client)
{
	uint8_t buf_recv[10] = { 0 };
	uint8_t buf[10];

	if(!zForce_ir_touch_detect_int_level()){
		g_touch_triggered = 1;
		schedule_delayed_work(&zForce_ir_touch_data.work, 0);
	}else{
		printk (KERN_ERR "[%s-%d] zforce boot not completed !!\n",__func__,__LINE__);
		return 0;
	}

	if(8==gptHWCFG->m_val.bTouchCtrl) {
		return i2c_master_send(client, cmd_getFirmwareVer_v2, sizeof(cmd_getFirmwareVer_v2));
	}else{
		return i2c_master_send(client, cmd_getFirmwareVer, sizeof(cmd_getFirmwareVer));
	}	
}

static int zForce_ir_touch_recv_data(struct i2c_client *client, uint8_t *buf)
{
	uint8_t buf_recv[2]={0,};
	int result = 0;
	char *pBuffer;

	if (buf == NULL)
		return -EINVAL;

	i2c_master_recv(client, buf_recv, 2);
	if (0xEE != buf_recv[0]) {
		if (0xEE != buf_recv[1]) {
			printk (KERN_ERR "[%s-%d] Error , frame start not found !!\n",__func__,__LINE__);
			return 0;
		}
		else
			i2c_master_recv(client, &buf_recv[1], 1);
	}
	i2c_master_recv(client, buf, buf_recv[1]);
	
	switch (buf[0]) {
		case 0:
			printk ("[%s-%d] command Deactivate ...\n",__func__,__LINE__);
			break;
		case 1:
			printk ("[%s-%d] command Activate (%d) ...\n",__func__,__LINE__,buf[1]);
			if(8==gptHWCFG->m_val.bTouchCtrl) {   //neonode v2
				i2c_master_send(client, cmd_Resolution_v2, sizeof(cmd_Resolution_v2));
			}else{
				i2c_master_send(client, cmd_Resolution, sizeof(cmd_Resolution));
			}
			g_zforce_initial_step = 1;
			break;
		case 2:
			printk ("[%s-%d] command Resolution (%d) ...\n",__func__,__LINE__,buf[1]);
			if(8==gptHWCFG->m_val.bTouchCtrl) {    //neonode v2
				i2c_master_send(client, cmd_Frequency_v2, sizeof(cmd_Frequency_v2));
			}else{
				i2c_master_send(client, cmd_Frequency, sizeof(cmd_Frequency));
			}	
			g_zforce_initial_step = 2;
			break;
		case 3:
			printk ("[%s-%d] command Configuration ...\n",__func__,__LINE__);
			break;
		case 4:
//			printk ("[%s-%d] command Touch Data (count %d)...\n",__func__,__LINE__,buf[1]);
//			printk ("[%02X %02X %02X %02X %02X %02X %02X]\n",buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8]);
			if(8==gptHWCFG->m_val.bTouchCtrl) {    //neonode v2
				memmove (buf, &buf[2], 9*buf[1]);
			}else{
				memmove (buf, &buf[2], 7*buf[1]);
			}
			result = 1;
			break;
		case 7:
			printk (KERN_ERR "[%s-%d] command BootComplete (%d)...\n",__func__,__LINE__,buf[1]);
			break;
		case 8:
			printk ("[%s-%d] command Frequency (%d) ...\n",__func__,__LINE__,buf[1]);
			if(8==gptHWCFG->m_val.bTouchCtrl) {  //neonode v2
				i2c_master_send(client, cmd_TouchData_v2, sizeof(cmd_TouchData_v2));
			}else{
				i2c_master_send(client, cmd_TouchData, sizeof(cmd_TouchData));
			}	
			g_zforce_initial_step = 8;
			break;
		case 0x0A:
			printk ("[%s-%d] firmware version %02X%02X %02X%02X %02X%02X %02X%02X \n", __func__, __LINE__, \
				buf[2], buf[1], buf[4], buf[3], buf[6], buf[5], buf[8], buf[7]);
			break;
		case 0x1E:
			if(8==gptHWCFG->m_val.bTouchCtrl) {  //neonode v2
				printk ("[%s-%d] firmware version %02X%02X %02X%02X %02X%02X %02X%02X \n", __func__, __LINE__, \
					buf[2], buf[1], buf[4], buf[3], buf[6], buf[5], buf[8], buf[7]);
			}
			break;
		case 0x25:
			printk (KERN_ERR "[%s-%d] command overrun (%d) ...\n",__func__,__LINE__,g_zforce_initial_step);
			switch (g_zforce_initial_step) {
				case 1:
					if(8==gptHWCFG->m_val.bTouchCtrl) {   //neonode v2
						i2c_master_send(client, cmd_Resolution_v2, sizeof(cmd_Resolution_v2));
					}else{
						i2c_master_send(client, cmd_Resolution, sizeof(cmd_Resolution));
					}
					break;
				case 2:
					if(8==gptHWCFG->m_val.bTouchCtrl) {    //neonode v2
						i2c_master_send(client, cmd_Frequency_v2, sizeof(cmd_Frequency_v2));
					}else{
						i2c_master_send(client, cmd_Frequency, sizeof(cmd_Frequency));
					}
					break;
				case 8:
					if(8==gptHWCFG->m_val.bTouchCtrl) {  //neonode v2
						i2c_master_send(client, cmd_TouchData_v2, sizeof(cmd_TouchData_v2));
					}else{
						i2c_master_send(client, cmd_TouchData, sizeof(cmd_TouchData));
					}	
					break;
			}
			break;
		default:
			printk (KERN_ERR "[%s-%d] undefined command %d (%d bytes)...\n",__func__,__LINE__, *buf, buf_recv[1]);
			break;
	}
	return result;
}

extern int gIsCustomerUi;
static void zForce_ir_touch_report_data(struct i2c_client *client, uint8_t *buf)
{
	static uint16_t last_x, last_y;
	static int	pressure = 100;
	uint16_t x1, x2, y1, y2;
	int state;
	if(8==gptHWCFG->m_val.bTouchCtrl) {
		state = buf[4] & 0x03;
	}else{
		state = (buf[4] >> 6) & 0x03;
	}
	
	if (2 == state) {
		input_report_abs(zForce_ir_touch_data.input, ABS_X, last_x);
		input_report_abs(zForce_ir_touch_data.input, ABS_Y, last_y);
		input_report_abs(zForce_ir_touch_data.input, ABS_PRESSURE, 0);	// Joseph 20101023
		input_report_key(zForce_ir_touch_data.input, BTN_TOUCH, 0);
		g_touch_pressed = 0;
//		printk ("[%s-%d] touch up (%d, %d) (%02X, %02X, %02X)\n",__func__,__LINE__, last_x, last_y,buf[4], buf[5], buf[6]);
	} 
	else {
		pressure ^= 1;  
		if (gIsCustomerUi) 
		{
			x1 = buf[0] | (buf[1] << 8);
			y1 = buf[2] | (buf[3] << 8);
			x1 = ZFORCE_TS_WIDTH-x1-1;
			input_report_abs(zForce_ir_touch_data.input, ABS_Y, x1);
			input_report_abs(zForce_ir_touch_data.input, ABS_X, y1);
			input_report_abs(zForce_ir_touch_data.input, ABS_PRESSURE, pressure);
			last_y = x1;
			last_x = y1;
//			printk ("[%s-%d] touch down (%d, %d, %d)\n",__func__,__LINE__,x1,y1, pressure);
		}
		else {
			y1 = buf[0] | (buf[1] << 8);
			x1 = buf[2] | (buf[3] << 8);
//			printk ("[%s-%d] touch down (%d, %d) (p=%d, %d, %d, %d, %d)\n",__func__,__LINE__,y1,x1,buf[4],buf[5],buf[6],buf[7],buf[8]);
			input_report_abs(zForce_ir_touch_data.input, ABS_X, x1);
			input_report_abs(zForce_ir_touch_data.input, ABS_Y, y1);
			input_report_abs(zForce_ir_touch_data.input, ABS_PRESSURE, 1024);
			input_report_key(zForce_ir_touch_data.input, BTN_TOUCH, 1);
			last_x = x1;
			last_y = y1;
		}
		input_report_key(zForce_ir_touch_data.input, BTN_TOUCH, 1);
		g_touch_pressed++;
	}
	input_sync(zForce_ir_touch_data.input);
	schedule();	// Joseph 20101023
}

static uint8_t gzForceBuffer[IDX_QUEUE_SIZE][IDX_PACKET_SIZE];
static int gQueueRear, gQueueFront;

void zForce_ir_touch_enqueue (void)
{
	if (zForce_ir_touch_recv_data(zForce_ir_touch_data.client, gzForceBuffer[gQueueRear])) {
		if (((gQueueRear+1)%IDX_QUEUE_SIZE) == gQueueFront)
			printk ("[%s-%d] touch queue full\n",__func__,__LINE__);
		else
			gQueueRear = (gQueueRear+1)%IDX_QUEUE_SIZE;
	}
}

static void zForce_ir_touch_work_func(struct work_struct *work)
{
	zForce_ir_touch_enqueue ();	
	while (gQueueRear != gQueueFront){
		zForce_ir_touch_report_data(zForce_ir_touch_data.client, gzForceBuffer[gQueueFront]);
		gQueueFront = (gQueueFront+1)%IDX_QUEUE_SIZE;
	}
	if (!zForce_ir_touch_detect_int_level ())
		schedule_delayed_work(&zForce_ir_touch_data.work, 1);
	else
		g_touch_triggered = 0;
}

static irqreturn_t zForce_ir_touch_ts_interrupt(int irq, void *dev_id)
{
	g_touch_triggered = 1;
	schedule_delayed_work(&zForce_ir_touch_data.work, 0);
	return IRQ_HANDLED;
}

void zForce_ir_touch_ts_triggered(void)
{
	g_touch_triggered = 1;
	schedule_delayed_work(&zForce_ir_touch_data.work, 0);
}

//tatic const struct i2c_device_id neonode_ts_id[] = {
//	{ "neonode_ts", 0 },
//	{ }
//};

static ssize_t neo_info(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return 0;
}

static ssize_t neo_ctl(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	switch(buf[0])
	{
		case 'a':
			printk("NeoActivate \n");
			if(8==gptHWCFG->m_val.bTouchCtrl) {
				i2c_master_send(zForce_ir_touch_data.client, cmd_Active_v2, sizeof(cmd_Active_v2));
			}else{
				i2c_master_send(zForce_ir_touch_data.client, cmd_Active, sizeof(cmd_Active));
			}	
			break;
		case 'd':
			printk("NeoDeactivate \n");
			if(8==gptHWCFG->m_val.bTouchCtrl) {
				i2c_master_send(zForce_ir_touch_data.client, cmd_Deactive_v2, sizeof(cmd_Deactive_v2));
			}else{
				i2c_master_send(zForce_ir_touch_data.client, cmd_Deactive, sizeof(cmd_Deactive));
			}	
			break;
		case 's':
			printk("NeoSetResolution \n");
			if(8==gptHWCFG->m_val.bTouchCtrl) {
				i2c_master_send(zForce_ir_touch_data.client, cmd_Resolution_v2, sizeof(cmd_Resolution_v2));
			}else{
				i2c_master_send(zForce_ir_touch_data.client, cmd_Resolution, sizeof(cmd_Resolution));
			}	
			break;
		case 'l':
			printk("Get LedSignalLevel \n");
//			i2c_master_send(zForce_ir_touch_data.client, cmd_LedLevel, sizeof(cmd_LedLevel));
			break;
	}
	return count;
}
static DEVICE_ATTR(neocmd, 0644, neo_info, neo_ctl);
extern int gSleep_Mode_Suspend;

static int zForce_ir_touch_suspend(struct platform_device *pdev, pm_message_t state)
{
//	printk ("[%s-%d] %s() %d\n",__FILE__,__LINE__,__func__,gSleep_Mode_Suspend);

	/* return immediatly if the driver is still handling touch data */
	if (g_touch_pressed || g_touch_triggered) {
		printk("[%s-%d] zForce still handling touch data\n");
		return -EBUSY;
	}

	/* the driver wants to send data, trigger a read */
	if (!zForce_ir_touch_detect_int_level ()) 
	{
		zForce_ir_touch_ts_triggered ();
		printk ("[%s-%d] zForce touch event not processed.\n",__func__,__LINE__);
		return -1;
	}
	if (gSleep_Mode_Suspend) {
		if(8==gptHWCFG->m_val.bTouchCtrl) {
			i2c_master_send(zForce_ir_touch_data.client, cmd_Deactive_v2, sizeof(cmd_Deactive_v2));
		}else{
			i2c_master_send(zForce_ir_touch_data.client, cmd_Deactive, sizeof(cmd_Deactive));
		}	
		msleep(200);
		disable_irq_wake(zForce_ir_touch_data.client->irq);
	}
	return 0;
}

static int zForce_ir_touch_resume(struct platform_device *pdev)
{
	if (gSleep_Mode_Suspend) {
		if(8==gptHWCFG->m_val.bTouchCtrl) {
			i2c_master_send(zForce_ir_touch_data.client, cmd_Active_v2, sizeof(cmd_Active_v2));
		}else{
			i2c_master_send(zForce_ir_touch_data.client, cmd_Active, sizeof(cmd_Active));
		}	
		enable_irq_wake(zForce_ir_touch_data.client->irq);
	}
	if (!zForce_ir_touch_detect_int_level ()) {
		zForce_ir_touch_ts_triggered ();
	}
	
	return 0;
}

static int zforce_i2c_open(struct input_dev *dev)
{
	struct i2c_client *client = input_get_drvdata(dev);

	printk ("[%s-%d] %s()\n",__FILE__,__LINE__,__func__);
	if(8==gptHWCFG->m_val.bTouchCtrl) {
		i2c_master_send(client, cmd_Active_v2, sizeof(cmd_Active_v2));
	}else{
		i2c_master_send(client, cmd_Active, sizeof(cmd_Active));
	}	
//	i2c_master_send(client, cmd_Resolution, sizeof(cmd_Resolution));
	
	return 0;
}

static void zforce_i2c_close(struct input_dev *dev)
{
	struct i2c_client *client = input_get_drvdata(dev);
	
	printk ("[%s-%d] %s()\n",__FILE__,__LINE__,__func__);
	if(8==gptHWCFG->m_val.bTouchCtrl) {
		i2c_master_send(client, cmd_Deactive_v2, sizeof(cmd_Deactive_v2));
	}else{
		i2c_master_send(client, cmd_Deactive, sizeof(cmd_Deactive));
	}	
}

static int zForce_ir_touch_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	zForce_ir_touch_data.client = client;
	strlcpy(client->name, ZFORCE_TS_NAME, I2C_NAME_SIZE);

	INIT_DELAYED_WORK(&zForce_ir_touch_data.work, zForce_ir_touch_work_func);
	
	zForce_ir_touch_data.intr_gpio = (client->dev).platform_data;
	
	zForce_ir_touch_data.input = input_allocate_device();
	if (zForce_ir_touch_data.input == NULL) {
		err = -ENOMEM;
		goto fail;
	}

	err = request_irq(zForce_ir_touch_data.client->irq, zForce_ir_touch_ts_interrupt, 0, ZFORCE_TS_NAME, ZFORCE_TS_NAME);
	if (err < 0) {
		printk(KERN_ERR "%s(%s): Can't allocate irq %d\n", __FILE__, __func__, zForce_ir_touch_data.client->irq);
	    goto fail;
	}
	enable_irq_wake(zForce_ir_touch_data.client->irq);

	err = __zForce_ir_touch_init(client);
	if (err < 0) {
	    printk(KERN_ERR "%s(%s): initial failed.\n", __FILE__, __func__);
	    goto fail;
	}

	zForce_ir_touch_data.input->name = ZFORCE_TS_NAME;
	zForce_ir_touch_data.input->id.bustype = BUS_I2C;
	zForce_ir_touch_data.input->open = zforce_i2c_open;
	zForce_ir_touch_data.input->close = zforce_i2c_close;

	input_set_drvdata(zForce_ir_touch_data.input, client);
	
	set_bit(EV_SYN, zForce_ir_touch_data.input->evbit);
	
	set_bit(EV_KEY, zForce_ir_touch_data.input->evbit);
	set_bit(BTN_TOUCH, zForce_ir_touch_data.input->keybit);
	set_bit(BTN_2, zForce_ir_touch_data.input->keybit);
	
	set_bit(EV_ABS, zForce_ir_touch_data.input->evbit);
	set_bit(ABS_X, zForce_ir_touch_data.input->absbit);
	set_bit(ABS_Y, zForce_ir_touch_data.input->absbit);
	set_bit(ABS_PRESSURE, zForce_ir_touch_data.input->absbit);
	set_bit(ABS_HAT0X, zForce_ir_touch_data.input->absbit);
	set_bit(ABS_HAT0Y, zForce_ir_touch_data.input->absbit);

    input_set_abs_params(zForce_ir_touch_data.input, ABS_X, 0, ZFORCE_TS_X_MAX, 0, 0);
	input_set_abs_params(zForce_ir_touch_data.input, ABS_Y, 0, ZFORCE_TS_Y_MAX, 0, 0);
	input_set_abs_params(zForce_ir_touch_data.input, ABS_PRESSURE, 0, 2048, 0, 0);
	input_set_abs_params(zForce_ir_touch_data.input, ABS_HAT0X, 0, ZFORCE_TS_X_MAX, 0, 0);
	input_set_abs_params(zForce_ir_touch_data.input, ABS_HAT0Y, 0, ZFORCE_TS_Y_MAX, 0, 0);

	{
		if(1==gptHWCFG->m_val.bDisplayResolution) {
			// 1024x758 .
			ZFORCE_TS_WIDTH=758;
			ZFORCE_TS_HIGHT=1024;
		}
		else if(2==gptHWCFG->m_val.bDisplayResolution) {
			// 1024x768
			ZFORCE_TS_WIDTH=768;
			ZFORCE_TS_HIGHT=1024;
		}
		else {
			// 800x600 
			ZFORCE_TS_WIDTH=600;
			ZFORCE_TS_HIGHT=800;
		}

		ZFORCE_TS_X_MAX=ZFORCE_TS_WIDTH<<1;
		ZFORCE_TS_Y_MAX=ZFORCE_TS_HIGHT<<1;

		cmd_Resolution[1] = (uint8_t)(ZFORCE_TS_WIDTH&0xff);
		cmd_Resolution[2] = (uint8_t)(ZFORCE_TS_WIDTH>>8);
		cmd_Resolution[3] = (uint8_t)(ZFORCE_TS_HIGHT&0xff);
		cmd_Resolution[4] = (uint8_t)(ZFORCE_TS_HIGHT>>8);

		cmd_Resolution_v2[3] = (uint8_t)(ZFORCE_TS_WIDTH&0xff);
		cmd_Resolution_v2[4] = (uint8_t)(ZFORCE_TS_WIDTH>>8);
		cmd_Resolution_v2[5] = (uint8_t)(ZFORCE_TS_HIGHT&0xff);
		cmd_Resolution_v2[6] = (uint8_t)(ZFORCE_TS_HIGHT>>8);
	}

	err = input_register_device(zForce_ir_touch_data.input);
	if (err < 0) {
		pr_debug("Register device file!\n");
		goto fail;
	}

	err = device_create_file(&client->dev, &dev_attr_neocmd);
	if (err) {
		pr_debug("Can't create device file!\n");
		return -ENODEV;
	}
	return 0;

fail:
	input_free_device(zForce_ir_touch_data.input);
	if (zForce_wq)
		destroy_workqueue(zForce_wq);
	return err;
}

static int zForce_ir_touch_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_neocmd);

	if (zForce_wq)
		destroy_workqueue(zForce_wq);

	input_unregister_device(zForce_ir_touch_data.input);

	free_irq(client->irq, ZFORCE_TS_NAME);
	return 0;
}

/* -------------------------------------------------------------------- */
static const struct i2c_device_id zForce_ir_touch_id[] = {
    {"zforce-ir-touch", 0 },
	{ }
};

static struct i2c_driver zForce_ir_touch_driver = {
	.probe		= zForce_ir_touch_probe,
	.remove		= zForce_ir_touch_remove,
	.suspend	= zForce_ir_touch_suspend,
	.resume		= zForce_ir_touch_resume,
	.id_table	= zForce_ir_touch_id,
	.driver		= {
		.name = "zforce-ir-touch",
		.owner = THIS_MODULE,
	},
};

static int __init zForce_ir_touch_init(void)
{
	return i2c_add_driver(&zForce_ir_touch_driver);
}

static void __exit zForce_ir_touch_exit(void)
{
	i2c_del_driver(&zForce_ir_touch_driver);
}

module_init(zForce_ir_touch_init);
module_exit(zForce_ir_touch_exit);

MODULE_AUTHOR("Joseph Lai. ");
MODULE_DESCRIPTION("NeoNode zForce IR Touch Screen driver");
MODULE_LICENSE("GPL");
