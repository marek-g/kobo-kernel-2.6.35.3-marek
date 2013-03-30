/*
 * BQ27x00 g_sensor driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>

#define DRIVER_VERSION			"1.1.0"
#define BQ27510_REG_FLAGS		0x0A
struct bq27x00_device_info *mma7660_di;
static struct i2c_client 	*mma7660_client;
#define RES_BUFF_LEN		160
static char			res_buff[RES_BUFF_LEN];
enum {
	AXIS_DIR_X	= 0,
	AXIS_DIR_X_N,
	AXIS_DIR_Y,
	AXIS_DIR_Y_N,
	AXIS_DIR_Z,
	AXIS_DIR_Z_N,
};

enum {
	SLIDER_X_UP	= 0,
	SLIDER_X_DOWN	,
	SLIDER_Y_UP	,
	SLIDER_Y_DOWN	,
};

static char *axis_dir[6] = {
	[AXIS_DIR_X] 		= "x",
	[AXIS_DIR_X_N] 		= "-x",
	[AXIS_DIR_Y]		= "y",
	[AXIS_DIR_Y_N]		= "-y",
	[AXIS_DIR_Z]		= "z",
	[AXIS_DIR_Z_N]		= "-z",
};

#define POLL_INTERVAL		20
static int 			debug = 0;
static int 			emu_joystick = 0;
static int 			poll_int = POLL_INTERVAL;



#undef DBG
#define DBG(format, arg...) do { if (debug) printk(KERN_DEBUG "%s: " format "\n" , __FILE__ , ## arg); } while (0)
//#define DBG //


/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(g_sensor_id);
static DEFINE_MUTEX(g_sensor_mutex);

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27x00_device_info *di);
};

struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	struct bq27x00_access_methods	*bus;
	struct i2c_client	*client;
};


struct mma7660_status {
	u8 mode;
	u8 ctl1;
	u8 ctl2;
	u8 axis_dir_x;
	u8 axis_dir_y;
	u8 axis_dir_z;
	u8 slider_key[4];
};





static struct mma7660_status mma_status = {
	.mode 		= 0,
	.ctl1 		= 0,
	.ctl2 		= 0,
	.axis_dir_x	= AXIS_DIR_X,
	.axis_dir_y	= AXIS_DIR_Y,
	.axis_dir_z	= AXIS_DIR_Z,
	.slider_key	= {0,0,0,0},
};


static enum power_supply_property bq27x00_g_sensor_props[] = {
	POWER_SUPPLY_PROP_PRESENT,               // Boolean 1 -> g_sensor detected, 0 g_sensor not inserted
	POWER_SUPPLY_PROP_VOLTAGE_NOW,           // Measured Voltage cell pack in mV
	POWER_SUPPLY_PROP_CURRENT_NOW,           // Signed measured average current (1 sec) in mA. Negative means discharging, positive means charging. 
	POWER_SUPPLY_PROP_CAPACITY,              // Predicted remaining g_sensor capacity expressed as a percentage 0 - 100%
	POWER_SUPPLY_PROP_TEMP,                  // Battery temperature converted in Celsius
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,     // Time to discharge the g_sensor in minutes based on the average current. 65535 indicates charging cycle
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,      // Time to recharge the g_sensor in minutes based on the average current. 65535 indicates discharging cycle
};

enum {
	MMA_REG_R = 0,		/* read register 	*/
	MMA_REG_W,		/* write register 	*/
	MMA_DUMPREG,		/* dump registers	*/
	MMA_REMAP_AXIS,		/* remap directiron of 3-axis	*/
	MMA_DUMPAXIS,		/* dump current axis mapping	*/
	MMA_ENJOYSTICK,		/* enable joystick	emulation	*/
	MMA_DISJOYSTICK,	/* disable joystick emulation	*/
	MMA_SLIDER_KEY,		/* set slider key		*/
	MMA_DUMP_SLKEY,		/* dump slider key code setting	*/
	MMA_CMD_MAX
};
static char *command[MMA_CMD_MAX] = {
	[MMA_REG_R] 		= "readreg",
	[MMA_REG_W] 		= "writereg",
	[MMA_DUMPREG]		= "dumpreg",
	[MMA_REMAP_AXIS]	= "remapaxis",
	[MMA_DUMPAXIS]		= "dumpaxis",
	[MMA_ENJOYSTICK]	= "enjoystick",
	[MMA_DISJOYSTICK]	= "disjoystick",
	[MMA_SLIDER_KEY]	= "sliderkey",
	[MMA_DUMP_SLKEY]	= "dumpslkey",
};

enum {
	REG_XOUT = 0x00,
	REG_YOUT,
	REG_ZOUT,
	REG_TILT,
	REG_SRST,
	REG_SPCNT,
	REG_INTSU,
	REG_MODE,
	REG_SR,
	REG_PDET,
	REG_PD,
	REG_END,
};

/*
 * Common code for BQ27x00 devices
 */
/* parse command argument */
static void parse_arg(const char *arg, int *reg, int *value)
{
	const char *p;

	for (p = arg;; p++) {
		if (*p == ' ' || *p == '\0')
			break;
	}

	p++;

	*reg 	= simple_strtoul(arg, NULL, 16);
	*value 	= simple_strtoul(p, NULL, 16);
}

static void cmd_read_reg(const char *arg)
{
	int reg, value, ret;

	parse_arg(arg, &reg, &value);
	ret = i2c_smbus_read_byte_data(mma7660_client, reg);
	dev_info(&mma7660_client->dev, "read reg0x%x = %x\n", reg, ret);
	sprintf(res_buff,"OK:read reg 0x%02x = 0x%02x\n",reg,ret);
}


static void cmd_dumpreg(void) {
	int reg,ret;

	sprintf(res_buff,"OK:dumpreg result:\n");

	for (reg = REG_XOUT; reg <= REG_END; reg++) {
		ret = i2c_smbus_read_byte_data(mma7660_client, reg);
	//	sprintf(&(res_buff[strlen(res_buff)]),"%02x ",ret);
		printk("reg %x %x\n",reg, ret);
//		if ((reg % 16) == 15) {
//			sprintf(&(res_buff[strlen(res_buff)]),"\n");
//		}
	}

	sprintf(&(res_buff[strlen(res_buff)]),"\n");
}/* show the command result */
/* do the axis translation */
static void remap_axis(short *rem_x,short *rem_y,short *rem_z,short x,short y,short z) {

	switch (mma_status.axis_dir_x) {
		case AXIS_DIR_X:
			*rem_x		= x;
			break;
		case AXIS_DIR_X_N:
			*rem_x		= -x;
			break;
		case AXIS_DIR_Y:
			*rem_x		= y;
			break;
		case AXIS_DIR_Y_N:
			*rem_x		= -y;
			break;
		case AXIS_DIR_Z:
			*rem_x		= z;
			break;
		case AXIS_DIR_Z_N:
			*rem_x		= -z;
			break;
	}
	
	switch (mma_status.axis_dir_y) {
		case AXIS_DIR_X:
			*rem_y		= x;
			break;
		case AXIS_DIR_X_N:
			*rem_y		= -x;
			break;
		case AXIS_DIR_Y:
			*rem_y		= y;
			break;
		case AXIS_DIR_Y_N:
			*rem_y		= -y;
			break;
		case AXIS_DIR_Z:
			*rem_y		= z;
			break;
		case AXIS_DIR_Z_N:
			*rem_y		= -z;
			break;
	}
	
	switch (mma_status.axis_dir_z) {
		case AXIS_DIR_X:
			*rem_z		= x;
			break;
		case AXIS_DIR_X_N:
			*rem_z		= -x;
			break;
		case AXIS_DIR_Y:
			*rem_z		= y;
			break;
		case AXIS_DIR_Y_N:
			*rem_z		= -y;
			break;
		case AXIS_DIR_Z:
			*rem_z		= z;
			break;
		case AXIS_DIR_Z_N:
			*rem_z		= -z;
			break;
	}
}


/* undo the axis translation */
static void unmap_axis(short *unm_x,short *unm_y,short *unm_z,short x,short y,short z) {

	switch (mma_status.axis_dir_x) {
		case AXIS_DIR_X:
			*unm_x		= x;
			break;
		case AXIS_DIR_X_N:
			*unm_x		= -x;
			break;
		case AXIS_DIR_Y:
			*unm_y		= x;
			break;
		case AXIS_DIR_Y_N:
			*unm_y		= -x;
			break;
		case AXIS_DIR_Z:
			*unm_z		= x;
			break;
		case AXIS_DIR_Z_N:
			*unm_z		= -x;
			break;
	}
	
	switch (mma_status.axis_dir_y) {
		case AXIS_DIR_X:
			*unm_x		= y;
			break;
		case AXIS_DIR_X_N:
			*unm_x		= -y;
			break;
		case AXIS_DIR_Y:
			*unm_y		= y;
			break;
		case AXIS_DIR_Y_N:
			*unm_y		= -y;
			break;
		case AXIS_DIR_Z:
			*unm_z		= y;
			break;
		case AXIS_DIR_Z_N:
			*unm_z		= -y;
			break;
	}
	
	switch (mma_status.axis_dir_z) {
		case AXIS_DIR_X:
			*unm_x		= z;
			break;
		case AXIS_DIR_X_N:
			*unm_x		= -z;
			break;
		case AXIS_DIR_Y:
			*unm_y		= z;
			break;
		case AXIS_DIR_Y_N:
			*unm_y		= -z;
			break;
		case AXIS_DIR_Z:
			*unm_z		= z;
			break;
		case AXIS_DIR_Z_N:
			*unm_z		= -z;
			break;
	}
}


/* set the Zslider key mapping */
static void cmd_sliderkey(const char *arg) 
{
	unsigned char key_code[4];
	int i;
	char *endptr,*p;

	p = (char *)arg;

	for ( i = 0; i < 4 ; i++ ) {
		if (*p == '\0') {
			break;
		}

		key_code[i] = (char)simple_strtoul(p,&endptr,16);
		p = endptr +1;

	}

	if (i != 4) {
		sprintf (res_buff,"FAIL:sliderkey command failed,only %d args provided\n",i);
		printk ("%s: Failed to set slider key, not enough key code in command\n",__FUNCTION__);
		return;
	}


	DBG("%s: set slider key code  %02x %02x %02x %02x \n",__FUNCTION__,
		key_code[0],key_code[1],key_code[2],key_code[3]);


	for (i = 0; i < 4; i++) {
		mma_status.slider_key[i] = key_code[i];
	}

	sprintf(res_buff,"OK:set slider key ok, key code %02x %02x %02x %02x \n",
		key_code[0],key_code[1],key_code[2],key_code[3]);
}

/* remap the axis */
static void cmd_remap_axis(const char *arg,size_t count) 
{
	int 	buff_len; 	
	char	delimiters[] = " ,";

	char 	*token = NULL;
	
	int 	axis_cnt = 0;
	u8	axis_map[3];
	
	if (count > 63) {
		buff_len = 63;
	} else {
		buff_len = count;
	}

	while ((token = strsep((char **)&arg,delimiters)) != NULL) {

		if (strcmp(token,"")) {
			if (strcasecmp(token,"x") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_X;
			} else if (strcasecmp(token,"-x") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_X_N;
			} else if (strcasecmp(token,"y") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_Y;
			} else if (strcasecmp(token,"-y") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_Y_N;
			} else if (strcasecmp(token,"z") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_Z;
			} else if (strcasecmp(token,"-z") == 0) {
				axis_map[axis_cnt] = AXIS_DIR_Z_N;
			} else {
				sprintf (res_buff,"FAIL:remapaxis error,wrong argument\n");
				return;
			}

			axis_cnt ++;

			if (axis_cnt == 3) {
				break;
			}
		}
	}

	if (axis_cnt != 3) {
		sprintf(res_buff,"FAIL:remapaxis error, not enough parameters(%d)\n",axis_cnt);
		return;
	}


	if (((axis_map[0] & 0xfe) == (axis_map[1] & 0xfe)) || 
		((axis_map[0] & 0xfe) == (axis_map[2] & 0xfe)) ||
		((axis_map[2] & 0xfe) == (axis_map[1] & 0xfe))) {

		sprintf(res_buff,"FAIL:remapaxis error, duplicate axis mapping\n");
		return;
	}


	mma_status.axis_dir_x	= axis_map[0];
	mma_status.axis_dir_y	= axis_map[1];
	mma_status.axis_dir_z	= axis_map[2];

	sprintf(res_buff,"OK:remapaxis ok, new mapping : %s %s %s\n",axis_dir[mma_status.axis_dir_x],
		axis_dir[mma_status.axis_dir_y],axis_dir[mma_status.axis_dir_z]);
}



static void cmd_write_reg(const char *arg)
{
	int reg, value, ret, modereg;

	parse_arg(arg, &reg, &value);

	modereg = i2c_smbus_read_byte_data(mma7660_client, REG_MODE);

	ret = i2c_smbus_write_byte_data(mma7660_client, REG_MODE, modereg & (~0x01));
	ret = i2c_smbus_write_byte_data(mma7660_client, reg, value);
	ret = i2c_smbus_write_byte_data(mma7660_client, REG_MODE, modereg);

	dev_info(&mma7660_client->dev, "write reg %d to 0x%02X result %s\n",reg, value,
		 ret ? "failed" : "success");
	sprintf(res_buff, "OK:write reg 0x%02x data 0x%02x result %s\n",reg,value,
		 ret ? "failed" : "success");
}

/* parse the command passed into driver, and execute it */
static int exec_command(const char *buf, size_t count)
{
	const char *p, *s;
	const char *arg;
	int i, value = 0;

	for (p = buf;; p++) {
		if (*p == ' ' || *p == '\0' || p - buf >= count)
			break;
	}
	arg = p + 1;

	for (i = MMA_REG_R; i < MMA_CMD_MAX; i++) {
		s = command[i];
		if (s && !strncmp(buf, s, p - buf - 1)) {
			dev_info(&mma7660_client->dev, "command %s\n", s);
			printk("command %s\n",s);
			goto mma_exec_command;
		}
	}

	dev_err(&mma7660_client->dev, "command not found\n");
	sprintf(res_buff,"FAIL:command [%s] not found\n",s);
	return -1;

mma_exec_command:
	if (i != MMA_REG_R && i != MMA_REG_W)
		value = simple_strtoul(arg, NULL, 16);

	switch (i) {
	case MMA_REG_R:
		cmd_read_reg(arg);
		break;
	case MMA_REG_W:
		cmd_write_reg(arg);
		break;
	case MMA_DUMPREG:		/* dump registers	*/
		cmd_dumpreg();
		break;
	case MMA_REMAP_AXIS:		/* remap 3 axis's direction	*/
		cmd_remap_axis(arg,(count - (arg - buf)));
		break;
	case MMA_DUMPAXIS:		/* remap 3 axis's direction	*/
		sprintf(res_buff,"OK:dumpaxis : %s %s %s\n",axis_dir[mma_status.axis_dir_x],
			axis_dir[mma_status.axis_dir_y],axis_dir[mma_status.axis_dir_z]);
		break;
	case MMA_ENJOYSTICK: 
		emu_joystick = 1;
		sprintf(res_buff,"OK:joystick movement emulation enabled\n");
		break;
	case MMA_DISJOYSTICK:
		emu_joystick = 0;
		sprintf(res_buff,"OK:joystick movement emulation disabled\n");
		break;
	case MMA_SLIDER_KEY:		/* load offset drift registers	*/
		cmd_sliderkey(arg);
		break;
	case MMA_DUMP_SLKEY:
		sprintf(res_buff,"OK:dumpslkey 0x%02x 0x%02x 0x%02x 0x%02x\n",mma_status.slider_key[0],
			mma_status.slider_key[1],mma_status.slider_key[2],mma_status.slider_key[3]);
		break;
	default:
		dev_err(&mma7660_client->dev, "command is not found\n");
		sprintf (res_buff,"FAIL:no such command %d\n",i);
		break;
	}
	
	return 0;
}


/* accept the command */
static ssize_t mma7660_store(struct device *dev,struct device_attribute *attr, 
		const char *buf,size_t count)
{
	exec_command(buf, count);
	return count;
}




static int bq27x00_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	int ret;
	ret = di->bus->read(reg, rt_value, b_single, di);
	return ret;
}

/*
 * Return the g_sensor temperature in Celcius degrees
 * Or < 0 if something fails.
 */
static int bq27x00_g_sensor_supply_prop_present(struct bq27x00_device_info *di)
{
	int ret;
	int bat_det_flag = 0;

	ret = bq27x00_read(BQ27510_REG_FLAGS, &bat_det_flag, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	} 
	return 1;
}


/*
 * BQ27200 specific code
 */

static int mma7660_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;
	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
                  if(!b_single)
		    *rt_value = data[0]+(data[1]<<8);
                  else
                    *rt_value = data[0];
		  return 0;
		}
	}
	return err;
}
/* read sensor data from mma7660 */
static int mma7660_read_data(short *x, short *y, short *z, short *tilt) {

    *x = (signed char)i2c_smbus_read_byte_data(mma7660_client, REG_XOUT);
    *y = (signed char)i2c_smbus_read_byte_data(mma7660_client, REG_YOUT);
    *z = (signed char)i2c_smbus_read_byte_data(mma7660_client, REG_ZOUT);
    *tilt = (signed char)i2c_smbus_read_byte_data(mma7660_client, REG_TILT);

    if (*x & 0x20) {    /* check for the bit 5 */
        *x |= 0xffc0;
    }

    if (*y & 0x20) {    /* check for the bit 5 */
        *y |= 0xffc0;
    }

    if (*z & 0x20) {    /* check for the bit 5 */
        *z |= 0xffc0;
    }

    return 0;
}

static ssize_t mma7660_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int reg = 0;
	int rt_value = 0;

   short   x,y,z,tilt;
    short   x_remap,y_remap,z_remap;
    int     rep_x=0,rep_y=0;
    u8  zslide_x,zslide_y;
//  int swing;

    if (mma7660_read_data(&x,&y,&z,&tilt) != 0) {
        printk("mma7660 data read failed\n");
        return;
    }
	printk("x %d y %d z %d tilt 0x%02X\n",x,y,z,tilt);
  	//cmd_dumpreg();
	return sprintf(buf,"%s",res_buff);
}

//static irqreturn_t mma7660_irq(int irq, void *v)
//{
//}

int mma7660_read_orient (void)
{
	int result;
	
	result = i2c_smbus_read_byte_data(mma7660_client, REG_TILT);
	switch ((result >> 2) & 0x07) {
		case 1:		// Left
			return 2;
		case 2:		// Right
			return 4;
		case 5:		// Down
			return 1;
		case 6:		// UP
			return 3;
	}
}

static struct device_attribute mma7660_dev_attr = {
	.attr 	= {
		 .name = "mma7660",
		 .mode = S_IRUGO | S_IWUGO,
	},
	.show 	= mma7660_show,
	.store 	= mma7660_store,
};

static int mma7660_g_sensor_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	struct bq27x00_access_methods *bus;
	int num;
	int retval = 0;
	int ret;

	/* Get new ID for the new g_sensor device */
	retval = idr_pre_get(&g_sensor_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&g_sensor_mutex);
	retval = idr_get_new(&g_sensor_id, client, &num);
	mutex_unlock(&g_sensor_mutex);
	if (retval < 0)
		return retval;
	name = kasprintf(GFP_KERNEL, "mma7660-%d", num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &mma7660_read;
	di->bus = bus;
	di->client = client;

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	mma7660_di = di;
	mma7660_client = client;
/* create device file in sysfs as user interface */
	ret = device_create_file(&client->dev, &mma7660_dev_attr);
	printk(KERN_INFO "create file\n");
	if (ret) {
		printk(KERN_INFO "create file failed\n");	
		dev_err(&client->dev, "create device file failed!\n");
		retval = -EINVAL;
		goto batt_failed_1;
	}

	printk(KERN_INFO "3 try to create file\n");
/* enable gSensor mode 8g, measure */
#if 0
	i2c_smbus_write_byte_data(client, REG_MODE, 0x00);		/* standby mode   */
	i2c_smbus_write_byte_data(client, REG_SPCNT, 0x00);		/* no sleep count */
	i2c_smbus_write_byte_data(client, REG_INTSU, 0x00);		/* no interrupt   */
	i2c_smbus_write_byte_data(client, REG_PDET, 0xE0);		/* disable tap    */
	i2c_smbus_write_byte_data(client, REG_SR, 0x6B);		/* 4 measurement, 16 sample  */
	ret = i2c_smbus_write_byte_data(client, REG_MODE, 0x01);		
#else
	//Configure MMA7660FC as Portrait/Landscape Detection
	retval = i2c_smbus_write_byte_data(client, REG_MODE, 0x00);		//Standby Mode
	if (0 > retval)
		goto batt_failed_4;
	i2c_smbus_write_byte_data(client, REG_SPCNT, 0x00);		//No sleep count
//	i2c_smbus_write_byte_data(client, REG_INTSU, 0x03);		//Configure GINT Interrupt
	i2c_smbus_write_byte_data(client, REG_INTSU, 0x02);		//Configure GINT Interrupt
	i2c_smbus_write_byte_data(client, REG_PDET, 0xE0);		//No tap detection enabled
//	i2c_smbus_write_byte_data(client, REG_SR, 	0x34);		//8 samples/s, TILT debounce filter = 2
	i2c_smbus_write_byte_data(client, REG_SR, 	0xDD);		//4 samples/s, TILT debounce filter = 6
	i2c_smbus_write_byte_data(client, REG_PD, 	0x00);		//No tap detection debounce count enabled
	i2c_smbus_write_byte_data(client, REG_MODE, 0x41);		//Active Mode, INT = push-pull and active low
	
//    request_irq(client->irq, mma7660_irq, IRQF_TRIGGER_FALLING,	name, client);
//    enable_irq_wake (client->irq);
#endif
	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&g_sensor_mutex);
	idr_remove(&g_sensor_id, num);
	mutex_unlock(&g_sensor_mutex);

	return retval;
}

static int mma7660_g_sensor_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	mutex_lock(&g_sensor_mutex);
	idr_remove(&g_sensor_id, di->id);
	mutex_unlock(&g_sensor_mutex);

	kfree(di);

	return 0;
}

extern int gSleep_Mode_Suspend;
static int gSensorDisabled;
static int mma7660_g_sensor_resume(struct i2c_client *client)
{
	int retval = 0;
//	printk ("[%s-%d] ...\n",__func__,__LINE__);
//	i2c_smbus_write_byte_data(client, REG_MODE, 0x01);		/* active mode   */
	if (gSleep_Mode_Suspend) {
		if (gSensorDisabled) {
			enable_irq_wake (mma7660_client->irq);
			gSensorDisabled = 0;
		}
		//Configure MMA7660FC as Portrait/Landscape Detection
		retval = i2c_smbus_write_byte_data(mma7660_client, REG_MODE, 0x00);		//Standby Mode
		if (0 > retval)
			return 0;
		i2c_smbus_write_byte_data(mma7660_client, REG_SPCNT, 0x00);		//No sleep count
	//	i2c_smbus_write_byte_data(client, REG_INTSU, 0x03);		//Configure GINT Interrupt
		i2c_smbus_write_byte_data(client, REG_INTSU, 0x02);		//Configure GINT Interrupt
		i2c_smbus_write_byte_data(client, REG_PDET, 0xE0);		//No tap detection enabled
	//	i2c_smbus_write_byte_data(client, REG_SR, 	0x34);		//8 samples/s, TILT debounce filter = 2
		i2c_smbus_write_byte_data(client, REG_SR, 	0xDD);		//4 samples/s, TILT debounce filter = 6
		i2c_smbus_write_byte_data(mma7660_client, REG_PD, 	0x00);		//No tap detection debounce count enabled
		i2c_smbus_write_byte_data(mma7660_client, REG_MODE, 0x41);		//Active Mode, INT = push-pull and active low
	}
	return 0;
}

static int mma7660_g_sensor_suspend(struct i2c_client *client, pm_message_t state)
{
//	printk ("[%s-%d] ...\n",__func__,__LINE__);
//	i2c_smbus_write_byte_data(client, REG_MODE, 0x00);		/* standby mode   */
	i2c_smbus_read_byte_data(mma7660_client, REG_TILT);		// read TILT to clear interrupt status
	if (gSleep_Mode_Suspend) {
		disable_irq_wake (mma7660_client->irq);
		gSensorDisabled = 1;
	}
	return 0;
}

/*
 * Module stuff
 */

static const struct i2c_device_id mma7660_id[] = {
	{ "mma7660", 0 },
	{},
};

static struct i2c_driver mma7660_g_sensor_driver = {
	.driver = {
		.name = "mma7660",
	},
	.probe = mma7660_g_sensor_probe,
	.remove = mma7660_g_sensor_remove,
	.suspend = mma7660_g_sensor_suspend,
	.resume = mma7660_g_sensor_resume,
	.id_table = mma7660_id,
};

static int __init bq27x00_g_sensor_init(void)
{
	int ret;

	ret = i2c_add_driver(&mma7660_g_sensor_driver);
	if (ret)
		printk(KERN_ERR "Unable to register MMA7660 g_sensor driver\n");

	return ret;
}
//module_init(bq27x00_g_sensor_init);
late_initcall(bq27x00_g_sensor_init);

static void __exit bq27x00_g_sensor_exit(void)
{
	i2c_del_driver(&mma7660_g_sensor_driver);
}
module_exit(bq27x00_g_sensor_exit);
module_param(debug, bool, S_IRUGO | S_IWUSR);
module_param(emu_joystick, bool, S_IRUGO | S_IWUSR);
module_param(poll_int, int, S_IRUGO | S_IWUSR);

MODULE_PARM_DESC(debug, "1: Enable verbose debugging messages");
MODULE_PARM_DESC(emu_joystick, "1: Enable emulate joystick movement by tilt");
MODULE_PARM_DESC(poll_int, "set the poll interval of gSensor data (unit: ms)");

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("MMA7660 g_sensor monitor driver");
MODULE_LICENSE("GPL");
