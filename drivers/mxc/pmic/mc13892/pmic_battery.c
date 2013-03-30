/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * Includes
 */
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <linux/pmic_battery.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>
#include <linux/reboot.h>

#define BIT_CHG_VOL_LSH		0
#define BIT_CHG_VOL_WID		3

#define BIT_CHG_CURR_LSH		3
#define BIT_CHG_CURR_WID		4

#define BIT_CHG_PLIM_LSH		15
#define BIT_CHG_PLIM_WID		2

#define BIT_CHG_DETS_LSH 6
#define BIT_CHG_DETS_WID 1
#define BIT_CHG_CURRS_LSH 11
#define BIT_CHG_CURRS_WID 1

#define BIT_CHG_LOBATHS_LSH 14
#define BIT_CHG_LOBATHS_WID 1

#define TRICKLE_CHG_EN_LSH	7
#define	LOW_POWER_BOOT_ACK_LSH	8
#define BAT_TH_CHECK_DIS_LSH	9
#define	BATTFET_CTL_EN_LSH	10
#define BATTFET_CTL_LSH		11
#define	REV_MOD_EN_LSH		13
#define PLIM_DIS_LSH		17
#define	CHG_LED_EN_LSH		18
#define CHGTMRRST_LSH		19
#define RESTART_CHG_STAT_LSH	20
#define	AUTO_CHG_DIS_LSH	21
#define CYCLING_DIS_LSH		22
#define	VI_PROGRAM_EN_LSH	23

#define TRICKLE_CHG_EN_WID	1
#define	LOW_POWER_BOOT_ACK_WID	1
#define BAT_TH_CHECK_DIS_WID	1
#define	BATTFET_CTL_EN_WID	1
#define BATTFET_CTL_WID		1
#define	REV_MOD_EN_WID		1
#define PLIM_DIS_WID		1
#define	CHG_LED_EN_WID		1
#define CHGTMRRST_WID		1
#define RESTART_CHG_STAT_WID	1
#define	AUTO_CHG_DIS_WID	1
#define CYCLING_DIS_WID		1
#define	VI_PROGRAM_EN_WID	1

#define ACC_STARTCC_LSH		0
#define ACC_STARTCC_WID		1
#define ACC_RSTCC_LSH		1
#define ACC_RSTCC_WID		1
#define ACC_CCFAULT_LSH		7
#define ACC_CCFAULT_WID		7
#define ACC_CCOUT_LSH		8
#define ACC_CCOUT_WID		16
#define ACC1_ONEC_LSH		0
#define ACC1_ONEC_WID		15

#define ACC_CALIBRATION 0x17
#define ACC_START_COUNTER 0x07
#define ACC_STOP_COUNTER 0x2
#define ACC_CONTROL_BIT_MASK 0x1f
#define ACC_ONEC_VALUE 2621
#define ACC_COULOMB_PER_LSB 1
#define ACC_CALIBRATION_DURATION_MSECS 20

#define BAT_VOLTAGE_UNIT_UV (4800000/1023)
#define BAT_CURRENT_UNIT_UA (3000000/511)
#define CHG_VOLTAGE_UINT_UV 23474
#define CHG_MIN_CURRENT_UA 3500

#define COULOMB_TO_UAH(c) (10000 * c / 36)

#define ICHRG_400MA	0x4
#define ICHRG_480MA	0x5
#define ICHRG_560MA	0x6
#define ICHRG_640MA	0x7
#define ICHRG_720MA 	0x8

#define BAT_CAP_MAH 1000UL
#define CHG_CUR_MA 400UL

enum chg_setting {
       TRICKLE_CHG_EN,
       LOW_POWER_BOOT_ACK,
       BAT_TH_CHECK_DIS,
       BATTFET_CTL_EN,
       BATTFET_CTL,
       REV_MOD_EN,
       PLIM_DIS,
       CHG_LED_EN,
       CHGTMRRST,
       RESTART_CHG_STAT,
       AUTO_CHG_DIS,
       CYCLING_DIS,
       VI_PROGRAM_EN
};

enum chg_state {
	CHG_POWER_OFF,
	CHG_RESTART,
	CHG_CHARGING,
	CHG_DISCHARGING_WITH_CHARGER,
	CHG_DISCHARGING,
};

/* Flag used to indicate if Charger workaround is active. */
int chg_wa_is_active;
/* Flag used to indicate if Charger workaround timer is on. */
int chg_wa_timer;
int disable_chg_timer;
static unsigned int g_low_batt_flag=0;
static int g_dc_charger_connect=0;
struct workqueue_struct *chg_wq;
struct delayed_work chg_work;
static unsigned long expire;
static int state=CHG_RESTART;
static int recent_voltage_uV[5];
static int recent_index;
static int g_battery_full_flag=0;

struct mc13892_dev_info *g_ntx_bat_di;

extern int ntx_charge_status (void);
extern int ntx_get_battery_vol (void);

#if 0
static int pmic_set_chg_current(unsigned short curr)
{
	unsigned int mask;
	unsigned int value;

printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
	value = BITFVAL(BIT_CHG_CURR, curr);
	mask = BITFMASK(BIT_CHG_CURR);
	CHECK_ERROR(pmic_write_reg(REG_CHARGE, value, mask));

	return 0;
}

static int pmic_set_chg_misc(enum chg_setting type, unsigned short flag)
{

	unsigned int reg_value = 0;
	unsigned int mask = 0;
printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);

	switch (type) {
	case TRICKLE_CHG_EN:
		reg_value = BITFVAL(TRICKLE_CHG_EN, flag);
		mask = BITFMASK(TRICKLE_CHG_EN);
		break;
	case LOW_POWER_BOOT_ACK:
		reg_value = BITFVAL(LOW_POWER_BOOT_ACK, flag);
		mask = BITFMASK(LOW_POWER_BOOT_ACK);
		break;
	case BAT_TH_CHECK_DIS:
		reg_value = BITFVAL(BAT_TH_CHECK_DIS, flag);
		mask = BITFMASK(BAT_TH_CHECK_DIS);
		break;
	case BATTFET_CTL_EN:
		reg_value = BITFVAL(BATTFET_CTL_EN, flag);
		mask = BITFMASK(BATTFET_CTL_EN);
		break;
	case BATTFET_CTL:
		reg_value = BITFVAL(BATTFET_CTL, flag);
		mask = BITFMASK(BATTFET_CTL);
		break;
	case REV_MOD_EN:
		reg_value = BITFVAL(REV_MOD_EN, flag);
		mask = BITFMASK(REV_MOD_EN);
		break;
	case PLIM_DIS:
		reg_value = BITFVAL(PLIM_DIS, flag);
		mask = BITFMASK(PLIM_DIS);
		break;
	case CHG_LED_EN:
		reg_value = BITFVAL(CHG_LED_EN, flag);
		mask = BITFMASK(CHG_LED_EN);
		break;
	case CHGTMRRST:
		reg_value = BITFVAL(CHGTMRRST, flag);
		mask = BITFMASK(CHGTMRRST);
		break;
	case RESTART_CHG_STAT:
		reg_value = BITFVAL(RESTART_CHG_STAT, flag);
		mask = BITFMASK(RESTART_CHG_STAT);
		break;
	case AUTO_CHG_DIS:
		reg_value = BITFVAL(AUTO_CHG_DIS, flag);
		mask = BITFMASK(AUTO_CHG_DIS);
		break;
	case CYCLING_DIS:
		reg_value = BITFVAL(CYCLING_DIS, flag);
		mask = BITFMASK(CYCLING_DIS);
		break;
	case VI_PROGRAM_EN:
		reg_value = BITFVAL(VI_PROGRAM_EN, flag);
		mask = BITFMASK(VI_PROGRAM_EN);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_CHARGE, reg_value, mask));

	return 0;
}
#endif

static int pmic_get_batt_voltage(unsigned short *voltage)
{
	t_channel channel;
	unsigned short result[8];

#if 0
	channel = BATTERY_VOLTAGE;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*voltage = result[0];
#else
	*voltage = 3800000;
#endif
	return 0;
}

static int pmic_get_batt_current(signed short *curr)
{
	t_channel channel;
	signed short result[8];
	bool valid_ch[8] = {1,0,1,0,0,1,0,1};
	int i;

#if 0
	channel = BATTERY_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));

	*curr = 0;
	for(i=0;i<8;i++)
		if(valid_ch[i])
			*curr += (result[i]&0x200) ? (0xffc00|result[i]) : result[i];
	*curr /= 4;

#else
	*curr = 0;
#endif

	return 0;
}

#if 0
static int coulomb_counter_calibration;
static unsigned int coulomb_counter_start_time_msecs;

static int pmic_start_coulomb_counter(void)
{
printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		ACC_COULOMB_PER_LSB * ACC_ONEC_VALUE, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_START_COUNTER, ACC_CONTROL_BIT_MASK));
	coulomb_counter_start_time_msecs = jiffies_to_msecs(jiffies);
	pr_debug("coulomb counter start time %u\n",
		coulomb_counter_start_time_msecs);
	return 0;
}

static int pmic_stop_coulomb_counter(void)
{
printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_STOP_COUNTER, ACC_CONTROL_BIT_MASK));
	return 0;
}

static int pmic_calibrate_coulomb_counter(void)
{
	int ret;
	unsigned int value;

printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		0x1, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_CALIBRATION, ACC_CONTROL_BIT_MASK));
	msleep(ACC_CALIBRATION_DURATION_MSECS);

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	pr_debug("calibrate value = %x\n", value);
	coulomb_counter_calibration = (int)((s16)((u16) value));
	pr_debug("coulomb_counter_calibration = %d\n",
		coulomb_counter_calibration);

	return 0;

}

static int pmic_get_charger_coulomb(int *coulomb)
{
	int ret;
	unsigned int value;
	int calibration;
	unsigned int time_diff_msec;

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	pr_debug("counter value = %x\n", value);
	*coulomb = ((s16)((u16)value)) * ACC_COULOMB_PER_LSB;

	if (abs(*coulomb) >= ACC_COULOMB_PER_LSB) {
			/* calibrate */
		time_diff_msec = jiffies_to_msecs(jiffies);
		time_diff_msec =
			(time_diff_msec > coulomb_counter_start_time_msecs) ?
			(time_diff_msec - coulomb_counter_start_time_msecs) :
			(0xffffffff - coulomb_counter_start_time_msecs
			+ time_diff_msec);
		calibration = coulomb_counter_calibration * (int)time_diff_msec
			/ (ACC_ONEC_VALUE * ACC_CALIBRATION_DURATION_MSECS);
		*coulomb -= calibration;
	}
	return 0;
}

static int pmic_restart_charging(void)
{
printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
	pmic_set_chg_misc(BAT_TH_CHECK_DIS, 1);
	pmic_set_chg_misc(AUTO_CHG_DIS, 0);
	pmic_set_chg_misc(VI_PROGRAM_EN, 1);
//	pmic_set_chg_current(ICHRG_400MA);
	pmic_set_chg_misc(RESTART_CHG_STAT, 1);
	pmic_set_chg_misc(PLIM_DIS, 3);
	return 0;
}

static void init_charger_timer(void)
{
//	pmic_set_chg_misc(CHGTMRRST, 1);
	expire = jiffies + ((BAT_CAP_MAH*3600UL*HZ)/CHG_CUR_MA);
}

static bool charger_timeout(void)
{
	return time_after(jiffies, expire);
}

static void reset_charger_timer(void)
{
//	if(!charger_timeout())
//		pmic_set_chg_misc(CHGTMRRST, 1);
}

#endif

struct mc13892_dev_info {
	struct device *dev;

	unsigned short voltage_raw;
	int voltage_uV;
	signed short current_raw;
	int current_uA;
	int battery_status;
	int full_counter;
	int charger_online;
	int charger_voltage_uV;
	int accum_current_uAh;

	struct power_supply bat;
	struct power_supply charger;

	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

#define mc13892_SENSER	25
#define to_mc13892_dev_info(x) container_of((x), struct mc13892_dev_info, \
					      bat);

static enum power_supply_property mc13892_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property mc13892_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

#if 0
static int pmic_get_chg_value(unsigned int *value)
{
	t_channel channel;
	unsigned short result[8], max1 = 0, min1 = 0, max2 = 0, min2 = 0, i;
	unsigned int average = 0, average1 = 0, average2 = 0;

printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
	channel = CHARGE_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));


	for (i = 0; i < 8; i++) {
		if ((result[i] & 0x200) != 0) {
			result[i] = 0x400 - result[i];
			average2 += result[i];
			if ((max2 == 0) || (max2 < result[i]))
				max2 = result[i];
			if ((min2 == 0) || (min2 > result[i]))
				min2 = result[i];
		} else {
			average1 += result[i];
			if ((max1 == 0) || (max1 < result[i]))
				max1 = result[i];
			if ((min1 == 0) || (min1 > result[i]))
				min1 = result[i];
		}
	}

	if (max1 != 0) {
		average1 -= max1;
		if (max2 != 0)
			average2 -= max2;
		else
			average1 -= min1;
	} else
		average2 -= max2 + min2;

	if (average1 >= average2) {
		average = (average1 - average2) / 6;
		*value = average;
	} else {
		average = (average2 - average1) / 6;
		*value = ((~average) + 1) & 0x3FF;
	}

	return 0;
}
#endif

int get_battery_mA(void) /* get charging current float into battery */
{
	int bat_curr, min_bat_curr=0;
	signed short value=0;
	int i=0;

	for(i=0;i<3;i++)
	{
		pmic_get_batt_current(&value);
		bat_curr = (value*3000)/512;
		min_bat_curr=min(bat_curr, min_bat_curr);
		mdelay(10);
	}
	return -min_bat_curr;
}
 
int get_battery_mV(void)
{
	unsigned short value=0;
	pmic_get_batt_voltage(&value);
	return(value*4800/1023);
}

void set_pmic_dc_charger_state(int dccharger)
{
	g_dc_charger_connect=dccharger;
}
EXPORT_SYMBOL(set_pmic_dc_charger_state);

static void chg_thread(struct work_struct *work)
{
	unsigned int value = 0;
	int charger, curr_mA;

#if 0
	pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));
	charger = BITFEXT(value, BIT_CHG_DETS);
#else
	charger = ntx_charge_status ();
#endif
	switch(state)
	{
	case CHG_RESTART:
//		pmic_restart_charging();
//		pmic_set_chg_current(0);
		if(charger){
#if 0
			if(get_battery_mV()>3500){
				init_charger_timer();
				if(g_dc_charger_connect)
				{
					pr_notice("DC charger connected. Enable 560mA charging\n");
					pmic_set_chg_current(ICHRG_560MA);
				}
				else
				{
					pmic_set_chg_current(ICHRG_400MA);
				}
				state = CHG_CHARGING;
			}
			else{
				if(g_dc_charger_connect)
				{
					pr_notice("DC charger connected. Enable 560mA charging\n");
					pmic_set_chg_current(ICHRG_560MA);
				}
				else
					pmic_set_chg_current(ICHRG_400MA);
				msleep(50);
				if(get_battery_mA()>240){ /* if PMIC can provide 400mA */
					init_charger_timer();
					state = CHG_CHARGING;
				}
				else
					state = CHG_POWER_OFF;
			}
#else
			state = CHG_CHARGING;
#endif
		}
		else
			state = CHG_DISCHARGING;
		queue_delayed_work(chg_wq, &chg_work, HZ*1);
		break;

	case CHG_POWER_OFF:
		pr_notice("Battery level < 3.5V!\n");
		pr_notice("After power off, PMIC will charge up battery.\n");
//		pmic_set_chg_current(0x1); /* charge battery @ 80mA during power off */
		orderly_poweroff(1);
		break;

	case CHG_CHARGING:
#if 0
		reset_charger_timer();
		curr_mA=get_battery_mA();
		if(curr_mA < 50)
		{
			g_battery_full_flag=1;
		}
		if(charger_timeout() || curr_mA <10){
			g_battery_full_flag=0;
			pmic_set_chg_current(0);
			state = CHG_DISCHARGING_WITH_CHARGER;
		}
		if(!charger){
			g_battery_full_flag=0;
			pmic_set_chg_current(0);
			state = CHG_DISCHARGING;
		}
#else
		if (charger & 2) {
		}
		else {
			g_battery_full_flag=1;
			state = CHG_DISCHARGING;
		}
#endif
		queue_delayed_work(chg_wq, &chg_work, HZ*5);
		break;

	case CHG_DISCHARGING:
		if(charger)
			state = CHG_RESTART;
		queue_delayed_work(chg_wq, &chg_work, HZ*10);
		break;

	case CHG_DISCHARGING_WITH_CHARGER:
		if(get_battery_mV()<4000)
			state = CHG_RESTART;
		if(!charger)
			state = CHG_DISCHARGING;
		queue_delayed_work(chg_wq, &chg_work, HZ*2);
		break;
	}
}
 
static int mc13892_charger_update_status(struct mc13892_dev_info *di)
{
	int ret;
	unsigned int value;
	int online;

#if 0
	ret = pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));
#else
	ret = 0;
#endif

	if (ret == 0) {
#if 0
		online = BITFEXT(value, BIT_CHG_DETS);
#else
		online = ntx_charge_status ();
#endif
		if (online != di->charger_online) {
			di->charger_online = online;
			dev_info(di->charger.dev, "charger status: %s\n",
				online ? "online" : "offline");
			power_supply_changed(&di->charger);

			cancel_delayed_work(&di->monitor_work);
			queue_delayed_work(di->monitor_wqueue,
				&di->monitor_work, HZ / 10);
//			if (online)
			if (online & 0x01)
			{
//				pmic_start_coulomb_counter();
				chg_wa_timer = 1;
			} else {
	 			chg_wa_timer = 0;
//				pmic_stop_coulomb_counter();
			}
		}
	}

	return ret;
}

static int mc13892_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di =
		container_of((psy), struct mc13892_dev_info, charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->charger_online;
		return 0;
	default:
		break;
	}
	return -EINVAL;
}

static int mc13892_battery_read_status(struct mc13892_dev_info *di)
{
#if 0
	int retval;
	int coulomb;
	retval = pmic_get_batt_voltage(&(di->voltage_raw));
	if (retval == 0)
		di->voltage_uV = di->voltage_raw * BAT_VOLTAGE_UNIT_UV;

	retval = pmic_get_batt_current(&(di->current_raw));
	if (retval == 0)
			di->current_uA = di->current_raw * BAT_CURRENT_UNIT_UA;

	retval = pmic_get_charger_coulomb(&coulomb);
	if (retval == 0)
		di->accum_current_uAh = COULOMB_TO_UAH(coulomb);
	return retval;
#else
	return 0;
#endif
}

static void mc13892_battery_update_status(struct mc13892_dev_info *di)
{
	unsigned int value;
	int retval;
	int old_battery_status = di->battery_status;

	if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN)
		di->full_counter = 0;

	if (di->charger_online & 0x01) {
#if 0
		retval = pmic_read_reg(REG_INT_SENSE0,
					&value, BITFMASK(BIT_CHG_CURRS));

		if (retval == 0) {
			value = BITFEXT(value, BIT_CHG_CURRS);
			if (value)
				di->battery_status =
					POWER_SUPPLY_STATUS_CHARGING;
			else
				di->battery_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
#else
		if (di->charger_online & 2)
			di->battery_status =
				POWER_SUPPLY_STATUS_CHARGING;
		else
			di->battery_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
#endif
		if (di->battery_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
			di->full_counter++;
		else
			di->full_counter = 0;
	} else {
		di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
		di->full_counter = 0;
	}

	dev_dbg(di->bat.dev, "bat status: %d\n",
		di->battery_status);

	if (old_battery_status != POWER_SUPPLY_STATUS_UNKNOWN &&
		di->battery_status != old_battery_status)
		power_supply_changed(&di->bat);
}

static void mc13892_battery_work(struct work_struct *work)
{
	struct mc13892_dev_info *di = container_of(work,
						     struct mc13892_dev_info,
						     monitor_work.work);
	const int interval = HZ * 60;

	dev_dbg(di->dev, "%s\n", __func__);

	mc13892_battery_update_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static void charger_online_event_callback(void *para)
{
	struct mc13892_dev_info *di = (struct mc13892_dev_info *) para;
printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
	pr_info("\n\n DETECTED charger plug/unplug event\n");
	mc13892_charger_update_status(di);
}

static void low_battery_low_event_callback (void *para)
{
	struct mc13892_dev_info *di = (struct mc13892_dev_info *) para;
	
	dev_info(di->bat.dev, "Low battery low\n");
	g_low_batt_flag=1;
}

static void low_battery_high_event_callback (void *para)
{
	struct mc13892_dev_info *di = (struct mc13892_dev_info *) para;
	unsigned int value;

	pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_LOBATHS));

	if( value )
	{
		dev_info(di->bat.dev, "Battery above 3.4V\n");
		g_low_batt_flag=0;
	}
	else
	{
		dev_info(di->bat.dev, "Low battery high\n");
		g_low_batt_flag=1;
	}
}

static int battery_get_capacity(int now_voltage_uV)
{
	int ratio,i;
	int over_voltage_uV;
	int avg_voltage_uV;
	int sum_voltage_uV = 0;

	if(g_low_batt_flag)
	{
		//When LOBATH/LOBATL triggered, we should treat capacity as 0
		return 0;
	}

	if(recent_index == -1)
	{
		recent_index = 0;
		for(i=0;i<5;i++)
			recent_voltage_uV[i] = now_voltage_uV;
	}

	recent_voltage_uV[recent_index] = now_voltage_uV;
	if(recent_index == 4)
		recent_index = 0;
	else
		recent_index++;

	for(i=0;i<5;i++)
	{
		sum_voltage_uV += recent_voltage_uV[i];
	}

	avg_voltage_uV = sum_voltage_uV / 5;

	if(now_voltage_uV <= 3400000)
		return 0;
	if(now_voltage_uV >= 4100000)
		return 100;
	over_voltage_uV = 4100000 - avg_voltage_uV;
	ratio = (int)over_voltage_uV/8000;
	return 100-ratio;

}

void ntx_charger_online_event_callback(void)
{
	mc13892_charger_update_status(g_ntx_bat_di);
//	mc13892_battery_update_status(g_ntx_bat_di);
}
EXPORT_SYMBOL(ntx_charger_online_event_callback);


static int mc13892_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di = to_mc13892_dev_info(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN) {
			mc13892_charger_update_status(di);
			mc13892_battery_update_status(di);
		}
		val->intval = di->battery_status;
		return 0;
	default:
		break;
	}

	mc13892_battery_read_status(di);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#if 0
		val->intval = di->voltage_uV;
#else
		val->intval = ntx_get_battery_vol ();
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if(g_battery_full_flag)
		{
			/* Hardcode a current value to cheat upper layer charge is full */
			val->intval = 50000;
		}
		else
#if 0
		val->intval = di->current_uA;
#else
		val->intval = 500000;
#endif
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
#if 0
		val->intval = di->accum_current_uAh;
#else
		val->intval = 500000;
#endif
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 3800000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3300000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#if 1
		val->intval = ntx_get_battery_vol ();
#else
		val->intval = battery_get_capacity(di->voltage_uV);
#endif
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t chg_wa_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
	if (chg_wa_is_active & chg_wa_timer)
		return sprintf(buf, "Charger LED workaround timer is on\n");
	else
		return sprintf(buf, "Charger LED workaround timer is off\n");
}

static ssize_t chg_wa_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
	if (strstr(buf, "1") != NULL) {
		if (chg_wa_is_active) {
			if (chg_wa_timer)
				printk(KERN_INFO "Charger timer is already on\n");
			else {
				chg_wa_timer = 1;
				printk(KERN_INFO "Turned on the timer\n");
			}
		}
	} else if (strstr(buf, "0") != NULL) {
		if (chg_wa_is_active) {
			if (chg_wa_timer) {
				chg_wa_timer = 0;
				printk(KERN_INFO "Turned off charger timer\n");
			 } else {
				printk(KERN_INFO "The Charger workaround timer is off\n");
			}
		}
	}

	return size;
}

static DEVICE_ATTR(enable, 0644, chg_wa_enable_show, chg_wa_enable_store);

static int pmic_battery_remove(struct platform_device *pdev)
{
	pmic_event_callback_t bat_event_callback;
	struct mc13892_dev_info *di = platform_get_drvdata(pdev);

printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
#if 0
	bat_event_callback.func = charger_online_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_unsubscribe(EVENT_CHGDETI, bat_event_callback);

	bat_event_callback.func = low_battery_low_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_unsubscribe(EVENT_LOBATLI, bat_event_callback);

	bat_event_callback.func = low_battery_high_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_unsubscribe(EVENT_LOBATHI, bat_event_callback);
#endif

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	cancel_rearming_delayed_workqueue(chg_wq,
					  &chg_work);
	destroy_workqueue(di->monitor_wqueue);
	destroy_workqueue(chg_wq);
	chg_wa_timer = 0;
	chg_wa_is_active = 0;
	disable_chg_timer = 0;
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->charger);

	kfree(di);

	return 0;
}

static int pmic_battery_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct mc13892_dev_info *di;
	pmic_event_callback_t bat_event_callback;
	pmic_version_t pmic_version;

printk ("[%s-%d] %s...\n",__FILE__,__LINE__,__func__);
#if 0
	/* Only apply battery driver for MC13892 V2.0 due to ENGR108085 */
	pmic_version = pmic_get_version();
	if (pmic_version.revision < 20) {
		pr_debug("Battery driver is only applied for MC13892 V2.0\n");
		return -1;
	}
	if (machine_is_mx50_arm2()) {
		pr_debug("mc13892 charger is not used for this platform\n");
		return -1;
	}
#endif
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	platform_set_drvdata(pdev, di);

	di->charger.name	= "mc13892_charger";
	di->charger.type = POWER_SUPPLY_TYPE_MAINS;
	di->charger.properties = mc13892_charger_props;
	di->charger.num_properties = ARRAY_SIZE(mc13892_charger_props);
	di->charger.get_property = mc13892_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->charger);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto charger_failed;
	}

	INIT_DELAYED_WORK(&chg_work, chg_thread);
	chg_wq = create_singlethread_workqueue("mxc_chg");
	if (!chg_wq) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(chg_wq, &chg_work, HZ);

	INIT_DELAYED_WORK(&di->monitor_work, mc13892_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 10);

	di->dev	= &pdev->dev;
	di->bat.name	= "mc13892_bat";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = mc13892_battery_props;
	di->bat.num_properties = ARRAY_SIZE(mc13892_battery_props);
	di->bat.get_property = mc13892_battery_get_property;
	di->bat.use_for_apm = 1;

	di->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}

	g_ntx_bat_di = di;
#if 0
	bat_event_callback.func = charger_online_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_subscribe(EVENT_CHGDETI, bat_event_callback);

	bat_event_callback.func = low_battery_low_event_callback;
	bat_event_callback.param = (void *) di;
	retval = pmic_event_subscribe(EVENT_LOBATLI, bat_event_callback);
	if(retval) {
		printk(KERN_ERR
		       "Battery: Cannot subscribe pmic event");
	}

	bat_event_callback.func = low_battery_high_event_callback;
	bat_event_callback.param = (void *) di;
	pmic_event_subscribe(EVENT_LOBATHI, bat_event_callback);
	if(retval) {
		printk(KERN_ERR
		       "Battery: Cannot subscribe pmic event");
	}
#endif
	retval = sysfs_create_file(&pdev->dev.kobj, &dev_attr_enable.attr);

	if (retval) {
		printk(KERN_ERR
		       "Battery: Unable to register sysdev entry for Battery");
		goto workqueue_failed;
	}
	chg_wa_is_active = 1;
	chg_wa_timer = 0;
	disable_chg_timer = 0;
	recent_index = -1;

//	pmic_stop_coulomb_counter();
//	pmic_calibrate_coulomb_counter();
	goto success;

workqueue_failed:
	power_supply_unregister(&di->charger);
charger_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	kfree(di);
di_alloc_failed:
success:
	dev_dbg(di->dev, "%s battery probed!\n", __func__);
	return retval;


	return 0;
}

static struct platform_driver pmic_battery_driver_ldm = {
	.driver = {
		   .name = "pmic_battery",
		   .bus = &platform_bus_type,
		   },
	.probe = pmic_battery_probe,
	.remove = pmic_battery_remove,
};

static int __init pmic_battery_init(void)
{
	pr_debug("PMIC Battery driver loading...\n");
	return platform_driver_register(&pmic_battery_driver_ldm);
}

static void __exit pmic_battery_exit(void)
{
	platform_driver_unregister(&pmic_battery_driver_ldm);
	pr_debug("PMIC Battery driver successfully unloaded\n");
}

module_init(pmic_battery_init);
module_exit(pmic_battery_exit);

MODULE_DESCRIPTION("pmic_battery driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
