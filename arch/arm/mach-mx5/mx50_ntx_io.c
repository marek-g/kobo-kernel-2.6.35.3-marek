#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/iomux-mx50.h>
#include <asm/uaccess.h>
#include <asm/system.h>


#include <generated/autoconf.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>
#include <linux/freezer.h>


//#define NTX_GPIO_KEYS		0
#ifdef NTX_GPIO_KEYS //[

#include <mach/common.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>

#endif //] NTX_GPIO_KEYS



//#define GPIOFN_PWRKEY	1

#ifdef GPIOFN_PWRKEY//[
	#include "../../../drivers/input/keyboard/gpiofn.h"
#endif //]GPIOFN_PWRKEY


//#define _WIFI_ALWAYS_ON_	// wifi always on for startic
//#define DIGITIZER_TEST		// PVI digitizer test

#include "ntx_hwconfig.h"

#define GPIO_ACIN_PG	(3*32 + 19)	/*GPIO_4_19 */
#define GPIO_CHG		(3*32 + 18)	/*GPIO_4_18 */
#define GPIO_ACIN_ID	(3*32 + 16)	/*GPIO_4_16 */
#define GPIO_LED_ON		(0*32 + 24)	/*GPIO_1_24 */
#define GPIO_ACT_ON		(5*32 + 24)	/*GPIO_6_24 */
#define GPIO_CHG_LED	(0*32 + 25)	/*GPIO_1_25 */
#define GPIO_PWR_SW		(3*32 + 10)	/*GPIO_4_10 */
#define GPIO_WIFI_3V3	(3*32 + 12)	/*GPIO_4_12 */
#define GPIO_WIFI_RST	(4*32 + 14)	/*GPIO_5_14 */
#define GPIO_WIFI_INT	(3*32 + 8)	/*GPIO_4_8 */
#define GPIO_MSP_INT	(3*32 + 11)	/*GPIO_4_11 */
#define SD2_CD			(4*32 + 17) /*GPIO_5_17 */
//#define SD2_WP			(4*32 + 16)	/*GPIO_5_16 */
#define TOUCH_PWR		(3*32 + 16)	/*GPIO_4_16 */
#define TOUCH_RST		(4*32 + 28)	/*GPIO_5_28 */
#define TOUCH_EN		(4*32 + 26)	/*GPIO_5_26 */
#define TOUCH_INT		(4*32 + 15)	/*GPIO_5_15 */
#define C_TOUCH_INT		(4*32 + 27)	/*GPIO_5_27 */
#define GPIO_I2C3_SDA	(5*32 + 23)	/*GPIO_6_23 */
#define GPIO_I2C3_SCL	(5*32 + 22)	/*GPIO_6_22 */
#define GPIO_AUDIO_PWR	(3*32 + 17)	/*GPIO_4_17 */
#define G_SENSOR_INT	(4*32 + 25)	/*GPIO_5_25 */
#define E50602_G_SENSOR_INT	(3*32 + 15)	/*GPIO_4_15 */
#define FL_EN			(3*32 + 14) /*GPIO_4_14 */
#define FL_R_EN			(3*32 + 22)	/*GPIO_4_22 */

#define GPIO_KEY_COL_0		(3*32 + 0)	/*GPIO_4_0 */
#define GPIO_KEY_ROW_0		(3*32 + 1)	/*GPIO_4_1 */
#define GPIO_KEY_COL_1		(3*32 + 2)	/*GPIO_4_2 */
#define GPIO_KEY_ROW_1		(3*32 + 3)	/*GPIO_4_3 */
#define GPIO_KEY_COL_2		(3*32 + 4)	/*GPIO_4_4 */
#define GPIO_KEY_ROW_2		(3*32 + 5)	/*GPIO_4_5 */
#define GPIO_KEY_COL_3		(3*32 + 6)	/*GPIO_4_6 */
#define GPIO_KEY_ROW_3		(3*32 + 7)	/*GPIO_4_7 */

#define GPIO_HWID_1			(5*32 + 14)	/*GPIO_6_14 */
#define GPIO_HWID_2			(5*32 + 15)	/*GPIO_6_15 */
#define GPIO_HWID_3			(5*32 + 17)	/*GPIO_6_17 */
#define GPIO_HWID_4			(4*32 + 16)	/*GPIO_5_16 */

#define EIM_DA0    (0*32 + 0) /*GPIO_1_0*/
#define EIM_DA1    (0*32 + 1) /*GPIO_1_1*/
#define EIM_DA2    (0*32 + 2) /*GPIO_1_2*/
#define EIM_DA3    (0*32 + 3) /*GPIO_1_3*/
#define EIM_DA4    (0*32 + 4) /*GPIO_1_4*/
#define EIM_DA5    (0*32 + 5) /*GPIO_1_5*/
#define EIM_DA6    (0*32 + 6) /*GPIO_1_6*/
#define EIM_DA7    (0*32 + 7) /*GPIO_1_7*/
#define EIM_DA8    (0*32 + 8) /*GPIO_1_8*/
#define EIM_DA9    (0*32 + 9) /*GPIO_1_9*/
#define EIM_DA10   (0*32 + 10) /*GPIO_1_10*/
#define EIM_DA11   (0*32 + 11) /*GPIO_1_11*/
#define EIM_DA12   (0*32 + 12) /*GPIO_1_12*/
#define EIM_DA13   (0*32 + 13) /*GPIO_1_13*/
#define EIM_DA14   (0*32 + 14) /*GPIO_1_14*/
#define EIM_DA15   (0*32 + 15) /*GPIO_1_15*/
#define EIM_CS2    (0*32 + 16) /*GPIO_1_16*/
#define EIM_CS1    (0*32 + 17) /*GPIO_1_17*/
#define EIM_CS0    (0*32 + 18) /*GPIO_1_18*/
#define EIM_EB0    (0*32 + 19) /*GPIO_1_19*/
#define EIM_EB1    (0*32 + 20) /*GPIO_1_20*/
#define EIM_WAIT   (0*32 + 21) /*GPIO_1_21*/
#define EIM_BCLK   (0*32 + 22) /*GPIO_1_22*/
#define EIM_RDY    (0*32 + 23) /*GPIO_1_23*/



#define		DEVICE_NAME 		"ntx_io"		// "pvi_io"
#define		DEVICE_MINJOR		190

#define  CM_PLATFORM		164
#define  CM_HWCONFIG		165
#define  CM_SET_HWCONFIG	166

#define	CM_SD_IN				117
#define	AC_IN					118
#define CM_PWR_ON2				112
#define CM_AUDIO_PWR			113
#define	CM_POWER_BTN			110
#define CM_USB_Plug_IN			108
#define CM_AC_CK				109
#define CM_CHARGE_STATUS		204
#define	CM_nLED					101
#define	CM_nLED_CPU				102
#define	POWER_OFF_COMMAND		0xC0	// 192
#define	SYS_RESET_COMMAND 		193		// Joseph 091223
#define	GET_LnBATT_CPU			0XC2		// 194
#define	GET_VBATT_TH			0XC3	// 195
#define	CM_SIGUSR1				104
//kay 20081110 for detecting SD write protect
#define	CM_SD_PROTECT			120
#define	SYS_AUTO_POWER_ON 		0xC4	// 196 Joseph 120620
                            	
//20090216 for detecting controller
#define	CM_CONTROLLER			121

//20090416 for detecting controller
#define	CM_USB_AC_STATUS		122
#define	CM_RTC_WAKEUP_FLAG		123
#define	CM_SYSTEM_RESET			124
#define	CM_USB_HOST_PWR			125
#define	CM_BLUETOOTH_PWR		126
#define	CM_TELLPID				99	
#define CM_LED_BLINK 			127
#define CM_TOUCH_LOCK 			128
#define CM_DEVICE_MODULE 		129
#define CM_BLUETOOTH_RESET 		130
#define CM_DEVICE_INFO 			131

//Joseph 091211 for 3G
#define CM_3G_POWER 			150
#define CM_3G_RF_OFF 			151
#define CM_3G_RESET 			152
#define CM_3G_GET_WAKE_STATUS	153

//Joseph 091209
#define	CM_ROTARY_STATUS 		200	
#define	CM_GET_KEY_STATUS 		201	
#define	CM_GET_WHEEL_KEY_STATUS 202	
#define	POWER_KEEP_COMMAND 		205	
#define	CM_GET_BATTERY_STATUS 	206	
#define	CM_SET_ALARM_WAKEUP	 	207	
#define	CM_WIFI_CTRL	 		208	
#define	CM_ROTARY_ENABLE 		209	

#define CM_GET_UP_VERSION 		215

// gallen 100621
// Audio functions ...
#define CM_AUDIO_GET_VOLUME		230
#define CM_AUDIO_SET_VOLUME		240
#define CM_FRONT_LIGHT_SET		241
#define CM_FRONT_LIGHT_AVAILABLE	242
#define CM_FRONT_LIGHT_DUTY		243
#define CM_FRONT_LIGHT_FREQUENCY	244

#define CM_GET_KEYS				107


#ifdef GPIOFN_PWRKEY//[
static void power_key_chk(unsigned long v);

static int PWR_SW_func(int iGPIOVal)
{
	printk("[%s]\n",__FUNCTION__);
	power_key_chk(0);
}

static GPIODATA gtNTX_PWR_GPIO_data = {
	.pfnGPIO = PWR_SW_func,
	.uGPIO = GPIO_PWR_SW,
	.szName = "PWR_SW",
	.tPADCtrl = MX50_PAD_CSPI_MISO__GPIO_4_10,
	.uiIRQType = IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
	.iWakeup = 1,
};



#endif //]GPIOFN_PWRKEY


unsigned short  __EBRMAIN_PID__ = 0;

unsigned char __USB_ADAPTOR__=0;
EXPORT_SYMBOL(__USB_ADAPTOR__);

static int Driver_Count = -1;
unsigned char __TOUCH_LOCK__= 0;
int gSleep_Mode_Suspend;

extern volatile NTX_HWCONFIG *gptHWCFG;

typedef enum __DEV_MODULE_NAME{
    EB500=0,
    EB600=1,
    EB600E=2,
    EB600EM=3,
    COOKIE=4,    
}__dev_module_name;

typedef enum __DEV_MODULE_CPU{
    CPU_S3C2410=0,
    CPU_S3C2440=1,
    CPU_S3C2416=2,
    CPU_CORETEX_A8=3,
    CPU_COOKIE=4,    
}__dev_module_cpu;

typedef enum __DEV_MODULE_CONTROLLER{
    CONTROLLER_PVI=0,
    CONTROLLER_EPSON=1,
    CONTROLLER_SW=2,
}__dev_module_controller;

typedef enum __DEV_MODULE_WIFI{
    WIFI_NONE=0,
    WIFI_MARVELL=1,
    WIFI_OTHER=2,
}__dev_module_wifi;

typedef enum __DEV_MODULE_BLUETOOTH{
    BLUETOOTH_NONE=0,
    BLUETOOTH_TI=1,
    BLUETOOTH_CSR=2,
}__devi_module_bluetooth;

struct ebook_device_info {
    char device_name;
    char cpu;
    char controller;
    char wifi;
    char bluetooth;
};
/* original table , new one = this /164
static unsigned int FL_table0[100]={
0x0100,0x0400,0x0500,0x0600,0x0800,0x0880,0x0900,0x0A00,0x0B00,0x0C00,
0x0D00,0x0E00,0x0F00,0x1000,0x1100,0x11D0,0x1280,0x1300,0x1400,0x1500,
0x1600,0x1700,0x1800,0x1900,0x1A00,0x1B00,0x1C00,0x1D00,0x1E00,0x1F00,
0x2000,0x2100,0x2200,0x2300,0x2400,0x2500,0x2600,0x2700,0x2800,0x2900,
0x2A00,0x2B00,0x2C00,0x2D00,0x2E00,0x2F00,0x3000,0x3400,0x3800,0x3C00,
0x4000,0x4400,0x4800,0x4C00,0x5000,0x5400,0x5800,0x5C00,0x6000,0x6400,
0x6800,0x6C00,0x7000,0x7400,0x76C0,0x7840,0x7C00,0x8000,0x8400,0x8800,
0x8C00,0x9000,0x9400,0x9800,0x9C00,0xA000,0xA400,0xA800,0xAC00,0xB000,
0xB400,0xB800,0xBC00,0xC000,0xC400,0xC800,0xCC00,0xD000,0xD400,0xD800,
0xDC00,0xE000,0xE400,0xE800,0xEC00,0xF000,0xF400,0xF800,0xFC00,0xFFFF
};
*/	
static unsigned short FL_table0[100]={
0x0001,0x0006,0x0007,0x0009,0x000C,0x000D,0x000E,0x000F,0x0011,0x0012,
0x0014,0x0015,0x0017,0x0018,0x001A,0x001B,0x001C,0x001D,0x001F,0x0020,
0x0022,0x0023,0x0025,0x0027,0x0028,0x002A,0x002B,0x002D,0x002E,0x0030,
0x0031,0x0033,0x0035,0x0036,0x0038,0x0039,0x003B,0x003C,0x003E,0x0040,
0x0041,0x0043,0x0044,0x0046,0x0047,0x0049,0x004A,0x0051,0x0057,0x005D,
0x0063,0x006A,0x0070,0x0076,0x007C,0x0083,0x0089,0x008F,0x0095,0x009C,
0x00A2,0x00A8,0x00AE,0x00B5,0x00B9,0x00BB,0x00C1,0x00C7,0x00CE,0x00D4,
0x00DA,0x00E0,0x00E7,0x00ED,0x00F3,0x00F9,0x0100,0x0106,0x010C,0x0112,
0x0118,0x011F,0x0125,0x012B,0x0131,0x0138,0x013E,0x0144,0x014A,0x0151,
0x0157,0x015D,0x0163,0x016A,0x0170,0x0176,0x017C,0x0183,0x0189,0x018F
};
	
//kay for LED thread
//static unsigned char LED_conitnuous=0;
static unsigned char LED_conitnuous=1;
static int LED_Flash_Count;
static int gKeepPowerAlive;
int gMxcPowerKeyIrqTriggered, gIsMSP430IntTriggered, g_power_key_pressed;
volatile int g_mxc_touch_triggered = 1;	//gallen 100420
int g_wakeup_by_alarm;
int gWifiEnabled=0;
static unsigned long g_usb_in_tick;	// Joseph 101001
static int g_ioctl_SD_status, g_ioctl_USB_status, g_ioctl_rotary_status,g_Cus_Ctrl_Led;
int g_mmc_card_detect_changed;	// Joseph 20110125
static DEFINE_SPINLOCK(led_flash_lock);
static DECLARE_WAIT_QUEUE_HEAD(LED_blink_WaitQueue);
static DECLARE_WAIT_QUEUE_HEAD(LED_freeze_WaitQueue);
static DECLARE_WAIT_QUEUE_HEAD(WheelKey_WaitQueue);
////////////////////

static DECLARE_WAIT_QUEUE_HEAD(Reset_WaitQueue);

extern int gIsCustomerUi;

extern void gpio_sdhc_inactive(int module);
extern void gpio_uart_inactive(int port, int no_irda);
extern void mxc_mmc_force_detect(int id);
extern void tle4913_init(void);

int ntx_charge_status (void);

//kay 20090925
//check WiFi ID
static int check_hardware_wifi(void)
{
    return  WIFI_NONE;           
}

//check Bluetooth ID
static int check_hardware_bt(void)
{
    return  BLUETOOTH_NONE;           
}

static int check_hardware_cpu(void)
{
    return CPU_S3C2440;
}

//static int check_hardeare_name(void)
int check_hardware_name(void)
{
	static int pcb_id = -1;

	if (0 >= pcb_id) {
#ifdef DIGITIZER_TEST
		pcb_id = 3;		// E60682 board
		mxc_iomux_v3_setup_pad(MX50_PAD_UART3_TXD__UART3_TXD);
		mxc_iomux_v3_setup_pad(MX50_PAD_UART3_RXD__UART3_RXD);
#else
		mxc_iomux_v3_setup_pad(MX50_PAD_UART3_TXD__GPIO_6_14);
		mxc_iomux_v3_setup_pad(MX50_PAD_UART3_RXD__GPIO_6_15);
		mxc_iomux_v3_setup_pad(MX50_PAD_UART4_RXD__GPIO_6_17);
		mxc_iomux_v3_setup_pad(MX50_PAD_SD2_WP_HWID0__GPIO_5_16);

		gpio_request(GPIO_HWID_1, "hwid_1");
		gpio_direction_input (GPIO_HWID_1);
		gpio_request(GPIO_HWID_2, "hwid_2");
		gpio_direction_input (GPIO_HWID_2);
		gpio_request(GPIO_HWID_3, "hwid_3");
		gpio_direction_input (GPIO_HWID_3);
		gpio_request(GPIO_HWID_4, "hwid_4");
		gpio_direction_input (GPIO_HWID_4);
#if 0	//READ HW ID		
		/*
		 *  Note:
		 *  Comparing to the schematic diagram
		 *  the lower 3 bits are in reversed order
		 *  the higher (4th) bit is inverted (1 <-> 0)
		 * 	 ** Noted by William Chen
		*/
		
		pcb_id = (gpio_get_value (GPIO_HWID_1)?1:0);
		pcb_id |= (gpio_get_value (GPIO_HWID_2)?2:0);
		pcb_id |= (gpio_get_value (GPIO_HWID_3)?4:0);
		pcb_id |= (gpio_get_value (GPIO_HWID_4)?0:8);  
		if (7 == pcb_id) 
			pcb_id = 4;
#else   //READ HWCONFIG
		switch(gptHWCFG->m_val.bPCB)
		{
			case 12: //E60610
			case 20: //E60610C
			case 21: //E60610D
				pcb_id = 1;
				break;
			case 15: //E60620
				pcb_id = 4;
				break;
			case 16: //E60630
				pcb_id = 6;
				break;
			case 18: //E50600
				pcb_id = 2;
				break;
			case 19: //E60680
				pcb_id = 3;
				break;
			case 22: //E606A0
				pcb_id = 10;
				break;
			case 23: //E60670
				pcb_id = 5;
				break;
			case 24: //E606B0
				pcb_id = 14;
				break;
			case 27: //E50610
				pcb_id = 9;
				break;
			default:
				printk ("[%s-%d] undefined PCBA ID\n",__func__,__LINE__);
				break;	
		}
#endif
#endif			
		printk ("[%s-%d] PCBA ID is %d\n",__func__,__LINE__,pcb_id);
	}

    return pcb_id;      
}
EXPORT_SYMBOL(check_hardware_name);

static int check_hardware_controller(void)
{
    return CONTROLLER_EPSON;
}

static void collect_hardware_info(struct ebook_device_info *info)
{
    info->cpu = check_hardware_cpu();
    info->device_name = check_hardware_name();
    info->controller = check_hardware_controller();
    info->wifi = check_hardware_wifi();
    info->bluetooth = check_hardware_bt();
}

static int openDriver(struct inode *inode,struct file *filp)
{
	if(!Driver_Count)
		Driver_Count++;
	return 0;
}
static int releaseDriver(struct inode *inode,struct file *filp)
{
	if(Driver_Count)
		Driver_Count--;
	return 0;
}
static void bluetooth_reset(int i)
{
}

static void bluetooth_pwr(int i)
{
}


#define SD2_CLK		(4*32 + 6)	/*GPIO_5_6 */
static void wifi_sdio_enable (int isEnable)
{
	extern iomux_v3_cfg_t mx50_sd3_disable_pads[];
	extern unsigned long gdw_mx50_sd3_disable_pads;

	extern iomux_v3_cfg_t mx50_sd2_disable_pads[];
	extern unsigned long gdw_mx50_sd2_disable_pads;


	if(9==check_hardware_name()) {
		// E50612 .
		//printk("E50612=> %s(%d)\n",__FUNCTION__,isEnable);
		if (isEnable) {
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CLK__SD2_CLK_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CMD__SD2_CMD_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D0__SD2_D0_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D1__SD2_D1_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D2__SD2_D2_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D3__SD2_D3_WIFI);
		}
		else {
			mxc_iomux_v3_setup_multiple_pads(mx50_sd2_disable_pads, gdw_mx50_sd2_disable_pads);
		}

	}
	else {
		if (isEnable) {
			mxc_iomux_v3_setup_pad(MX50_PAD_SD3_CLK__SD3_CLK_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD3_CMD__SD3_CMD_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D0__SD3_D0_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D1__SD3_D1_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D2__SD3_D2_WIFI);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D3__SD3_D3_WIFI);
		}
		else {
			mxc_iomux_v3_setup_multiple_pads(mx50_sd3_disable_pads, gdw_mx50_sd3_disable_pads);
		}
	}
}


void ntx_wifi_power_ctrl (int isWifiEnable)
{
	gWifiEnabled=isWifiEnable;
	printk("Wifi / BT power control %d\n", isWifiEnable);
    if(isWifiEnable == 0){
		gpio_direction_input(GPIO_WIFI_3V3);	// turn off Wifi_3V3_on
		gpio_set_value(GPIO_WIFI_RST, 0);		// turn on wifi_RST
		wifi_sdio_enable (0);
#ifdef _WIFI_ALWAYS_ON_
		disable_irq_wake(gpio_to_irq(GPIO_WIFI_INT));
#endif
	}else{
		gpio_direction_output(GPIO_WIFI_3V3, 0);	// turn on Wifi_3V3_on
    	sleep_on_timeout(&Reset_WaitQueue,HZ/50);			
		gpio_set_value(GPIO_WIFI_RST, 1);		// turn on wifi_RST
		wifi_sdio_enable (1);
#ifdef _WIFI_ALWAYS_ON_
		enable_irq_wake(gpio_to_irq(GPIO_WIFI_INT));
#endif
	}
	sleep_on_timeout(&Reset_WaitQueue,HZ/10);			
	if(9==check_hardware_name()) {
		// E50612 .
		mxc_mmc_force_detect (1);
	}
	else {
		mxc_mmc_force_detect (2);
	}

	schedule_timeout (500);
}
EXPORT_SYMBOL(ntx_wifi_power_ctrl);

extern u16 msp430_deviceid(void);
extern void msp430_auto_power(int minutes);
extern void msp430_poweroff(void);
extern void msp430_reset(void);
extern void msp430_powerkeep(int n);
extern int msp430_battery(void);
int msp430_check_wakeup(void);
int msp430_setwatchdog(int n);

extern int msp430_write(unsigned int reg, unsigned int value);
extern unsigned int msp430_read(unsigned int reg);
extern int mma7660_read_orient (void);
int gTSC2004_exist;	// Joseph 20100723
unsigned long gLastBatTick, gUSB_Change_Tick;
int gLastBatValue;
int g_power_key_debounce;		// Joseph 20100921 for ESD


unsigned long long hwconfig = 0x0000000011000001LL;
EXPORT_SYMBOL(hwconfig);
unsigned char platform_type[32];
EXPORT_SYMBOL(platform_type);

static int __init early_hw(char *p)
{
	hwconfig = simple_strtoull(p, NULL, 16);
	printk("hwconfig: %16llX\n", hwconfig);
	return 0;
}
early_param("hwconfig", early_hw);

//to parse hardware configuration bits
static int __init early_board(char *p)
{
	strncpy(platform_type, p, sizeof(platform_type));
	printk("board: %s\n", platform_type);
	return 0;
}
early_param("board", early_board);

static int  ioctlDriver(struct inode *inode, struct file *filp, unsigned int command, unsigned long arg)
{
	unsigned long i = 0, temp;
	unsigned int p = arg;//*(unsigned int *)arg;
	static unsigned int  last_FL_duty = 0;
	static unsigned int  current_FL_freq = 0xFFFF;
  struct ebook_device_info info;  
  	
	if(!Driver_Count){
		printk("pvi_io : do not open\n");
		return -1;	
	}

	switch(command)
	{
		case POWER_OFF_COMMAND:
			if (!gKeepPowerAlive) {
				LED_conitnuous = 0;
		       	gpio_set_value (GPIO_LED_ON,1);
		       	while (1) {
					printk("Kernel---Power Down ---\n");
					msp430_poweroff();
			      	sleep_on_timeout(&Reset_WaitQueue, 14*HZ/10);
				}
			}
			else {
				printk("Kernel---in keep alive mode ---\n");
			}
			break;
		case SYS_RESET_COMMAND:
		    while (1) {
				printk("Kernel---System reset ---\n");
				gKeepPowerAlive = 0;
				msp430_reset();
			    sleep_on_timeout(&Reset_WaitQueue, 14*HZ/10);
			}
			break;
			
		case SYS_AUTO_POWER_ON:
			msp430_auto_power(p);
		    while (1) {
				printk("Kernel---System reset ---\n");
				gKeepPowerAlive = 0;
				msp430_poweroff();
			    sleep_on_timeout(&Reset_WaitQueue, 14*HZ/10);
			}
			break;
			
		case POWER_KEEP_COMMAND:
			printk("Kernel---System Keep Alive --- %d\n",p);
			gKeepPowerAlive=p;
			if (gKeepPowerAlive) {
				msp430_powerkeep(1);
		   		wake_up_interruptible(&LED_freeze_WaitQueue);
			}
			else
				msp430_powerkeep(0);
			break;
			
		case CM_GET_BATTERY_STATUS:
			if (gUSB_Change_Tick) {
				if (500 < (jiffies - gUSB_Change_Tick)) {
					gUSB_Change_Tick = 0;
					gLastBatValue = 0;
				}
			}
			
			if ((6 == check_hardware_name()) || (2 == check_hardware_name())) {		// E60632 || E50602
				i = 1023;
				copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
	
				break;
			}
	
			if (gIsMSP430IntTriggered || !gLastBatValue || ((0==gUSB_Change_Tick) && (200 < (jiffies - gLastBatTick)))) {
				i = msp430_battery ();
	 			if (i) {
					gLastBatTick = jiffies;
					temp = msp430_read (0x60);
					if (0x8000 & temp) {
						printk ("[%s-%d] =================> Micro P MSP430 alarm triggered <===================\n", __func__, __LINE__);
						g_wakeup_by_alarm = 1;
					}
					if ((0x01 & temp) || (0x8000 & i)) {
						printk ("[%s-%d] =================> Micro P MSP430 Critical_Battery_Low <===================\n", __func__, __LINE__);
						i |= 0x8000;
						gIsMSP430IntTriggered = 1;
					}
					else {
//						printk ("[%s-%d] battery ===> %d\n", __func__, __LINE__,i);
						if (!gLastBatValue)
							gLastBatValue = i;
						if (gpio_get_value (GPIO_ACIN_PG)) {// not charging
							if (gLastBatValue > i) 
								gLastBatValue = i;
							else
								i = gLastBatValue;
						}
						else {
							if (gLastBatValue < i)
								gLastBatValue = i;
							else
								i = gLastBatValue;
						}
					}
				}
				else {
					printk ("[%s-%d] MSP430 read failed\n", __func__, __LINE__);
					i = 0x7FFF;
				}
			}
			else
				i = gLastBatValue;
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));

			break;
			
		case AC_IN:
			i = gpio_get_value (GPIO_ACIN_PG)?0:1;
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;	

		case CM_SD_IN:
			g_ioctl_SD_status = gpio_get_value (SD2_CD);
			i = (g_ioctl_SD_status)?0:1;
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;
			
		case CM_USB_Plug_IN:
			g_ioctl_USB_status = gpio_get_value (GPIO_ACIN_PG);
#if 0
			if (!g_ioctl_USB_status && g_usb_in_tick && (50 < (jiffies - g_usb_in_tick)))
				g_ioctl_USB_status = 0;
			else
				g_ioctl_USB_status = 1;
#endif			
			i = (g_ioctl_USB_status)?0:1;
			if (!g_Cus_Ctrl_Led) 
				gpio_set_value (GPIO_CHG_LED,g_ioctl_USB_status);
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;
			
		case GET_LnBATT_CPU:
			break;
		case GET_VBATT_TH:
			break;
		case CM_AC_CK:
			break;
		case CM_CHARGE_STATUS:
			i = ntx_charge_status();
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;
		case CM_PWR_ON2:
			break;
		case CM_AUDIO_PWR:
	        if (p) {
	        	// Turn on audio power
        		gpio_direction_output(GPIO_AUDIO_PWR, 0);
				mxc_iomux_v3_setup_pad(MX50_PAD_PWM2__PWMO);
//	        	gpio_activate_audio_ports(); // 會造成IO REQUEST ERROR !?
	        }
	        else {	// turn off audio power
				mxc_iomux_v3_setup_pad(MX50_PAD_PWM2__GPIO_6_25);
        		gpio_direction_input(GPIO_AUDIO_PWR);
//	        	gpio_inactivate_audio_ports(); // 會造成IO REQUEST ERROR !?
	        }
			break;
		case CM_nLED:
			//printk("CM_nLED %d\n",p);
			switch (check_hardware_name())
			{
				case 4:  //E60622
				case 2:	 //E50602
				case 6:  //E60632
					gpio_set_value (GPIO_ACT_ON,p);
	        	break;
				default:	
					gpio_set_value (GPIO_LED_ON,p);
				break;
			}
			break;			
			
		case CM_nLED_CPU:
			break;			
			
		case CM_SD_PROTECT:
#if 1
			i = 0;
#else
			i = (gpio_get_value (SD2_WP))?1:0;
#endif
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;
			
		case CM_CONTROLLER:
            i = 2;	// 2: Epson controller. for micro window
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;
			
		case CM_USB_AC_STATUS:
			i = 0;
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;
			
		case CM_RTC_WAKEUP_FLAG:
			if (!g_wakeup_by_alarm) {
				int tmp = msp430_read (0x60);
				if (0x8000 & tmp) {
					printk ("[%s-%d] =================> Micro P MSP430 alarm triggered <===================\n", __func__, __LINE__);
					g_wakeup_by_alarm = 1;
				}
			}
            i = g_wakeup_by_alarm;		// Joseph 091221 for slide show test.
			if (g_wakeup_by_alarm) {
				gIsMSP430IntTriggered = 0;
	            g_wakeup_by_alarm = 0;
			}
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;
			
		case CM_SYSTEM_RESET:
			printk("Kernel---System reset ---\n");
			gKeepPowerAlive = 0;
			msp430_reset();
			break;
			
		case CM_USB_HOST_PWR:
			break;
			
		case CM_BLUETOOTH_PWR:
			ntx_wifi_power_ctrl (p);
			break;
			
        case CM_TELLPID:
			if(p!=0){
			    printk("PID %d\n",p);
			    __EBRMAIN_PID__= p;
			}
			break;
			
		case CM_LED_BLINK:
		    if (2==p) {
				spin_lock(&led_flash_lock);
		    	LED_Flash_Count++;
				spin_unlock(&led_flash_lock);
		   	}
		   	if (!LED_conitnuous)
		   		wake_up_interruptible(&LED_freeze_WaitQueue);
			LED_conitnuous = p;         
	      break;
	      
		case CM_TOUCH_LOCK:
			if(p==0)
			{
		         __TOUCH_LOCK__ = 0;         
		      }else{
		         __TOUCH_LOCK__ = 1;        
		      }
	      break;  
      
		case CM_DEVICE_MODULE:
      		i = check_hardware_name();
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;
              	          	
		case CM_BLUETOOTH_RESET:
			gpio_set_value(GPIO_WIFI_RST, 0);	// WIFI_RST
    		sleep_on_timeout(&Reset_WaitQueue,HZ/100);
			gpio_set_value(GPIO_WIFI_RST, 1);
			mxc_mmc_force_detect (2);
			schedule_timeout (500);
			break;
			
		case CM_DEVICE_INFO:		  
		  	collect_hardware_info(&info);
		  	copy_to_user((void __user *)arg, &info, sizeof(info));
      		break;	
      		
		case CM_ROTARY_STATUS:	
			if (4 == check_hardware_name() || 2 == check_hardware_name() || 3 == check_hardware_name()) {
				i = mma7660_read_orient ();
				if (2 == check_hardware_name()) {
					if (5 == (++i))
						i = 1;
				}
			}
			else
				i = 0;
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
      		break;	
      		
		case CM_ROTARY_ENABLE:	
			switch (check_hardware_name()) {
				case 4:
				case 3:
					if (p)
						enable_irq_wake(gpio_to_irq(G_SENSOR_INT));
					else
						disable_irq_wake(gpio_to_irq(G_SENSOR_INT));
					break;
				case 2:
					if (p)
						enable_irq_wake(gpio_to_irq(E50602_G_SENSOR_INT));
					else
						disable_irq_wake(gpio_to_irq(E50602_G_SENSOR_INT));
					break;
				default:
					break;
			}
      		break;	
		case CM_GET_KEYS:
			i = 0;
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			break;
		case CM_POWER_BTN:
		case CM_GET_KEY_STATUS:	
			if (gIsMSP430IntTriggered) {
				if ((6 != check_hardware_name()) && (2 != check_hardware_name())) {
					unsigned int tmp;
					tmp = msp430_read (0x60);
	//				printk ("[%s-%d] Micro P MSP430 status 0x%04X ....\n", __func__, __LINE__,tmp);
					if (0x8000 & tmp) {
						printk ("[%s-%d] =================> Micro P MSP430 alarm triggered <===================\n", __func__, __LINE__);
						g_wakeup_by_alarm = 1;
					}
					if (0x01 & tmp) {
						printk ("[%s-%d] =================> Micro P MSP430 Critical_Battery_Low <===================\n", __func__, __LINE__);
						gMxcPowerKeyIrqTriggered = 1;
						g_power_key_debounce = 5;	// Joseph 20100921 for ESD
						g_power_key_pressed = 1;
						gLastBatTick = jiffies-500;	
					}
					else
						gIsMSP430IntTriggered = 0;
				}
			}
			
			if (g_power_key_pressed) {
				g_power_key_pressed = 0;
				i = 1;
			}
			else {
				if ((6 == check_hardware_name()) || (2 == check_hardware_name())) 		// E60632 || E50602
					i = (gpio_get_value (GPIO_PWR_SW))?1:0;	// POWER key
				else
					i = (gpio_get_value (GPIO_PWR_SW))?0:1;	// POWER key
				
				if (i) {
					if (2 >= g_power_key_debounce) { 	// Joseph 20100921 for ESD
						printk ("[%s-%d] power key bounce detected %d\n",__func__,__LINE__, g_power_key_debounce);
						i=0;
					}
					else {
						gMxcPowerKeyIrqTriggered = 0;
					}
				}
				else if (gMxcPowerKeyIrqTriggered) {	// POWER key interrupt triggered.
					if (2 < g_power_key_debounce) {
						i = 1;
					}
					else
						printk ("[%s-%d] power key bounce detected %d\n",__func__,__LINE__,g_power_key_debounce);
					gMxcPowerKeyIrqTriggered = 0;
				}
			}
			
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			g_mxc_touch_triggered = 0;
      		break;	
      		
		case CM_GET_WHEEL_KEY_STATUS:
			i=0;
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
      		break;	
      		
		case CM_3G_POWER:
			break;	
					
		case CM_3G_RF_OFF:
			break;	
					
		case CM_3G_RESET:
			break;	
					
		case CM_WIFI_CTRL:		
			ntx_wifi_power_ctrl (p);
			break;	
					
		case CM_3G_GET_WAKE_STATUS:	
			i = 0;	
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
	    	break;	
	    	
		case CM_SET_ALARM_WAKEUP:
	    	break;	
	    	
		case CM_GET_UP_VERSION:
			i = msp430_deviceid();
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
	    	break;

		case CM_AUDIO_GET_VOLUME:
			{
			#ifdef CONFIG_SND_SOC_ALC5623 //[
			extern int alc5623_get_volume(void);
			i = alc5623_get_volume();
			#else //][!CONFIG_SND_SOC_ALC5623
			i=0;
			#endif //]CONFIG_SND_SOC_ALC5623
			copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			}
			break;
			
		case CM_AUDIO_SET_VOLUME:
			{
			#ifdef CONFIG_SND_SOC_ALC5623//[
			extern int alc5623_set_volume(int iSet);
			i = alc5623_set_volume(p);
			#else //][!CONFIG_SND_SOC_ALC5623
			i = 0;
			#endif//]CONFIG_SND_SOC_ALC5623
			//copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			}
			break;

		case CM_FRONT_LIGHT_SET:
			if(0!=gptHWCFG->m_val.bFrontLight)
			{
				if (p) {			
					printk ("\nset front light level : %d\n",p);
					if(p>0 && p<=50)
					{
						gpio_direction_output(FL_R_EN,0);
						msp430_write (0xA7, FL_table0[2*(p-1)]&0xFF00);
						msp430_write (0xA6, FL_table0[2*(p-1)]<<8);
						printk("PWMCNT : 0x%04x\n", FL_table0[2*(p-1)]);
					}else if(p>50 && p<=100){
						gpio_direction_output(FL_R_EN,1);
						msp430_write (0xA7, FL_table0[p-1]&0xFF00);
						msp430_write (0xA6, FL_table0[p-1]<<8);
						printk("PWMCNT : 0x%04x\n", FL_table0[p-1]);
					}else{
						printk("Wrong number! level range from 0 to 100\n");
					}
					if (0 == last_FL_duty){
						msp430_write (0xA1, 0xFF00);
						msp430_write (0xA2, 0xFF00);
						msp430_write (0xA5, 0x0100);   
						msp430_write (0xA4, 0x9000);
						msp430_write (0xA3, 0x0100);

						msleep(100);
						gpio_direction_output(FL_EN,0);
					}
				}
				else {
					printk ("turn off front light\n");
					msp430_write (0xA3, 0);

					gpio_direction_input(FL_EN);
					gpio_direction_output(FL_R_EN,0);
				}
				last_FL_duty = p;
			}
			break;

		case CM_FRONT_LIGHT_AVAILABLE:
			{
      	i = (unsigned long) (gptHWCFG->m_val.bFrontLight?1:0) ;
				copy_to_user((void __user *)arg, &i, sizeof(unsigned long));
			}
			break;

		case CM_FRONT_LIGHT_DUTY:
			if(0!=gptHWCFG->m_val.bFrontLight)
			{
				if (p) {			
					printk ("\nSet front light PWMCNT : 0x%4X\n",p);
					printk ("Current front light Frequency : (8MHz/0x%4X)\n",current_FL_freq);		
					msp430_write (0xA7, p&0xFF00);
					msp430_write (0xA6, p<<8);
					if (0 == last_FL_duty){
						msp430_write (0xA1, 0xFF00);
						msp430_write (0xA2, 0xFF00);
//						msp430_write (0xA5, 0xFF00);   
//						msp430_write (0xA4, 0xFF00);
						msp430_write (0xA3, 0x0100);

						msleep(100);
						gpio_direction_output(FL_EN,0);
					}
				}
				else {
					printk ("turn off front light\n");
					msp430_write (0xA3, 0);

					gpio_direction_input(FL_EN);
				}
				last_FL_duty = p;
			}
			break;

		case CM_FRONT_LIGHT_FREQUENCY:
			if(0!=gptHWCFG->m_val.bFrontLight)
			{
				if (p) {
					printk ("set front light Frequency : (8MHz/0x%4X)\n",p);		
//					msp430_write (0xA4, (p<<8));
					msp430_write (0xA5, p&0xFF00);   
					msp430_write (0xA4, (p<<8));
					current_FL_freq = p;
				}
			}
			break;

		case CM_PLATFORM:
			copy_to_user((void __user *)arg, &platform_type, 32);
			break;
		case CM_HWCONFIG:
			copy_to_user((void __user *)arg, &hwconfig, sizeof(unsigned long
				     long));
			break;
		case CM_SET_HWCONFIG:
			if (!capable(CAP_SYS_ADMIN)) 
				return -EPERM;
			copy_from_user(&hwconfig, (void __user *)arg, sizeof(unsigned long
				       long));
			break;

		default:
			printk("pvi_io : do not get the command [%d]\n", command);
			return -1;
	}
	return 0;
}	
static struct file_operations driverFops= {
    .owner  	=   THIS_MODULE,
    .open   	=   openDriver,
    .ioctl	=   ioctlDriver,
    .release    =   releaseDriver,
};
static struct miscdevice driverDevice = {
	.minor		= DEVICE_MINJOR,
	.name		= DEVICE_NAME,
	.fops		= &driverFops,
};

// ================================= Simulate MC13892 Signaling LED Driver ================================
static struct timer_list green_led_timer, blue_led_timer, red_led_timer;
static unsigned char green_led_dc, blue_led_dc, red_led_dc, \ 
					 green_led_period, blue_led_period, red_led_period, \
					 green_led_flag, blue_led_flag, red_led_flag;

void ntx_led_set_timer (struct timer_list *pTimer, unsigned char dc, unsigned char blink)
{
	int period;
	
	if (0 == dc)
		return;
	switch (blink) {
	case 0:		// 1/256 s
		return;
	case 1:
		period = 100 / 8;	// 1/8 s
		break;
	case 2:
		period = 100;	// 1 s
		break;
	case 3:
		period = 200;	// 2 s
		break;
	default:
		return;
	}
	mod_timer(pTimer, jiffies + period);
}

static void green_led_blink_func (unsigned long v)
{
	green_led_flag ^= 1;
	gpio_set_value (GPIO_LED_ON, green_led_flag);
	ntx_led_set_timer (&green_led_timer, green_led_dc, green_led_period);
}

static void blue_led_blink_func (unsigned long v)
{
	blue_led_flag ^= 1;
	gpio_set_value (GPIO_ACT_ON, blue_led_flag);
	ntx_led_set_timer (&blue_led_timer, blue_led_dc, blue_led_period);
}

static void red_led_blink_func (unsigned long v)
{
	red_led_flag ^= 1;
	gpio_set_value (GPIO_CHG_LED, red_led_flag);
	ntx_led_set_timer (&red_led_timer, red_led_dc, red_led_period);
}

void ntx_led_blink (unsigned int channel, unsigned char period)
{
	g_Cus_Ctrl_Led = 1;
	switch (channel) {
	case 3:
		red_led_period = period&3;
		ntx_led_set_timer (&red_led_timer, red_led_dc, red_led_period);
		break;
	case 4:
		green_led_period = period&3;
		ntx_led_set_timer (&green_led_timer, green_led_dc, green_led_period);
		break;
	case 5:
		blue_led_period = period&3;
		ntx_led_set_timer (&blue_led_timer, blue_led_dc, blue_led_period);
		break;
	default:
		break;
	}
}

void ntx_led_dc (unsigned int channel, unsigned char dc)
{
	LED_conitnuous = 0;
	g_Cus_Ctrl_Led = 1;
	switch (channel) {
	case 3:
		red_led_dc = dc;
		red_led_flag = (dc)?0:1;
		gpio_set_value (GPIO_CHG_LED,red_led_flag);
		ntx_led_set_timer (&red_led_timer, red_led_dc, red_led_period);
		break;
	case 4:
		green_led_dc = dc;
		green_led_flag = (dc)?0:1;
		gpio_set_value (GPIO_LED_ON,green_led_flag);
		ntx_led_set_timer (&green_led_timer, green_led_dc, green_led_period);
		break;
	case 5:
		blue_led_dc = dc;
		blue_led_flag = (dc)?0:1;
		gpio_set_value (GPIO_ACT_ON,blue_led_flag);
		ntx_led_set_timer (&blue_led_timer, blue_led_dc, blue_led_period);
		break;
	default:
		break;
	}
}

void ntx_led_current (unsigned int channel, unsigned char value)
{
	g_Cus_Ctrl_Led = 1;
	if (!value)
		ntx_led_dc (channel, 0);
}

void led_green (int isOn)
{
	gpio_set_value (GPIO_LED_ON,(isOn)?0:1);
}

void led_blue (int isOn)
{
	gpio_set_value (GPIO_ACT_ON,(isOn)?0:1);
}

void led_red (int isOn) {
	gpio_set_value (GPIO_CHG_LED,(isOn)?0:1);
}

static void LED(int on)
{
	switch (check_hardware_name())
	{
	case 4:	// E60622
	case 2:	// E50602 (UPG)
	case 6:	// E60632 (UPG)
		if(on) 
			gpio_set_value (GPIO_ACT_ON,0);
		else
	    	gpio_set_value (GPIO_ACT_ON,1);
    	break;
    default:
		if(on) 
			gpio_set_value (GPIO_LED_ON,0);
		else
	    	gpio_set_value (GPIO_LED_ON,1);
    	break;
    }
}

static int sleep_thread(void)
{
int	rc = 0;

//	try_to_freeze();
	set_current_state(TASK_INTERRUPTIBLE);
	if(signal_pending(current))
	  rc = -EINTR;
	__set_current_state(TASK_RUNNING);
	return rc;
}

static int LED_Thread(void *param)
{
	  sigset_t		thread_signal_mask;	  
  	siginitsetinv(&thread_signal_mask, sigmask(SIGKILL));
	  sigprocmask(SIG_SETMASK, &thread_signal_mask, NULL);
	  
    while(1){
      if(freezing(current)){
          printk("freeze 0 !!!!!!!!!!!!!!!!!!!!\n");
          //interruptible_sleep_on(&LED_freeze_WaitQueue);
          //sleep_thread();
		  try_to_freeze();
      }

      if(LED_conitnuous == 0){
//          LED(0);
          interruptible_sleep_on(&LED_freeze_WaitQueue);
          if(freezing(current)){
            printk("freeze 1!!!!!!!!!!!!!!!!!!!!\n");
            //interruptible_sleep_on(&LED_freeze_WaitQueue);
            //sleep_thread();
	    	try_to_freeze();
          }
      }
      if (g_Cus_Ctrl_Led) {
      	LED_conitnuous = 0;
      	continue;
      }
      LED(1);
	  while (gKeepPowerAlive) {
	      sleep_on_timeout(&Reset_WaitQueue,HZ*2);
		  msp430_powerkeep(1);
	  }
      //start to blink LED;
      if (2 == LED_conitnuous) {
		spin_lock(&led_flash_lock);
//      	if (0>=LED_Flash_Count) {
      		LED_Flash_Count = 0;
      		LED_conitnuous = 0;
//      	}
//      	else {
//      		LED_Flash_Count--;
//      	}
		spin_unlock(&led_flash_lock);
      	sleep_on_timeout(&LED_blink_WaitQueue,HZ/10);
	    LED(0);
      	sleep_on_timeout(&LED_blink_WaitQueue,HZ/10);
        LED(1);
      	sleep_on_timeout(&LED_blink_WaitQueue,HZ/10);
      	if (!green_led_dc) 
	    	LED(0);
      	sleep_on_timeout(&LED_blink_WaitQueue,HZ/10);
      }
      else {
      	sleep_on_timeout(&LED_blink_WaitQueue,HZ/2);
      	if (!green_led_dc) 
		    LED(0);
	    sleep_on_timeout(&LED_blink_WaitQueue,HZ/2);
      }
    }    
	return 0;
}

int wlan_power (bool status)
{
	if (status) {
//		gpio_direction_output(MFP_PIN_GPIO44, 0);	// turn on Wifi_1V8_on
//		gpio_direction_output(MFP_PIN_GPIO45, 0);	// turn on Wifi_3V3_on
	}
	else {
//		__raw_writel(0x0b, APMU_SDH2);	// enable SD3_SD4_AXICLK, This have to be done when wifi power on.
//		gpio_direction_input(MFP_PIN_GPIO44);	// turn off Wifi_1V8_on
//		gpio_direction_input(MFP_PIN_GPIO45);	// turn off Wifi_3V3_on
	}
	return 0;
}
EXPORT_SYMBOL(wlan_power);

int wlan_reset(bool status)
{
	gpio_set_value(GPIO_WIFI_RST, ((status)?1:0));	// wifi_RST
	return 0;
}
EXPORT_SYMBOL(wlan_reset);

/*!
 *  rotary seneor interrupt handler.
 */
static irqreturn_t rotary_int(int irq, void *dev_id)
{
	pr_info(KERN_INFO "rotary sensor triggered ...\n");
	return 0;
}

static irqreturn_t c_touch_int(int irq, void *dev_id)
{
	return 0;
}

int mxc_usb_plug_getstatus (void)
{
	if (gIsCustomerUi) {
		g_ioctl_USB_status = gpio_get_value (GPIO_ACIN_PG);
		return (g_ioctl_USB_status)?0:1;
	}
	else
		return 0;
}

int pxa168_chechk_suspend (void)
{

	if ( gMxcPowerKeyIrqTriggered || gIsMSP430IntTriggered || g_mxc_touch_triggered) {
		printk ("[%s-%d] key triggered ( %d, %d, %d)....\n", 
			__func__, __LINE__, gMxcPowerKeyIrqTriggered, gIsMSP430IntTriggered, g_mxc_touch_triggered);
		return 0;
	}
	else if (!gpio_get_value (GPIO_MSP_INT)) {
		printk ("[%s-%d] MSP430 int triggered ....\n", 	__func__, __LINE__);
		return 0;
	}
	else if (g_ioctl_SD_status != gpio_get_value (SD2_CD)) {
		printk ("[%s-%d] SD status changed ....\n", __func__, __LINE__);
		return 0;
	}
	else if (g_ioctl_USB_status != gpio_get_value (GPIO_ACIN_PG)) {
		printk ("[%s-%d] USB status changed ....\n", __func__, __LINE__);
		return 0;
	}
	return 1;
}

static struct timer_list power_key_timer;
extern void mxc_kpp_report_power(int isDown);

static void power_key_chk(unsigned long v)
{
	int pwr_key;
	
	if ((6 == check_hardware_name()) || (2 == check_hardware_name())) 		// E60632 || E50602
		pwr_key = gpio_get_value (GPIO_PWR_SW)?1:0;
	else
		pwr_key = gpio_get_value (GPIO_PWR_SW)?0:1;
	
	if (pwr_key) {
		++g_power_key_debounce;
		if ((2 == g_power_key_debounce) && gIsCustomerUi)
			mxc_kpp_report_power(1);
		mod_timer(&power_key_timer, jiffies + 1);
	}
	else if (gIsCustomerUi)
		mxc_kpp_report_power(0);
}

void power_key_int_function(void)
{
	gMxcPowerKeyIrqTriggered = 1;
	g_power_key_debounce = 0;
	mod_timer(&power_key_timer, jiffies + 1);
}

static irqreturn_t power_key_int(int irq, void *dev_id)
{
	power_key_int_function();
	return 0;
}

static struct timer_list acin_pg_timer;
static int g_acin_pg_debounce;
typedef void (*usb_insert_handler) (char inserted);
extern usb_insert_handler mxc_misc_report_usb;
extern void ntx_charger_online_event_callback(void);

static void acin_pg_chk(unsigned long v)
{
	int i;

	if (!gpio_get_value (GPIO_ACIN_PG)) {
		++g_acin_pg_debounce;
		if (10 == g_acin_pg_debounce) {
			if (gIsCustomerUi) {
				mxc_misc_report_usb(1);
				ntx_charger_online_event_callback ();
			}
		}
		mod_timer(&acin_pg_timer, jiffies + 1);
	}
	else {
		//if (gLastBatValue)
		//	gLastBatValue += 50;

		if (gIsCustomerUi) {
			mxc_misc_report_usb(0);
			ntx_charger_online_event_callback ();
		}
	}
}

int ntx_charge_status (void)
{
	return ((gpio_get_value (GPIO_ACIN_PG)?0:1) | (gpio_get_value (GPIO_CHG)?0:2));
}
int ntx_get_battery_vol (void)
{
	int i, battValue, temp, result;
	const unsigned short battGasgauge[] = {
	//	3.0V, 3.1V, 3.2V, 3.3V, 3.4V, 3.5V, 3.6V, 3.7V, 3.8V, 3.9V, 4.0V, 4.1V, 4.2V,
//		 743,  767,  791,  812,  835,  860,  885,  909,  935,  960,  985, 1010, 1023,
		 767,  791,  812,  833,  852,  877,  903,  928,  950,  979,  993, 1019, 1023,
	};
	
	gIsMSP430IntTriggered = 0;
	if (gUSB_Change_Tick) {
		if (500 < (jiffies - gUSB_Change_Tick)) {
			gUSB_Change_Tick = 0;
			gLastBatValue = 0;
		}
	}
	
	if (gIsMSP430IntTriggered || !gLastBatValue || ((0 == gUSB_Change_Tick) && (200 < (jiffies - gLastBatTick)))) {
		battValue = msp430_battery ();
		if (battValue) {
			gLastBatTick = jiffies;
			if (gpio_get_value (GPIO_ACIN_PG)) {// not charging
				temp = msp430_read (0x60);
				if (-1 != temp ) {
					if (0x8000 & temp) {
						printk ("[%s-%d] =================> Micro P MSP430 alarm triggered <===================\n", __func__, __LINE__);
						g_wakeup_by_alarm = 1;
					}
					if (0x01 & temp) {
						printk ("[%s-%d] =================> Micro P MSP430 Critical_Battery_Low <===================\n", __func__, __LINE__);
						return 0;
					}
					else if (!gLastBatValue) 
						gLastBatValue = battValue;
					else if (gLastBatValue > battValue)
						gLastBatValue = battValue;
					else
						battValue = gLastBatValue;
				}
			}
			else {
				if (gLastBatValue < battValue)
					gLastBatValue = battValue;
				else
					battValue = gLastBatValue;
			}
		}
		else {
			printk ("[%s-%d] MSP430 read failed\n", __func__, __LINE__);
			battValue = 0;
		}
	}
	else 
		battValue = gLastBatValue;
	
	// transfer to uV to pmic interface.
	for (i=0; i< sizeof (battGasgauge); i++) {                 
		if (battValue <= battGasgauge[i]) {
			if (i && (battValue != battGasgauge[i])) {
				result = 3000000+ (i-1)*100000;
				result += ((battValue-battGasgauge[i-1]) * 100000 / (battGasgauge[i]-battGasgauge[i-1]));
			}
			else
				result = 3000000+ i*100000;
			break;
		}
	}
//	printk ("[%s-%d] battery %d (%d)\n", __func__, __LINE__, battValue,result);
	if (4100000 <= result) {
		printk("%s : full !! %d\n",__FUNCTION__,result);
		return 100;
	}
	if (3400000 > result) {
		printk("%s : empty !! %d\n",__FUNCTION__,result);
		return 0;
	}
	result = 4100000 - result;
	result /= 7000;
	printk ("[%s-%d] %d,bat=%d\n", __func__, __LINE__, (100-result),battValue);
	return 100-result;
}

static irqreturn_t ac_in_int(int irq, void *dev_id)
{
	gUSB_Change_Tick = jiffies;	// do not check battery value in 6 seconds
	if (gpio_get_value (GPIO_ACIN_PG)) 
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
	else {
		set_irq_type(irq, IRQF_TRIGGER_RISING);
	}
	
	g_acin_pg_debounce = 0;
	mod_timer(&acin_pg_timer, jiffies + 1);
	return 0;
}

/*!
 *  Key raw pins interrupt handler.
 */
static irqreturn_t gpio_key_row_int(int irq, void *dev_id)
{
//	pr_info(KERN_INFO "key matrix pressed ...\n");
	return 0;
}

static irqreturn_t msp_int(int irq, void *dev_id)
{
	printk ("[%s-%d] MSP430 interrupt triggered !!!\n",__func__,__LINE__);
	gIsMSP430IntTriggered = 1;
	return 0;
}


#ifdef NTX_GPIO_KEYS //[

#define HALLSENSOR_KEY  (4*32 + 25) /* GPIO_5_25 */

#define NTX_GPIO_KEYS_MAX		5
static int gi_ntx_gpio_buttons_total = 0;
static struct gpio_keys_button ntx_gpio_buttons[NTX_GPIO_KEYS_MAX] = {
	{0,},
};
static struct gpio_keys_platform_data ntx_gpio_key_data = {
  .buttons=ntx_gpio_buttons,
  .nbuttons=0,
  .rep=0,
};
static struct platform_device ntx_gpio_key_device = {
  .name = "gpio-keys",
  .id = -1,
  .dev = {
    .platform_data = &ntx_gpio_key_data,
  },
};

#endif//] NTX_GPIO_KEYS



#define IR_TOUCH_RST		(4*32 + 26)	/*GPIO_5_26 */

static int gpio_initials(void)
{
	int irq, ret;
	int error;
	
	// Epson_3v3_PWR_ON 

	// 3G_RF_Off

	// 3G_RF_RST

	// 3G_RF_Wake

	/* ACIN PG */
	mxc_iomux_v3_setup_pad(MX50_PAD_ECSPI2_SS0__GPIO_4_19);
	gpio_request(GPIO_ACIN_PG, "acin_pg");
	gpio_direction_input(GPIO_ACIN_PG);
	{
		/* Set AC in as wakeup resource */
		irq = gpio_to_irq(GPIO_ACIN_PG);
		
		if (gpio_get_value (GPIO_ACIN_PG))
			set_irq_type(irq, IRQF_TRIGGER_FALLING);
		else
			set_irq_type(irq, IRQF_TRIGGER_RISING);
		ret = request_irq(irq, ac_in_int, 0, "acin_pg", 0);
		if (ret)
			pr_info("register ACIN_PG interrupt failed\n");
		else
			enable_irq_wake(irq);
		acin_pg_timer.function = acin_pg_chk;
		init_timer(&acin_pg_timer);
	}
	mxc_iomux_v3_setup_pad(MX50_PAD_ECSPI2_MISO__GPIO_4_18);
	gpio_request(GPIO_CHG, "charge_det");
	gpio_direction_input(GPIO_CHG);
	
	/* ON_LED */
	mxc_iomux_v3_setup_pad(MX50_PAD_EIM_OE__GPIO_1_24);
	gpio_request(GPIO_LED_ON, "led_on");
	gpio_direction_output(GPIO_LED_ON, 1);
	
	mxc_iomux_v3_setup_pad(MX50_PAD_PWM1__GPIO_6_24);
	gpio_request(GPIO_ACT_ON, "action_on");
	gpio_direction_output(GPIO_ACT_ON, 1);
	
	/* CHG_LED */
	mxc_iomux_v3_setup_pad(MX50_PAD_EIM_RW__GPIO_1_25);
	gpio_request(GPIO_CHG_LED, "charge_led");
	gpio_direction_output(GPIO_CHG_LED, 1);
	
	/* Audio_PWR_ON */
	mxc_iomux_v3_setup_pad(MX50_PAD_ECSPI2_MOSI__GPIO_4_17);
	gpio_request(GPIO_AUDIO_PWR, "audio_pwr");
	gpio_direction_input(GPIO_AUDIO_PWR);
	
	/* AMP_EN */
	
	/* OFF_CHK */
	#ifdef GPIOFN_PWRKEY//[
	gpiofn_register(&gtNTX_PWR_GPIO_data);
	#else //][!GPIOFN_PWRKEY
	mxc_iomux_v3_setup_pad(MX50_PAD_CSPI_MISO__GPIO_4_10);
	gpio_request(GPIO_PWR_SW, "pwr_sw");
	gpio_direction_input(GPIO_PWR_SW);
	{
		/* Set power key as wakeup resource */
		irq = gpio_to_irq(GPIO_PWR_SW);
	
		if ((6 == check_hardware_name()) || (2 == check_hardware_name())) 		// E60632 || E50602
			set_irq_type(irq, IRQF_TRIGGER_RISING);
		else
			set_irq_type(irq, IRQF_TRIGGER_FALLING);
		ret = request_irq(irq, power_key_int, 0, "power_key", 0);
		if (ret)
			pr_info("register on-off key interrupt failed\n");
		else
			enable_irq_wake(irq);
	}
	#endif //]GPIOFN_PWRKEY
	power_key_timer.function = power_key_chk;
	init_timer(&power_key_timer);
	
	tle4913_init();
	
//	if ((1 == check_hardware_name()) || (10 == check_hardware_name()) || (14 == check_hardware_name())) {	// E60612 , E606A2 ,E606B2 , ir touch 
	if(4==gptHWCFG->m_val.bTouchType) {    //IR touch

		// Touch interrupt
		mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D7__GPIO_5_15);
		gpio_request(TOUCH_INT, "touch_int");
		gpio_direction_input(TOUCH_INT);
		irq = gpio_to_irq(TOUCH_INT);
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
		
	    // Touch reset
		mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D6__GPIO_5_26);
		gpio_request(IR_TOUCH_RST, "ir_touch_rst");	
		gpio_direction_output(IR_TOUCH_RST, 0);
		msleep(20);
		gpio_direction_input(IR_TOUCH_RST);
	}
	else {		// C touch.
		// Touch interrupt
		mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D7__GPIO_5_27);
		gpio_request(C_TOUCH_INT, "c_touch_int");
		gpio_direction_input(C_TOUCH_INT);
		
#ifdef DIGITIZER_TEST
		set_irq_type(gpio_to_irq(C_TOUCH_INT), IRQF_TRIGGER_RISING);
		ret = request_irq(gpio_to_irq(C_TOUCH_INT), c_touch_int, 0, "c_touch", 0);
		if (ret)
			pr_info("register G_SENSOR_INT interrupt failed\n");
		else
			enable_irq_wake(gpio_to_irq(C_TOUCH_INT));
#endif
		
		// Touch reset
		mxc_iomux_v3_setup_pad(MX50_PAD_SD3_WP__GPIO_5_28);
		gpio_request(TOUCH_RST, "touch_rst");
		gpio_direction_output(TOUCH_RST, 0);
		
		// Touch power
		mxc_iomux_v3_setup_pad(MX50_PAD_ECSPI2_SCLK__GPIO_4_16);
		gpio_request(TOUCH_PWR, "touch_pwr");
		gpio_direction_output(TOUCH_PWR, 0);
		
		// Touch enable
		mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D6__GPIO_5_26);
		gpio_request(TOUCH_EN, "touch_en");
#ifdef DIGITIZER_TEST
#ifdef HANVON_TOUCH
		gpio_direction_output(TOUCH_EN, 0);
#else		
		gpio_direction_output(TOUCH_EN, 1);	// PVI touch
#endif
		gpio_direction_output(TOUCH_RST, 1);
		msleep (10);
		gpio_direction_output(TOUCH_RST, 0);
		msleep (100);
		gpio_direction_output(TOUCH_RST, 1);
#else
		gpio_direction_output(TOUCH_EN, 1);
		gpio_direction_output(TOUCH_RST, 1);
		msleep (50);
#endif
		mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D7__GPIO_5_27_PU);
	}
	
	if (4 == check_hardware_name() || 3 == check_hardware_name()) {
		// MMA7660 INT
		mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D5__GPIO_5_25);
		gpio_request(G_SENSOR_INT, "touch_rst");
		gpio_direction_input(G_SENSOR_INT);

		set_irq_type(gpio_to_irq(G_SENSOR_INT), IRQF_TRIGGER_FALLING);
		ret = request_irq(gpio_to_irq(G_SENSOR_INT), rotary_int, 0, "rotary_a", 0);
		if (ret)
			pr_info("register G_SENSOR_INT interrupt failed\n");
		else
			enable_irq_wake(gpio_to_irq(G_SENSOR_INT));
	}
	else if (2 == check_hardware_name()) {	
		// MMA7660 INT
		mxc_iomux_v3_setup_pad(MX50_PAD_ECSPI1_SS0__GPIO_4_15);
		gpio_request(E50602_G_SENSOR_INT, "g_sensor_int");
		gpio_direction_input(E50602_G_SENSOR_INT);

		set_irq_type(gpio_to_irq(E50602_G_SENSOR_INT), IRQF_TRIGGER_FALLING);
		ret = request_irq(gpio_to_irq(E50602_G_SENSOR_INT), rotary_int, 0, "rotary_a", 0);
		if (ret)
			pr_info("register E50602_G_SENSOR_INT interrupt failed\n");
		else
			enable_irq_wake(gpio_to_irq(E50602_G_SENSOR_INT));
			
		mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CLK__SD2_CLK_DSL);
		mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CMD__SD2_CMD_DSL);
		mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D0__SD2_D0_DSL);
		mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D1__SD2_D1_DSL);
		mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D2__SD2_D2_DSL);
		mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D3__SD2_D3_DSL);
	}

	switch (gptHWCFG->m_val.bPCB) {
		case 21: // E60610D
		case 22: // E606AX 
		{
			// E60610D 
			printk("ESD DSM\n");
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CLK__SD2_CLK_DSM);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CMD__SD2_CMD_DSM);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D0__SD2_D0_DSM);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D1__SD2_D1_DSM);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D2__SD2_D2_DSM);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D3__SD2_D3_DSM);
		}
		break;
		/*
		case 22:
		{
			// E606A2 
			printk("ESD DSL\n");
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CLK__SD2_CLK_DSL);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_CMD__SD2_CMD_DSL);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D0__SD2_D0_DSL);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D1__SD2_D1_DSL);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D2__SD2_D2_DSL);
			mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D3__SD2_D3_DSL);
		}
		break;
		*/

		default :
		break;
	}
	
	
	// FL_EN
	if(0!=gptHWCFG->m_val.bFrontLight) {	
		mxc_iomux_v3_setup_pad(MX50_PAD_ECSPI1_MISO__GPIO_4_14);
		gpio_request(FL_EN, "fl_en");
		gpio_direction_input(FL_EN);
		
		mxc_iomux_v3_setup_pad(MX50_PAD_EPDC_VCOM1__GPIO_4_22);
		gpio_request(FL_R_EN, "fl_r_en");
		gpio_direction_output(FL_R_EN,0);
	}
	
	// WIFI_3V3_ON 
	mxc_iomux_v3_setup_pad(MX50_PAD_ECSPI1_SCLK__GPIO_4_12);
	gpio_request(GPIO_WIFI_3V3, "wifi_3v3");
	gpio_direction_input(GPIO_WIFI_3V3);
		
	// WIFI_1V8_ON
		
	// WIFI_RST
	mxc_iomux_v3_setup_pad(MX50_PAD_SD2_D6__GPIO_5_14);
	gpio_request(GPIO_WIFI_RST, "wifi_rst");
	gpio_direction_output(GPIO_WIFI_RST,0);
	
	// WIFI_IRQ 
	mxc_iomux_v3_setup_pad(MX50_PAD_CSPI_SCLK__GPIO_4_8);
	gpio_request(GPIO_WIFI_INT, "wifi_int");
	gpio_direction_input(GPIO_WIFI_INT);
#ifdef _WIFI_ALWAYS_ON_
	irq = gpio_to_irq(GPIO_WIFI_INT);
	set_irq_type(irq, IRQF_TRIGGER_FALLING);
//	enable_irq_wake(irq);
#endif


	// EIM pins for power comsumption .
  gpio_request(EIM_DA0, "eim-da0");
  gpio_direction_input(EIM_DA0);
  gpio_request(EIM_DA1, "eim-da1");
  gpio_direction_input(EIM_DA1);
  gpio_request(EIM_DA2, "eim-da2");
  gpio_direction_input(EIM_DA2);
  gpio_request(EIM_DA3, "eim-da3");
  gpio_direction_input(EIM_DA3);
  gpio_request(EIM_DA4, "eim-da4");
  gpio_direction_input(EIM_DA4);
  gpio_request(EIM_DA5, "eim-da5");
  gpio_direction_input(EIM_DA5);
  gpio_request(EIM_DA6, "eim-da6");
  gpio_direction_input(EIM_DA6);
  gpio_request(EIM_DA7, "eim-da7");
  gpio_direction_input(EIM_DA7);
  gpio_request(EIM_DA8, "eim-da8");
  gpio_direction_input(EIM_DA8);
  gpio_request(EIM_DA9, "eim-da9");
  gpio_direction_input(EIM_DA9);
  gpio_request(EIM_DA10, "eim-da10");
  gpio_direction_input(EIM_DA10);
  gpio_request(EIM_DA11, "eim-da11");
  gpio_direction_input(EIM_DA11);
  gpio_request(EIM_DA12, "eim-da12");
  gpio_direction_input(EIM_DA12);
  gpio_request(EIM_DA13, "eim-da13");
  gpio_direction_input(EIM_DA13);
  gpio_request(EIM_DA14, "eim-da14");
  gpio_direction_input(EIM_DA14);
  gpio_request(EIM_DA15, "eim-da15");
  gpio_direction_input(EIM_DA15);
  gpio_request(EIM_CS2, "eim-cs2");
  gpio_direction_input(EIM_CS2);
  gpio_request(EIM_CS1, "eim-cs1");
  gpio_direction_input(EIM_CS1);
  gpio_request(EIM_CS0, "eim-cs0");
  gpio_direction_input(EIM_CS0);
  gpio_request(EIM_EB0, "eim-eb0");
  gpio_direction_input(EIM_EB0);
  gpio_request(EIM_EB1, "eim-eb1");
  gpio_direction_input(EIM_EB1);
  gpio_request(EIM_WAIT, "eim-wait");
  gpio_direction_input(EIM_WAIT);
  gpio_request(EIM_BCLK, "eim-bclk");
  gpio_direction_input(EIM_BCLK);
  gpio_request(EIM_RDY, "eim-rdy");
  gpio_direction_input(EIM_RDY);


	// ESD_WP 

#ifdef NTX_GPIO_KEYS //[

  // Hall Sensor key .
	if(gptHWCFG->m_val.bHallSensor) {
		mxc_iomux_v3_setup_pad(MX50_PAD_SD3_D5__GPIO_5_25);
  	gpio_request(HALLSENSOR_KEY, "HALLSENSOR_KEY");
  	gpio_direction_input(HALLSENSOR_KEY);
	}

#endif //] NTX_GPIO_KEYS


	// DCIN/MSP_INT#
	mxc_iomux_v3_setup_pad(MX50_PAD_CSPI_SS0__GPIO_4_11);
	gpio_request(GPIO_MSP_INT, "msp_int");
	gpio_direction_input (GPIO_MSP_INT);
	irq = gpio_to_irq(GPIO_MSP_INT);
	set_irq_type(irq, IRQF_TRIGGER_FALLING);
	ret = request_irq(irq, msp_int, 0, "msp_int", 0);
	if (ret)
		pr_info("register MSP430 interrupt failed\n");
	else
		enable_irq_wake(irq);

#if 0
	{
		/* Set power key as wakeup resource */
		int irq, ret;
	
		irq = gpio_to_irq(GPIO_KEY_ROW_0);
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
		ret = request_irq(irq, gpio_key_row_int, 0, "gpio_key_row_0", 0);
		if (ret)
			pr_info("register gpio_key_row_0 interrupt failed\n");
		enable_irq_wake (irq);
	
		irq = gpio_to_irq(GPIO_KEY_ROW_1);
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
		ret = request_irq(irq, gpio_key_row_int, 0, "gpio_key_row_1", 0);
		if (ret)
			pr_info("register gpio_key_row_1 interrupt failed\n");
		enable_irq_wake (irq);
	
		irq = gpio_to_irq(GPIO_KEY_ROW_2);
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
		ret = request_irq(irq, gpio_key_row_int, 0, "gpio_key_row_2", 0);
		if (ret)
			pr_info("register gpio_key_row_2 interrupt failed\n");
		enable_irq_wake (irq);
	
		irq = gpio_to_irq(GPIO_KEY_ROW_3);
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
		ret = request_irq(irq, gpio_key_row_int, 0, "gpio_key_row_3", 0);
		if (ret)
			pr_info("register gpio_key_row_3 interrupt failed\n");
		enable_irq_wake (irq);
	}
#endif
	// initial test point for ESD , Joseph 20100504
	return 0;
}

#include <mach/hardware.h>
#define SSI1_IO_BASE_ADDR	IO_ADDRESS(SSI1_BASE_ADDR)
#define SSI2_IO_BASE_ADDR	IO_ADDRESS(SSI2_BASE_ADDR)
#define SSI1_SCR    ((SSI1_IO_BASE_ADDR) + 0x10)
#define SSI2_SCR    ((SSI2_IO_BASE_ADDR) + 0x10)

#include <mach/arc_otg.h>
#include "crm_regs.h"
extern void __iomem *apll_base;
#define GPIO_PWRALL     (0*32 + 27) /* GPIO_1_27 */
unsigned long gUart2_ucr1;

void ntx_gpio_suspend (void)
{
#if 0
	printk ("[%s-%d] %s ()\n",__FILE__,__LINE__,__func__);
	printk ("\t MXC_CCM_CCGR0	0x%08X\n",__raw_readl(MXC_CCM_CCGR0));
	printk ("\t MXC_CCM_CCGR1	0x%08X\n",__raw_readl(MXC_CCM_CCGR1));
	printk ("\t MXC_CCM_CCGR2	0x%08X\n",__raw_readl(MXC_CCM_CCGR2));
	printk ("\t MXC_CCM_CCGR3	0x%08X\n",__raw_readl(MXC_CCM_CCGR3));
	printk ("\t MXC_CCM_CCGR4	0x%08X\n",__raw_readl(MXC_CCM_CCGR4));
	printk ("\t MXC_CCM_CCGR5	0x%08X\n",__raw_readl(MXC_CCM_CCGR5));
	printk ("\t MXC_CCM_CCGR6	0x%08X\n",__raw_readl(MXC_CCM_CCGR6));
	printk ("\t MXC_CCM_CCGR7	0x%08X\n",__raw_readl(MXC_CCM_CCGR7));
	printk ("\t MXC_ANADIG_FRAC0	0x%08X\n",__raw_readl(ioremap(ANATOP_BASE_ADDR, SZ_4K)+MXC_ANADIG_FRAC0));
	printk ("\t MXC_ANADIG_FRAC1	0x%08X\n",__raw_readl(ioremap(ANATOP_BASE_ADDR, SZ_4K)+MXC_ANADIG_FRAC1));
	printk ("\t MXC_ANADIG_MISC	0x%08X\n",__raw_readl(ioremap(ANATOP_BASE_ADDR, SZ_4K)+MXC_ANADIG_MISC));
	printk ("\t MXC_ANADIG_PLLCTRL	0x%08X\n",__raw_readl(ioremap(ANATOP_BASE_ADDR, SZ_4K)+MXC_ANADIG_PLLCTRL));
	printk ("\t HW_PERFMON_CTRL	0x%08X\n",__raw_readl(ioremap(PERFMON_BASE_ADDR, SZ_4K)));
/*
	printk ("\t SSI1_SCR	0x%08X\n",__raw_readl(SSI1_SCR));
	printk ("\t SSI2_SCR	0x%08X\n",__raw_readl(SSI2_SCR));
	printk ("\t PWM1CR	0x%08X\n",__raw_readl(ioremap(MX53_BASE_ADDR(PWM1_BASE_ADDR), SZ_4K)));
	printk ("\t PWM2CR	0x%08X\n",__raw_readl(ioremap(MX53_BASE_ADDR(PWM2_BASE_ADDR), SZ_4K)));
	printk ("\t CSPI1 CONREG	0x%08X\n",__raw_readl(ioremap(MX53_BASE_ADDR(CSPI1_BASE_ADDR), SZ_4K)+0x08));
	printk ("\t CSPI2 CONREG	0x%08X\n",__raw_readl(ioremap(MX53_BASE_ADDR(CSPI2_BASE_ADDR), SZ_4K)+0x08));
	printk ("\t CSPI3 CONREG	0x%08X\n",__raw_readl(ioremap(MX53_BASE_ADDR(CSPI3_BASE_ADDR), SZ_4K)+0x08));
	printk ("\t UART1 UCR1 0x%08X\n",__raw_readl(ioremap(MX53_BASE_ADDR(UART1_BASE_ADDR), SZ_4K)+0x80));
	printk ("\t UART2 UCR1 0x%08X\n", __raw_readl(ioremap(MX53_BASE_ADDR(UART2_BASE_ADDR), SZ_4K)+0x80));
//	printk ("\t FEC ECR	0x%08X\n",__raw_readl(ioremap(MX53_BASE_ADDR(FEC_BASE_ADDR), SZ_4K)+0x24));
	
	printk ("\t UH1_PORTSC1	0x%08X\n",UH1_PORTSC1);
	printk ("\t USBH1_PHY_CTRL0	0x%08X\n",USBH1_PHY_CTRL0);
	printk ("\t USB_CLKONOFF_CTRL	0x%08X\n",USB_CLKONOFF_CTRL);
	printk ("\t UOG_PORTSC1	0x%08X\n",UOG_PORTSC1);
*/
#endif

	g_wakeup_by_alarm = 0;
	if (gUSB_Change_Tick) 
		gUSB_Change_Tick = 0;

//	if (gSleep_Mode_Suspend && (1 != check_hardware_name()) && (10 != check_hardware_name()) && (14 != check_hardware_name())) {
	if (gSleep_Mode_Suspend && (4 != gptHWCFG->m_val.bTouchType)) {

		mdelay (20);
		disable_irq(gpio_to_irq(C_TOUCH_INT));
		gpio_direction_output(C_TOUCH_INT, 0);
		
		mdelay (20);
		if (2 == check_hardware_name()) {
			disable_irq(gpio_to_irq(E50602_G_SENSOR_INT));
			gpio_direction_output(E50602_G_SENSOR_INT, 0);
		}
		else {
			disable_irq(gpio_to_irq(G_SENSOR_INT));
			gpio_direction_output(G_SENSOR_INT, 0);
		}
		
		mxc_iomux_v3_setup_pad(MX50_PAD_I2C1_SDA__GPIO_6_19);
		gpio_request((5*32+19), "GPIO_6_19");
		gpio_direction_output((5*32+19),0);
		
		mxc_iomux_v3_setup_pad(MX50_PAD_I2C1_SCL__GPIO_6_18);
		gpio_request((5*32+18), "GPIO_6_18");
		gpio_direction_output((5*32+18),0);
		
#ifndef DIGITIZER_TEST
		gpio_direction_output(TOUCH_EN, 0);
		gpio_direction_output(TOUCH_RST, 0);
		gpio_direction_output(TOUCH_PWR, 1);
#endif
	}

	// turn off wifi power 
#ifndef _WIFI_ALWAYS_ON_
	gpio_direction_input(GPIO_WIFI_3V3);
	gpio_direction_output(GPIO_WIFI_RST,0);
#endif

	// turn off EPD power
	//gpio_direction_output(GPIO_PWRALL,0);

	// turn off audio power
	gpio_direction_input(GPIO_AUDIO_PWR);

	// MX50_PAD_PWM2__GPIO_6_25
	gpio_request((5*32+25), "GPIO_6_25");
	gpio_direction_output((5*32+25),0);
	// MX50_PAD_I2C2_SDA__GPIO_6_21
	gpio_request((5*32+21), "GPIO_6_21");
	gpio_direction_output((5*32+21),0);
	// MX50_PAD_I2C2_SCL__GPIO_6_20
	gpio_request((5*32+20), "GPIO_6_20");
	gpio_direction_output((5*32+20),0);

	
	gpio_set_value (GPIO_CHG_LED, 1);
	gpio_set_value (GPIO_LED_ON, 1);
	gpio_set_value (GPIO_ACT_ON, 1);

	__raw_writel(0x00058000, apll_base + MXC_ANADIG_MISC_SET);	// Powers down the bandgap reference
	gUart2_ucr1 = __raw_readl(ioremap(MX53_BASE_ADDR(UART2_BASE_ADDR), SZ_4K)+0x80);
	__raw_writel(0, ioremap(MX53_BASE_ADDR(UART2_BASE_ADDR), SZ_4K)+0x80);
	
}

void ntx_gpio_resume (void)
{
	__raw_writel(gUart2_ucr1, ioremap(MX53_BASE_ADDR(UART2_BASE_ADDR), SZ_4K)+0x80);
	__raw_writel(0x00058000, apll_base + MXC_ANADIG_MISC_CLR);
	
//	if (gSleep_Mode_Suspend && (1 != check_hardware_name()) && (10 != check_hardware_name()) && (14 != check_hardware_name())) {
	if (gSleep_Mode_Suspend && (4 != gptHWCFG->m_val.bTouchType)) {
#ifndef DIGITIZER_TEST
		gpio_direction_output(TOUCH_PWR, 0);
		gpio_direction_output(TOUCH_EN, 1);
		gpio_direction_output(TOUCH_RST, 1);
#endif
		gpio_free(5*32+18);
		gpio_free(5*32+19);
		mxc_iomux_v3_setup_pad(MX50_PAD_I2C1_SCL__I2C1_SCL);
		mxc_iomux_v3_setup_pad(MX50_PAD_I2C1_SDA__I2C1_SDA);
		mdelay (50);
		gpio_direction_input(C_TOUCH_INT);
		enable_irq(gpio_to_irq(C_TOUCH_INT));
		if (2 == check_hardware_name()) {
			gpio_direction_input(E50602_G_SENSOR_INT);
			enable_irq(gpio_to_irq(E50602_G_SENSOR_INT));
		}
		else {
			gpio_direction_input(G_SENSOR_INT);
			enable_irq(gpio_to_irq(G_SENSOR_INT));
		}
	}
	
#if 1	
	if ((6 == check_hardware_name()) || (2 == check_hardware_name())) 		// E60632 || E50602
		g_power_key_pressed = (gpio_get_value (GPIO_PWR_SW))?1:0;	// POWER key
	else
		g_power_key_pressed = (gpio_get_value (GPIO_PWR_SW))?0:1;	// POWER key
#endif

	// turn on audio power
	gpio_free(5*32+25);
	mxc_iomux_v3_setup_pad(MX50_PAD_PWM2__PWMO);
//	gpio_direction_output(GPIO_AUDIO_PWR, 0);
	gpio_free(5*32+20);
	gpio_free(5*32+21);

	if (LED_conitnuous)
   		wake_up_interruptible(&LED_freeze_WaitQueue);
   	else {
		ntx_led_blink (3, red_led_period);
		ntx_led_blink (4, green_led_period);
		ntx_led_blink (5, blue_led_period);
	}
}

void ntx_gpio_touch_reset (void)
{
	gpio_direction_output(TOUCH_RST, 0);
	msleep (10);
	gpio_direction_output(TOUCH_RST, 1);
}

void ntx_msp430_i2c_force_release (void)
{
	int retryCnt=20;
	mxc_iomux_v3_setup_pad(MX50_PAD_I2C3_SDA__GPIO_6_23);
	gpio_request(GPIO_I2C3_SDA, "i2c3_sda");
	gpio_direction_input (GPIO_I2C3_SDA);
	mxc_iomux_v3_setup_pad(MX50_PAD_I2C3_SCL__GPIO_6_22);
	gpio_request(GPIO_I2C3_SCL, "i2c3_scl");
	gpio_direction_output (GPIO_I2C3_SCL, 1);
	// send clock out until i2c SDA released.
	while (retryCnt-- && !gpio_get_value (GPIO_I2C3_SDA)) {
		gpio_set_value (GPIO_I2C3_SCL,1);
		udelay (5);
		gpio_set_value (GPIO_I2C3_SCL,0);
		schedule_timeout (1);
//		udelay (5);
	}
	// simulate i2c stop signal
	gpio_direction_output (GPIO_I2C3_SDA,0);
	gpio_free(GPIO_I2C3_SCL);
	mxc_iomux_v3_setup_pad(MX50_PAD_I2C3_SCL__I2C3_SCL);
	udelay (2);
	gpio_free(GPIO_I2C3_SDA);
	mxc_iomux_v3_setup_pad(MX50_PAD_I2C3_SDA__I2C3_SDA);
}

void ntx_machine_restart(char mode, const char *cmd)
{
	while (1) {
		printk("Kernel---System reset ---\n");
		gKeepPowerAlive = 0;
		msp430_reset();
	    sleep_on_timeout(&Reset_WaitQueue, 14*HZ/10);
	}
}

static int __init initDriver(void)
{
	int ret;
        
	ret = misc_register(&driverDevice);
	if (ret < 0) {
		printk("pvi_io: can't get major number\n");
		return ret;
	}

    gpio_initials();


	#ifdef NTX_GPIO_KEYS //[
	
	
	if(gptHWCFG->m_val.bHallSensor) {
		ntx_gpio_buttons[gi_ntx_gpio_buttons_total].code=KEY_F1;
		ntx_gpio_buttons[gi_ntx_gpio_buttons_total].gpio=HALLSENSOR_KEY;
		ntx_gpio_buttons[gi_ntx_gpio_buttons_total].active_low=1;
		ntx_gpio_buttons[gi_ntx_gpio_buttons_total].type=EV_KEY;
		ntx_gpio_buttons[gi_ntx_gpio_buttons_total].wakeup=1;
		ntx_gpio_buttons[gi_ntx_gpio_buttons_total].debounce_interval=500;

		++gi_ntx_gpio_buttons_total;
		//ASSERT(gi_ntx_gpio_buttons_total<NTX_GPIO_KEYS_MAX);
	}

	ntx_gpio_key_data.nbuttons = gi_ntx_gpio_buttons_total;
	if(gi_ntx_gpio_buttons_total>0) {
		mxc_register_device(&ntx_gpio_key_device, &ntx_gpio_key_data);
	}
	#endif //] NTX_GPIO_KEYS
    
	//start a kernel thread;
	ret = kernel_thread(LED_Thread,NULL,CLONE_KERNEL);
	if(ret < 0){
	    printk("LED thread creat error\n");
	}

	////////////////////////
	green_led_timer.function = green_led_blink_func;
	init_timer(&green_led_timer);
	blue_led_timer.function = blue_led_blink_func;
	init_timer(&blue_led_timer);
	red_led_timer.function = red_led_blink_func;
	init_timer(&red_led_timer);
	
	arm_pm_restart = ntx_machine_restart;

	return 0;
}
static void __exit exitDriver(void) {
	misc_deregister(&driverDevice);
}
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joe");
MODULE_VERSION("2007-9-20");
MODULE_DESCRIPTION ("PVI_IO driver");
module_init(initDriver);
module_exit(exitDriver);
