#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/iomux-mx50.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/input.h>


#include "gpiofn.h"
#include "ntx_hwconfig.h"

#define GDEBUG 0
#include <linux/gallen_dbg.h>



extern volatile NTX_HWCONFIG *gptHWCFG;
extern void mxc_kpp_report_key(int isDown,__u16 wKeyCode);
extern int gIsCustomerUi;
extern void power_key_int_function(void);
extern void mxc_kpp_report_power(int isDown);

#define GPIO_tle4913Q 	(4*32+25) /* GPIO_5_25 */
static int tle4913Q_func(int iGPIOVal,unsigned uGPIO);



static GPIODATA gtTLE4913_GPIO_data = {
	.pfnGPIO_INT = tle4913Q_func,
	.uGPIO = GPIO_tle4913Q,
	.szName = "TLE4913",
	.tPADCtrl = MX50_PAD_SD3_D5__GPIO_5_25,
	.uiIRQType = IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
	.iWakeup = 1,
	.dwDebounceTicks = 1,
	.iActive = 0 ,
};




static int tle4913Q_func(int iGPIOVal,unsigned uGPIO)
{
	DBG0_MSG("%s(%d): gpio%u,val=%d\n",__FILE__,__LINE__,uGPIO,iGPIOVal);
	
	if(gIsCustomerUi) {
		mxc_kpp_report_key(iGPIOVal?0:1,KEY_F1);
	}
	else {
		mxc_kpp_report_key(iGPIOVal?0:1,KEY_H);
	}
	
	return 0;
}



void tle4913_init(void)
{
	if(gptHWCFG&&1==gptHWCFG->m_val.bHallSensor) {
		gpiofn_register(&gtTLE4913_GPIO_data);
	}
	else {
		WARNING_MSG("%s : HallSensor not exist !\n",__FUNCTION__);
	}
}

void tle4913_release(void)
{
	if(gptHWCFG&&1==gptHWCFG->m_val.bHallSensor) {
		gpiofn_unregister(&gtTLE4913_GPIO_data);
	}
}

