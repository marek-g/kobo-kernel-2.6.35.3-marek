
#ifndef __GPIOFN_H //[
#define __GPIOFN_H

#include <mach/iomux-mx50.h>
#include <linux/timer.h>
#include <linux/list.h>

typedef int (*fn_gpio)(int I_iGPIO_val,unsigned uGPIO);

typedef struct tagGPIODATA{
	//-------------------------------
	// public :
	fn_gpio pfnGPIO;
	fn_gpio pfnGPIO_INT;
	unsigned uGPIO;
	char *szName;
	iomux_v3_cfg_t tPADCtrl;
	unsigned int uiIRQType;
	int iWakeup; // 1->enable wakeup device , 0-> disable wakeup device .
	unsigned long dwDebounceTicks;
	int iActive; 
	//--------------------------------
	// private :
	struct work_struct tWork;
	struct workqueue_struct *ptWQ;
	struct timer_list tTimerDebounce;
	unsigned long dwCurrentDebounceTicks;
	//struct list_head list;
	int iLastGPIOVal;
	int iCurrentGPIOVal;
}GPIODATA;


int gpiofn_init(void);
int gpiofn_release(void);


int gpiofn_register(GPIODATA *I_ptGPIO_Data);
int gpiofn_unregister(GPIODATA *I_ptGPIO_Data);


#endif //] __GPIOFN_H
