#ifndef LK_TPS65185_H//[
#define LK_TPS65185_H


#define TPS65185_RET_ALLPOWEROFF		(2) // all power is turned off .
#define TPS65185_RET_ALLPOWERGOOD		(1) // all power is good .
#define TPS65185_RET_SUCCESS			(0) // all right .
#define TPS65185_RET_PARAMERR			(-1) // parameter error !
#define TPS65185_RET_I2CCHN_NOTFOUND	(-2) // i2c adapter not found !
#define TPS65185_RET_NEWDEVFAIL			(-3) // register i2c client fail !!
#define TPS65185_RET_CHIP_NOTFOUND		(-4) // hardware chip not found !
#define TPS65185_RET_I2CTRANS_ERR		(-5) // i2c trans error !
#define TPS65185_RET_INITNOTYET			(-6) // should init first !
#define TPS65185_RET_IOCONFIG_ERR		(-7) // gpio configuration error !
#define TPS65185_RET_REGREADFAIL		(-8) // register read fail !
#define TPS65185_RET_POWERNOTGOOD		(-9) // .
#define TPS65185_RET_TIMEOUT			(-10) // .
#define TPS65185_RET_VCOMWRFAIL			(-11) // VCOM write fail .


int tps65185_release(void);

#define EPDTIMING_V110		1
#define EPDTIMING_V220		2
int tps65185_init(int iPort,int iEPDTimingType);

int tps65185_get_temperature(int *O_piTemperature);

int tps65185_is_panel_poweron(void);

int tps65185_wait_panel_poweron(void);
int tps65185_wait_panel_poweroff(void);

////////////////////////////////////////////////////////
// definitions for chg_mode() ...
#define TPS65185_MODE_UNKOWN		0x00000000
#define TPS65185_MODE_ACTIVE		0x00000001
#define TPS65185_MODE_SLEEP			0x00000002
#define TPS65185_MODE_STANDBY		0x00000004
int tps65185_chg_mode(unsigned long *IO_pdwMode,int iIsWaitPwrOff);

int tps65185_vcom_set(int I_iVCOM_mv,int iIsWriteToFlash);
int tps65185_vcom_get(int *O_piVCOM_mv);
int tps65185_vcom_kickback_measurement(int *O_piVCOM_mv);
void tps65185_suspend(void);
void tps65185_resume(void);

#endif //]LK_LM75_H
