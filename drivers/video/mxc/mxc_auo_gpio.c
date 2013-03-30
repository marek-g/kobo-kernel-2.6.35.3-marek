#include <linux/module.h>
#include <linux/kernel.h>



#include <linux/gpio.h>


#define GDEBUG 10
#include <linux/gallen_dbg.h>


#include "auo_k1901.h"


/////////////////////////////////////////////////////////////////////
// auo hardware ...

static inline int _auo_io_init(void)
{
	int iRet = 0;
	// auo CS,HWE,HRD,BUSY_N,D/C,RST,DATA pin initailize 
	
	return iRet;
}

static inline void _auo_io_SLPN_set(int iIsHigh)
{
}

static inline void _auo_io_RSTN_set(int iIsHigh)
{
}

static inline void _auo_io_CS_set(int iIsHigh)
{
}

static inline void _auo_io_DC_set(int iIsCommand)
{
}

static inline void _auo_io_RW_set(int iIsRead)
{
}

static inline void _auo_io_Data_set(unsigned long dwData)
{
}

static inline unsigned long _auo_io_Data_get(void)
{
	unsigned long dwData;
	
	return dwData ;
}

static inline int _auo_io_BUSYN_get(void)
{
	int iRet;
	return iRet;
}



//////////////////////////////////////////////////////
// AUO K1901 low level ...

static int _k1901_power_on(void)
{
}

static int _k1901_power_off(void)
{
}



static int _k1901_i80_sned_command(unsigned short I_wCmd,unsigned short *I_pwParam,unsigned long I_dwParamCnt)
{
}

static int _k1901_i80_send_data(unsigned short *I_pwData,unsigned long I_dwDataCnt)
{
}

static int _k1901_i80_receive_data(unsigned short *I_pwData,unsigned long I_dwDataCnt)
{
}

//////////////////////////////////////////////////////
// AUO K1901 interface for kernel drivers ... 

#define K1901_CMD_READ_VERSION	0x4000



int K1901_read_version(K1901_VERSION *O_ptK1901_version)
{
	int iRet = 0;
	
	int iChk;
	
	iChk = _k1901_i80_sned_command(K1901_CMD_READ_VERSION,0,0);
	if(iChk>=0) {
		iChk = _k1901_i80_receive_data((unsigned short *)O_ptK1901_version,sizeof(K1901_VERSION));
		DBG_MSG("%s : K1901 version (%d): \n",__FUNCTION__,iChk);
		DBG_MSG("\twTemperature=0x%x\n",O_ptK1901_version->wTemprature);
		DBG_MSG("\twLOT number=0x%x\n",O_ptK1901_version->wLotNumber);
		DBG_MSG("\tbSample_Version=0x%x\n",O_ptK1901_version->bSample_Version);
		DBG_MSG("\tbTCON_Version=0x%x\n",O_ptK1901_version->bTCON_Version);
		DBG_MSG("\tbP_WF_Version=0x%x\n",O_ptK1901_version->bP_WF_Version);
	}
	else {
		ERR_MSG("%s : READ_VERSION command fail !\n",__FUNCTION__);
		iRet = iChk;
	}
	
	return iRet;
}



