

#include <linux/kernel.h>
//#include <linux/config.h>

#include <linux/i2c.h>

#define GDEBUG	1
#include <linux/gallen_dbg.h>


#include "lk_lm75.h"


// LM75 command byte definitions ...

#define REG_UNKOWN_VAL	0xcc

#define TOTAL_CHIPS	1

//MODULE_LICENSE("GPL");

static struct i2c_adapter *gpI2C_adapter = 0;
static struct i2c_client *gpI2C_clientA[TOTAL_CHIPS] = {0,};


static struct i2c_board_info gtLM75_BIA[TOTAL_CHIPS] = {
	{
	 .type = "lm75-1",
	 .addr = 0x48,
	 .platform_data = NULL,
	 },
};

typedef struct tagLM75_data{
	int iCurrent_temprature;
	unsigned short wTempratureData;
	//int iLast_temprature;
}LM75_data;

static LM75_data gtLM75_DataA[TOTAL_CHIPS] = {
	{-1,},
};


// auto detect lm75 .
// parameters :
// 	iPort : i2c channel in system (from 1~3) .
int lm75_init(int iPort)
{
	int iRet = LM75_RET_SUCCESS;
	int iChipIdx;
	
	GALLEN_DBGLOCAL_BEGIN();
	
	printk ("%s(%d) \n",__func__,iPort);
	gpI2C_adapter = i2c_get_adapter(iPort-1);//
	if( NULL == gpI2C_adapter )
	{
		GALLEN_DBGLOCAL_ESC();
		printk ("%s() LM75_RET_I2CCHN_NOTFOUND\n",__func__);
		return LM75_RET_I2CCHN_NOTFOUND;
	}
	
	
	for(iChipIdx=0;iChipIdx<TOTAL_CHIPS;iChipIdx++) {
		gpI2C_clientA[iChipIdx] = i2c_new_device(gpI2C_adapter, &gtLM75_BIA[iChipIdx]);
		if(NULL == gpI2C_clientA[iChipIdx]) {
			lm75_release();
			GALLEN_DBGLOCAL_ESC();
			printk ("%s() LM75_RET_NEWDEVFAIL\n",__func__);
			return LM75_RET_NEWDEVFAIL;
		}
		printk("client%d ,addr=0x%x,name=%s\n",iChipIdx,
			gpI2C_clientA[iChipIdx]->addr,gpI2C_clientA[iChipIdx]->name);
	}
	
	//lm75_get_temperature(0,0);// test .
	
	GALLEN_DBGLOCAL_END();
	return iRet;
}



int lm75_release(void)
{
	int iRet = LM75_RET_SUCCESS;
	int iChipIdx ,i;
	
	printk("%s(%d):%s()\n",__FILE__,__LINE__,__FUNCTION__);
	for(iChipIdx=0;iChipIdx<TOTAL_CHIPS;iChipIdx++) {
		if(gpI2C_clientA[iChipIdx]) {
			i2c_unregister_device(gpI2C_clientA[iChipIdx]);
			gpI2C_clientA[iChipIdx] = NULL;
		}
		gtLM75_DataA[iChipIdx].iCurrent_temprature = -1;
		//gtLM75_DataA[iChipIdx].iLast_temprature = -1;
	}
	
	gpI2C_adapter = NULL;
	
	return iRet;
}




int lm75_get_temperature(int iChipIdx,int *O_piTemperature)
{
	int iRet = LM75_RET_SUCCESS;
	int iChk;
	
	unsigned short wTemp;
	unsigned char bA[2] = {0x00}; // pointer of temprature .
	
	unsigned char bTemp;
	int iTemp;
	
	//printk("%s()\n",__FUNCTION__);
	
	if( NULL == gpI2C_adapter ) {
		ERR_MSG("%s(%d):%s cannot get temp without init .!!\n",__FILE__,__LINE__,__FUNCTION__);
		return LM75_RET_INITNOTYET;
	}
	
	iChk = i2c_master_send(gpI2C_clientA[iChipIdx], (const char *)bA, 1);

	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_send fail !\n",__FILE__,__LINE__,__FUNCTION__);
		return LM75_RET_I2CTRANS_ERR;
	}
	
	iChk = i2c_master_recv(gpI2C_clientA[iChipIdx], bA, 2);
	

	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_recv fail !\n",__FILE__,__LINE__,__FUNCTION__);
		return LM75_RET_I2CTRANS_ERR;
	}
	wTemp = bA[0]<<3|bA[1]>>5;
	gtLM75_DataA[iChipIdx].wTempratureData = wTemp;
	if(bA[0]&0x80) {
		// negative .
		bTemp=(~bA[0])+1;
		iTemp = bTemp;
		iTemp = (~iTemp)+1;
	}
	else {
		// positive .
		iTemp = (int)(bA[0]);
	}	
	gtLM75_DataA[iChipIdx].iCurrent_temprature = iTemp;
	DBG_MSG("lm75 temprature data = 0x%x%x,%d\n",bA[0],bA[1],gtLM75_DataA[iChipIdx].iCurrent_temprature);
	
	//gtLM75_DataA[iChipIdx].iCurrent_temprature = bA[0];
	if(O_piTemperature) {
		*O_piTemperature = gtLM75_DataA[iChipIdx].iCurrent_temprature;
	}
	
	return iRet;
}

void lm75_suspend(void)
{
	//int iRet = LM75_RET_SUCCESS;
	int iChk;
	
	//unsigned short wTemp;
	unsigned char bA[2] = {01,01}; 
	
	
	if( NULL == gpI2C_adapter ) {
		ERR_MSG("%s(%d):%s cannot get temp without init .!!\n",__FILE__,__LINE__,__FUNCTION__);
		return ;
	}
	
	iChk = i2c_master_send(gpI2C_clientA[0], (const char *)bA, 2);

	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_send fail !\n",__FILE__,__LINE__,__FUNCTION__);
		return;
	}
}

void lm75_resume(void)
{
	//int iRet = LM75_RET_SUCCESS;
	int iChk;
	
	//unsigned short wTemp;
	unsigned char bA[2] = {01,0}; 
	
	
	if( NULL == gpI2C_adapter ) {
		ERR_MSG("%s(%d):%s cannot get temp without init .!!\n",__FILE__,__LINE__,__FUNCTION__);
		return ;
	}
	
	iChk = i2c_master_send(gpI2C_clientA[0], (const char *)bA, 2);

	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_send fail !\n",__FILE__,__LINE__,__FUNCTION__);
		return;
	}
}

//EXPORT_SYMBOL(lm75_init);
//EXPORT_SYMBOL(lm75_release);
//EXPORT_SYMBOL(lm75_get_temperature);

