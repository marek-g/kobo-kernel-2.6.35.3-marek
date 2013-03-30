
#if defined(__UBOOT__) //[
	#include <common.h>
#elif defined(__KERNEL__) //][
	#include <linux/kernel.h>
	#include <linux/fs.h>
	#include <linux/syscalls.h>
#else //][!__KERNEL__
	#include <stdio.h>
	#include <unistd.h>
	#include <fcntl.h>
	#include <string.h>
	#include <assert.h>
	#include <sys/types.h>
#endif //] __KERNEL__





#include "ntx_hwconfig.h"




#define _DEBUG

#ifdef _DEBUG//[
	#ifdef __UBOOT__//[
	// uboot compile ...
	
	#define dbgENTER()	printf("%s(%d) : Enter %s\n",__FILE__,__LINE__,__FUNCTION__)
	#define dbgLEAVE()  printf("%s(%d) : Leave %s\n",__FILE__,__LINE__,__FUNCTION__)
	#define DBG_MSG(fmt,args...)	printf(fmt,##args)
	#define	ERR_MSG(fmt,args...)	printf(fmt,##args)
	#define WARNING_MSG(fmt,args...)				printf(fmt,##args)
	#define ASSERT(x)				assert(x)	
	#define ssize_t		int
	#define O_RDONLY 	0
	#define O_TRUNC		0
	#define O_RDWR		0
	#define O_CREAT		0
	#define SEEK_SET	0
	#elif defined(__KERNEL__) //][
	// kernel compile ...
	
	#define dbgENTER()					
	#define dbgLEAVE()					
	#define DBG_MSG(fmt,args...)					
	#define	ERR_MSG(fmt,args...)					printk(fmt,##args)
	#define WARNING_MSG(fmt,args...)			printk(fmt,##args)
	#ifdef CONFIG_KERNEL_ASSERTS//[
		/* kgdb stuff */
		#define assert(p) KERNEL_ASSERT(#p, p)
	#else//][!CONFIG_KERNEL_ASSERTS
		#define assert(p) do {  \
			if (!(p)) {     \
				printk(KERN_CRIT "BUG at %s:%d assert(%s)\n",   \
					__FILE__, __LINE__, #p);                 \
					BUG();  \
			}               \
		} while (0)
	#endif//] CONFIG_KERNEL_ASSERTS
	#define ASSERT(x)				assert(x)
	
	#else //][! __KERNEL__
	// linux ap level ...
	
	#define dbgENTER()	fprintf(stderr,"%s(%d) : Enter %s\n",__FILE__,__LINE__,__FUNCTION__)
	#define dbgLEAVE()  fprintf(stderr,"%s(%d) : Leave %s\n",__FILE__,__LINE__,__FUNCTION__)
	#define DBG_MSG(fmt,args...)	fprintf(stderr,fmt,##args)
	#define	ERR_MSG(fmt,args...)	fprintf(stderr,fmt,##args)
	#define WARNING_MSG(fmt,args...)				fprintf(stderr,fmt,##args)
	#define ASSERT(x)				assert(x)
	
	#endif //] __KERNEL__
#else//][ !_DEBUG
	#define dbgENTER()					
	#define dbgLEAVE()					
	#define DBG_MSG(fmt,args...)					
	#define	ERR_MSG(fmt,args...)					
	#define WARNING_MSG(fmt,args...)			
	#define ASSERT(x)							
#endif//] _DEBUG


const char gszNtxHwCfgMagic[]="HW CONFIG ";// hw config tool magic .
const char gszNtxHwCfgVersion[]="v1.3"; // hw config tool version .


// field values table ...
const char * gszPCBA[]={"E60800","E60810","E60820","E90800","E90810","E60830","E60850",
"E50800","E50810","E60860","E60MT2","E60M10","E60610","E60M00","E60M30","E60620","E60630","E60640","E50600",
"E60680","E60610C","E60610D","E606A0","E60670","E606B0","E50620","Q70Q00","E50610","E606C0","E606D0","E606E0"};
const char * gszKeyPadA[]={"MX357","MX357+Wheel","MX357+Joystick","MX35-5inch","1Key","E60M10",
"E60M10+Touch","E60620","E60630","E60640","E606A0","FL_Key","NO_Key"};
/*
 *
 * 1Key : keypad layout only home key .
 * FL_Key : keypad layout only FrontLight Key .
 * NO_Key : no any key .
 *  
 */
const char * gszAudioCodecA[]={"No","ALC5623"};//
const char * gszAudioAmpA[]={"No","TPA2016"};//
const char * gszWifiA[]={"No","AW-GH381","AW-GH321","GB9619","PW621","WC160","WC121","WC121A2"};//
const char * gszBTA[]={"No","AW-GH381","AW-GH105"};//
const char * gszMobileA[]={"No","Moto 3G"};//
const char * gszTouchCtrlA[]={"No","TSC2004","Wacom Digitizer","Watop Digitizer","AUO-TP2","neonode","PVI","ITE","neonode_v2"};//touch controller .
const char * gszTouchTypeA[]={"No","R-Type","Digitizer","C-Type","IR-Type"};//
const char * gszDisplayCtrlA[]={"S1D13521","S1D13522","K1900","M166E","MX508","K1901","MX508+TPS65185"};
const char * gszDisplayPanelA[]={"6\" Left EPD","6\" Right EPD","9\" Right EPD","5\" Left EPD","5\" Right EPD","6\" Top EPD","6\" Bottom EPD","5\" Top EPD","5\" Bottom EPD",\
	"6.8\" Top EPD","6.8\" Bottom EPD"};
const char * gszRSensorA[]={"No","Rotary Encoder","G Sensor"};//
const char * gszMicroPA[]={"MSP430"};//
const char * gszCustomerA[]={"0","1","2","3","4","5","6","7","8",\
	"9","10","11","12","13","14","15","16","17","18","19","20","21","22","23"};//
const char * gszBatteryA[]={"1000mA","1500mA"};//
const char * gszLedA[]={"TYPE1"};//
const char * gszRamSizeA[]={"128MB","64MB","256MB","512MB","1GB","2GB","4GB"};// ram size 
const char * gszIFlashA[]={"Micro SD","NAND Flash"}; // internal flash type .
const char * gszExternalMemA[]={"No","SD","Micro SD"};// external sd type .
const char * gszRootFsTypeA[]={"Ext2","Ext3","Ext4","Vfat"};// root fs type .
	/*
	TYPE1 : 
		1->rootfs
		2->user vfat 
	TYPE2 : 
		1->rootfs (recovery mode)
		2->rootfs
		3->user vfat
	TYPE3 :
		1->rootfs
		2->rootfs (recovery mode)
		3->user vfat
	TYPE4 :
		1->user vfat
		2->rootfs
	TYPE5 :
		1->rootfs
	TYPE6 :
		1->rootfs
		2->user vfat
		3->rootfs (recovery mode)
	TYPE7 :
		1->user vfat
		2->rootfs
		3->rootfs (recovery mode)
	TYPE8 :
		1->user vfat
		2->system
		4->rootfs (recovery mode)
		5/3->data
		6/3->cache
	*/
const char * gszSysPartTypeA[]={"TYPE1","TYPE2","TYPE3","TYPE4","TYPE5","TYPE6","TYPE7","TYPE8","TYPE9","TYPE10"};// system partition type .
const char * gszCPUA[]={"mx35","m166e","mx50","x86","mx6"}; // platform CPU .
const char * gszUIStyleA[]={"Ebrmain","Customer UI","Android"};// UI Style .
const char * gszRAMTypeA[]={"MDDR","DDR2","K4X2G323PC","K4X2G323PD"};// Ram Type .
const char * gszUIConfigA[]={"Normal","Normal2","AD"};// UI Config .
const char * gszDisplayResolutionA[]={"800x600","1024x758","1024x768","1440x1080"};// Display resolution .
const char * gszFrontLightA[]={"No","TABLE0"};// Front Light .
const char * gszCPUFreqA[]={"NC","800M","1G","1.2G"};// CPU Freqency  .
const char * gszHallSensorA[]={"No","TLE4913"};// Hall Sensor .



#define _TOTOAL_HWCONFIG_FIELDS 	(sizeof(gtHwConfigFields)/sizeof(gtHwConfigFields[0]))

static HwConfigField gtHwConfigFields[] = {
	{"v0.1","PCB",sizeof(gszPCBA)/sizeof(gszPCBA[0]),
		(char **)gszPCBA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","KeyPad",sizeof(gszKeyPadA)/sizeof(gszKeyPadA[0]),
		(char **)gszKeyPadA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","AudioCodec",sizeof(gszAudioCodecA)/sizeof(gszAudioCodecA[0]),
		(char **)gszAudioCodecA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","AudioAmp",sizeof(gszAudioAmpA)/sizeof(gszAudioAmpA[0]),
		(char **)gszAudioAmpA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","Wifi",sizeof(gszWifiA)/sizeof(gszWifiA[0]),
		(char **)gszWifiA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","BT",sizeof(gszBTA)/sizeof(gszBTA[0]),
		(char **)gszBTA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","Mobile",sizeof(gszMobileA)/sizeof(gszMobileA[0]),
		(char **)gszMobileA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","TouchCtrl",sizeof(gszTouchCtrlA)/sizeof(gszTouchCtrlA[0]),
		(char **)gszTouchCtrlA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","TouchType",sizeof(gszTouchTypeA)/sizeof(gszTouchTypeA[0]),
		(char **)gszTouchTypeA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","DisplayCtrl",sizeof(gszDisplayCtrlA)/sizeof(gszDisplayCtrlA[0]),
		(char **)gszDisplayCtrlA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","DisplayPanel",sizeof(gszDisplayPanelA)/sizeof(gszDisplayPanelA[0]),
		(char **)gszDisplayPanelA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","RSensor",sizeof(gszRSensorA)/sizeof(gszRSensorA[0]),
		(char **)gszRSensorA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","MicroP",sizeof(gszMicroPA)/sizeof(gszMicroPA[0]),
		(char **)gszMicroPA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","Customer",sizeof(gszCustomerA)/sizeof(gszCustomerA[0]),
		(char **)gszCustomerA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_SW},
	{"v0.1","Battery",sizeof(gszBatteryA)/sizeof(gszBatteryA[0]),
		(char **)gszBatteryA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","Led",sizeof(gszLedA)/sizeof(gszLedA[0]),
		(char **)gszLedA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","RamSize",sizeof(gszRamSizeA)/sizeof(gszRamSizeA[0]),
		(char **)gszRamSizeA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","IFlash",sizeof(gszIFlashA)/sizeof(gszIFlashA[0]),
		(char **)gszIFlashA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.1","ExternalMem",sizeof(gszExternalMemA)/sizeof(gszExternalMemA[0]),
		(char **)gszExternalMemA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.2","RootFsType",sizeof(gszRootFsTypeA)/sizeof(gszRootFsTypeA[0]),
		(char **)gszRootFsTypeA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_SW},
	{"v0.3","SysPartType",sizeof(gszSysPartTypeA)/sizeof(gszSysPartTypeA[0]),
		(char **)gszSysPartTypeA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_SW},
	{"v0.4","ProgressXHiByte",0,0,FIELD_TYPE_BYTE,FIELD_FLAGS_SW},
	{"v0.4","ProgressXLoByte",0,0,FIELD_TYPE_BYTE,FIELD_FLAGS_SW},
	{"v0.4","ProgressYHiByte",0,0,FIELD_TYPE_BYTE,FIELD_FLAGS_SW},
	{"v0.4","ProgressYLoByte",0,0,FIELD_TYPE_BYTE,FIELD_FLAGS_SW},
	{"v0.4","ProgressCnts",0,0,FIELD_TYPE_BYTE,FIELD_FLAGS_SW},
	{"v0.5","ContentType",0,0,FIELD_TYPE_BYTE,FIELD_FLAGS_SW},
	{"v0.6","CPU",sizeof(gszCPUA)/sizeof(gszCPUA[0]),
		(char **)gszCPUA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.7","UIStyle",sizeof(gszUIStyleA)/sizeof(gszUIStyleA[0]),
		(char **)gszUIStyleA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_SW},
	{"v0.8","RAMType",sizeof(gszRAMTypeA)/sizeof(gszRAMTypeA[0]),
		(char **)gszRAMTypeA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v0.9","UIConfig",sizeof(gszUIConfigA)/sizeof(gszUIConfigA[0]),
		(char **)gszUIConfigA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_SW},
	{"v1.0","DisplayResolution",sizeof(gszDisplayResolutionA)/sizeof(gszDisplayResolutionA[0]),
		(char **)gszDisplayResolutionA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v1.1","FrontLight",sizeof(gszFrontLightA)/sizeof(gszFrontLightA[0]),
		(char **)gszFrontLightA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v1.2","CPUFreq",sizeof(gszCPUFreqA)/sizeof(gszCPUFreqA[0]),
		(char **)gszCPUFreqA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
	{"v1.3","HallSensor",sizeof(gszHallSensorA)/sizeof(gszHallSensorA[0]),
		(char **)gszHallSensorA,FIELD_TYPE_IDXSTR,FIELD_FLAGS_HW},
};

static NTX_HWCONFIG _gtNtxHwCfg,*_gptNtxHwCfg=&_gtNtxHwCfg;


//static int giIsForceIgnoreVersion=0; 

static int my_strcmp(const char *szA,const char *szB)
{
	char *pcA=(char *)szA,*pcB=(char *)szB;
	int iRet = -1;
	
	ASSERT(pcA);
	ASSERT(pcB);
	
	while( *pcA==*pcB ) {
		if(*pcA == '\0') {
			iRet = 0 ;
			break;
		}
		++pcA;
		++pcB;		
	}
	
	return iRet;
}


static int my_open(const char *I_pszFilename,int I_iFlags)
{
	int iFd = -1;
#ifdef __KERNEL__//[
	iFd = sys_open(I_pszFilename,I_iFlags,0);
#else//][!__KERNEL__
	iFd = open(I_pszFilename,I_iFlags);
#endif //] __KERNEL__
	return iFd;
}

static int my_close(int I_iFd)
{
	int iRet;
#if defined(__UBOOT__) //[
	iRet = -1;
#elif defined(__KERNEL__)//[
	iRet = sys_close(I_iFd);
#else//][!__KERNEL__
	iRet = close(I_iFd);
#endif //] __KERNEL__
	return iRet;
}

static int my_lseek(int I_iFd,unsigned int I_uiOffset,unsigned int I_uiOrigin)
{
	int iRet;
#if defined(__UBOOT__) //[
	iRet = -1;
#elif defined(__KERNEL__)//[
	iRet = sys_lseek(I_iFd,I_uiOffset,I_uiOrigin);
#else//][!__KERNEL__
	iRet = lseek(I_iFd,I_uiOffset,I_uiOrigin);
#endif //] __KERNEL__
	return iRet;
}

static ssize_t my_read(int I_iFd,unsigned char *O_pbBuf,unsigned int I_uiBufSize)
{
	ssize_t tRet;
#if defined(__UBOOT__) //[
	tRet = -1;
#elif defined(__KERNEL__)//[
	tRet = sys_read(I_iFd,O_pbBuf,I_uiBufSize);
#else//][!__KERNEL__
	tRet = read(I_iFd,O_pbBuf,I_uiBufSize);
#endif //] __KERNEL__
	return tRet;
}

static ssize_t my_write(int I_iFd,unsigned char *I_pbBuf,unsigned int I_uiBufSize)
{
	ssize_t tRet;
#if defined(__UBOOT__) //[
	tRet = -1;
#elif defined(__KERNEL__)//[
	tRet = sys_write(I_iFd,I_pbBuf,I_uiBufSize);
#else//][!__KERNEL__
	tRet = write(I_iFd,I_pbBuf,I_uiBufSize);
#endif //] __KERNEL__
	return tRet;
}


static int _compare_hdrver_fldver(NTX_HWCONFIG *pHdr,int iFldIdx)
{
	int iRet ;
	ASSERT(NtxHwCfg_ChkCfgHeaderEx(pHdr,1)>=0);
	
	if(pHdr->m_hdr.cVersionNameA[1]>gtHwConfigFields[iFldIdx].szVersion[1]) {
		iRet = +2;
	}
	else if(pHdr->m_hdr.cVersionNameA[1]<gtHwConfigFields[iFldIdx].szVersion[1]) {
		iRet = -2;
	}
	else {
		if(pHdr->m_hdr.cVersionNameA[3]>gtHwConfigFields[iFldIdx].szVersion[3]) {
			iRet = 1;
		}
		else if(pHdr->m_hdr.cVersionNameA[3]<gtHwConfigFields[iFldIdx].szVersion[3]) {
			iRet = -1;
		}
		else {
			iRet = 0;
		}
	}
	return iRet;
}



NTX_HWCONFIG *NtxHwCfg_Load(const char *szFileName,int iIsSeek)
{
	NTX_HWCONFIG *ptRet = 0;
	char *pszFileName = (char *)szFileName;
	
	if(0==szFileName) {
		#ifdef _X86_//[
		return 0;
		#else //][!_X86_
		pszFileName = "/dev/mmcblk0";
		iIsSeek = 1;
		#endif //]_X86_
	}
	
	{
		int iFd = -1;
		ssize_t tChk;
		
		iFd = my_open(pszFileName,O_RDONLY);
		if(iFd>=0) {
			if(iIsSeek) {
				iIsSeek = SYSHWCONFIG_SEEKSIZE;
			}
			else {
				iIsSeek = 0;
			}
				
			my_lseek(iFd,(unsigned int)iIsSeek,SEEK_SET);
			tChk = my_read(iFd,(unsigned char *)_gptNtxHwCfg,sizeof(_gtNtxHwCfg));
			if((int)tChk==sizeof(_gtNtxHwCfg)) {
				if(NtxHwCfg_ChkCfgHeaderEx(_gptNtxHwCfg,1)>=0) {
					ptRet = _gptNtxHwCfg;
				}
			}
			my_close(iFd);iFd=-1;
		}
		else {
			ERR_MSG("%s : File \"%s\" open fail !\n",__FUNCTION__,pszFileName);
		}
		
	}
	
	return ptRet;
}

int NtxHwCfg_Save(const char *szFileName,int iIsSeek)
{
	int iRet;
	char *pszFileName = (char *)szFileName;
	
	if(0==szFileName) {
		#ifdef _X86_//[
		return 0;
		#else //][!_X86_
		pszFileName = "/dev/mmcblk0";
		iIsSeek = 1;
		#endif //]_X86_
	}
	
	{
		int iFd = -1;
		ssize_t tChk;
		
		iFd = my_open(pszFileName,O_RDWR|O_TRUNC|O_CREAT);
		if(iFd>=0) {
			if(iIsSeek) {
				iIsSeek = SYSHWCONFIG_SEEKSIZE;
			}
			else {
				iIsSeek = 0;
			}
				
			my_lseek(iFd,(unsigned int)iIsSeek,SEEK_SET);
			tChk = my_write(iFd,(unsigned char *)_gptNtxHwCfg,sizeof(_gtNtxHwCfg));
			if((int)tChk==sizeof(_gtNtxHwCfg)) {
				iRet = HWCFG_RET_SUCCESS;
			}
			else {
				iRet = HWCFG_RET_FILEWRITEFAIL;
			}
			my_close(iFd);iFd=-1;
		}
		else {
			ERR_MSG("%s : File \"%s\" open fail !\n",__FUNCTION__,pszFileName);
			iRet = HWCFG_RET_FILEOPENFAIL;
		}
		
	}
	
	return iRet;
}


NTX_HWCONFIG *NtxHwCfg_Get(void)
{
	int iChk;
	iChk = NtxHwCfg_ChkCfgHeaderEx(_gptNtxHwCfg,1);
	if(iChk>=0) {
		return _gptNtxHwCfg;
	}
	else {
		return 0;
	}
}





int NtxHwCfg_GetTotalFlds(void)
{
	return _TOTOAL_HWCONFIG_FIELDS;
}

int NtxHwCfg_FldName2Idx(const char *szFldName)
{
	int i;
	int iRet = HWCFG_RET_NOTHISFIELDNAME;
	for(i=0;i<_TOTOAL_HWCONFIG_FIELDS;i++) 
	{
		if(0==my_strcmp(szFldName,gtHwConfigFields[i].szFieldName)) {
			iRet = i;
		}
	}
	return iRet ; 
}


int NtxHwCfg_GetFldVal(int iFldIdx,HwConfigField *O_ptHwCfgFld)
{
	if(iFldIdx>=_TOTOAL_HWCONFIG_FIELDS) {
		return HWCFG_RET_NOTHISFIELDIDX;
	}
	O_ptHwCfgFld->szVersion = gtHwConfigFields[iFldIdx].szVersion;
	O_ptHwCfgFld->szFieldName = gtHwConfigFields[iFldIdx].szFieldName;
	O_ptHwCfgFld->iFieldValueCnt = gtHwConfigFields[iFldIdx].iFieldValueCnt;
	O_ptHwCfgFld->szFieldValueA = gtHwConfigFields[iFldIdx].szFieldValueA;
	O_ptHwCfgFld->wFieldType = gtHwConfigFields[iFldIdx].wFieldType;
	return HWCFG_RET_SUCCESS;
}

unsigned char NtxHwCfg_FldStrVal2Val(int iFldIdx,char *szFldStrVal)
{
	int iTotalVals;
	int i;
	unsigned char bRet = 0xff;
	
	if(iFldIdx>=_TOTOAL_HWCONFIG_FIELDS) {
		return 0xff;
	}
	if(gtHwConfigFields[iFldIdx].wFieldType != FIELD_TYPE_IDXSTR) {
		return 0xff;
	}
	iTotalVals = gtHwConfigFields[iFldIdx].iFieldValueCnt;
	ASSERT(iTotalVals<255);
	for(i=0;i<iTotalVals;i++) 
	{
		if(0==my_strcmp(gtHwConfigFields[iFldIdx].szFieldValueA[i],szFldStrVal)) 
		{
			bRet = (unsigned char)i;
			break;
		}
	}
	
	return bRet;
	
}

const char *NtxHwCfg_FldVal2StrVal(int iFldIdx,unsigned char bFldVal)
{
	int iTotalVals;
	int i;
	char *pszRet = 0;
	
	if(iFldIdx>=_TOTOAL_HWCONFIG_FIELDS) {
		return 0;
	}
	if(gtHwConfigFields[iFldIdx].wFieldType != FIELD_TYPE_IDXSTR) {
		return 0;
	}
	iTotalVals = gtHwConfigFields[iFldIdx].iFieldValueCnt;
	ASSERT(iTotalVals<255);
	for(i=0;i<iTotalVals;i++) 
	{
		if(i==(int)bFldVal) {
			pszRet = gtHwConfigFields[iFldIdx].szFieldValueA[i];
			break;
		}
	}
	return pszRet;
}









int NtxHwCfg_ChkCfgHeader(NTX_HWCONFIG *pHdr)
{
	return NtxHwCfg_ChkCfgHeaderEx(pHdr,0);
}

int NtxHwCfg_ChkCfgHeaderEx(NTX_HWCONFIG *pHdr,int iIsIgnoreVersion)
{
	int iRet ;
	
	if(!pHdr) {
		return HWCFG_RET_PTRERR;
	}
	
	if(gszNtxHwCfgMagic[0]==pHdr->m_hdr.cMagicNameA[0] &&\
		gszNtxHwCfgMagic[1]==pHdr->m_hdr.cMagicNameA[1] &&\
		gszNtxHwCfgMagic[2]==pHdr->m_hdr.cMagicNameA[2] &&\
		gszNtxHwCfgMagic[3]==pHdr->m_hdr.cMagicNameA[3] &&\
		gszNtxHwCfgMagic[4]==pHdr->m_hdr.cMagicNameA[4] &&\
		gszNtxHwCfgMagic[5]==pHdr->m_hdr.cMagicNameA[5] &&\
		gszNtxHwCfgMagic[6]==pHdr->m_hdr.cMagicNameA[6] &&\
		gszNtxHwCfgMagic[7]==pHdr->m_hdr.cMagicNameA[7] &&\
		gszNtxHwCfgMagic[8]==pHdr->m_hdr.cMagicNameA[8] &&\
		gszNtxHwCfgMagic[9]==pHdr->m_hdr.cMagicNameA[9] )
	{
		if(iIsIgnoreVersion) {
			iRet = HWCFG_RET_SUCCESS;
		}
		else {
			if(gszNtxHwCfgVersion[0]==pHdr->m_hdr.cVersionNameA[0] &&\
				gszNtxHwCfgVersion[2]==pHdr->m_hdr.cVersionNameA[2] &&\
				gszNtxHwCfgVersion[4]==pHdr->m_hdr.cVersionNameA[4] ) 
			{
				if(pHdr->m_hdr.cVersionNameA[1]>gszNtxHwCfgVersion[1]) {
					iRet = HWCFG_RET_CFGVERTOONEW;
				}
				else if(pHdr->m_hdr.cVersionNameA[1]<gszNtxHwCfgVersion[1]) {
					iRet = HWCFG_RET_CFGVERTOOOLD;
				}
				else {
					if(pHdr->m_hdr.cVersionNameA[3]>gszNtxHwCfgVersion[3]) {
						iRet = HWCFG_RET_CFGVERTOONEW;
					}
					else if(pHdr->m_hdr.cVersionNameA[3]<gszNtxHwCfgVersion[3]) {
						iRet = HWCFG_RET_CFGVERTOOOLD;
					}
					else {
						iRet = HWCFG_RET_SUCCESS;
					}
				}
			}
			else {
				iRet = HWCFG_RET_CFGVERFMTERR;
			}
		}
	}
	else {
		iRet = HWCFG_RET_HDRNOTMATCH;
	}
	
	return iRet;
}

int NtxHwCfg_CfgUpgrade(NTX_HWCONFIG *pHdr)
{
	int iRet = NtxHwCfg_ChkCfgHeader(pHdr);
	
	if( HWCFG_RET_CFGVERTOOOLD == iRet )
	{
		int i;
		int iIdxUpperVersion = -1;
		unsigned char *pb ;
		
		
		
		for(i=0;i<_TOTOAL_HWCONFIG_FIELDS;i++)
		{
			if( gtHwConfigFields[i].szVersion[1]==pHdr->m_hdr.cVersionNameA[1] &&\
				gtHwConfigFields[i].szVersion[3]==pHdr->m_hdr.cVersionNameA[3] )
			{
				iIdxUpperVersion = i;
			}
			else {
				if(iIdxUpperVersion!=-1) {
					break; // got it .
				}
			}
		}
		
		pb = (unsigned char *)&pHdr->m_val;
		
		for(i=iIdxUpperVersion+1;i<_TOTOAL_HWCONFIG_FIELDS;i++) {
			//pb[i] = 0;
		}
		
		pHdr->m_hdr.cVersionNameA[1] = gszNtxHwCfgVersion[1];
		pHdr->m_hdr.cVersionNameA[3] = gszNtxHwCfgVersion[3];
		
		pHdr->m_hdr.bHWConfigSize = _TOTOAL_HWCONFIG_FIELDS;
		
		iRet = 0;
	}
	else if (HWCFG_RET_CFGVERTOONEW == iRet ) {
		WARNING_MSG("%s:[WARNING]config file version newer than this tool !! please update this tool .\n",__FUNCTION__);
	}
	return iRet;
}


int NtxHwCfg_GetCfgTotalFlds(NTX_HWCONFIG *pHdr)
{
	
	int iRet ;
	
	iRet = NtxHwCfg_ChkCfgHeader(pHdr);
	
	if(HWCFG_RET_CFGVERTOOOLD == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too old !! Please update config file !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_CFGVERTOONEW == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too new !! Please update hardware config tool !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_HDRNOTMATCH == iRet) {
		WARNING_MSG("%s:[WARNING]Config header error !! \n",__FUNCTION__);
	}
	
	if(iRet>=0) 
	{
		iRet = pHdr->m_hdr.bHWConfigSize;
	} 
	/*
	int iChkItems = sizeof(gtHwConfigFields)/sizof(gtHwConfigFields[0])
	
	if(iRet!=iChkItems) {
		WARNING_MSG("\n");
	}
	*/
	
	return iRet;
}




int NtxHwCfg_GetCfgFldVal(NTX_HWCONFIG *pHdr,int iFieldIdx)
{
	unsigned char *pbVal ;
	int iRet;
	
	iRet = NtxHwCfg_ChkCfgHeaderEx(pHdr,0);
	
	if(HWCFG_RET_CFGVERTOOOLD == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too old !! Please update config file !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_CFGVERTOONEW == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too new !! Please update hardware config tool !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_HDRNOTMATCH == iRet) {
		WARNING_MSG("%s:[WARNING]Config header error !! \n",__FUNCTION__);
	}
	
	if(iRet>=0) {
		if(_compare_hdrver_fldver(pHdr,iFieldIdx)>=0) {
			pbVal = (unsigned char *)&pHdr->m_val;
			iRet = (int)(pbVal[iFieldIdx]);
		}
		else {
			iRet = HWCFG_RET_CFGVERTOOOLD;
		}
	} 
	
	return iRet;
}

const char *NtxHwCfg_GetCfgFldStrVal(NTX_HWCONFIG *pHdr,int iFieldIdx)
{
	int iRet;
	char *pszRet = 0;
	const char **szValNameA;
	
	iRet = NtxHwCfg_ChkCfgHeaderEx(pHdr,0);
	if(HWCFG_RET_CFGVERTOOOLD == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too old !! Please update config file !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_CFGVERTOONEW == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too new !! Please update hardware config tool !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_HDRNOTMATCH == iRet) {
		WARNING_MSG("%s:[WARNING]Config header error !! \n",__FUNCTION__);
	}
	
	if(iRet>=0) 
	{
		unsigned char *pbVal = (unsigned char *)&pHdr->m_val;
		if(_compare_hdrver_fldver(pHdr,iFieldIdx)>=0) {
			if(gtHwConfigFields[iFieldIdx].wFieldType != FIELD_TYPE_IDXSTR) {
				
			}
			else {
				if((int)pbVal[iFieldIdx]<gtHwConfigFields[iFieldIdx].iFieldValueCnt) {
					szValNameA = (const char **)gtHwConfigFields[iFieldIdx].szFieldValueA;
					pszRet = (char *)szValNameA[pbVal[iFieldIdx]];
				}
			}
		}
	} 
	else {
		ERR_MSG("%s : error (%d)\n",__FUNCTION__,iRet);
	}
	
	return pszRet;
}

int NtxHwCfg_SetCfgFldVal(NTX_HWCONFIG *pHdr,int iFieldIdx,int iFieldVal)
{
	int iRet ;
	
	iRet = NtxHwCfg_ChkCfgHeader(pHdr);
	if(HWCFG_RET_CFGVERTOOOLD == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too old !! Please update config file !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_CFGVERTOONEW == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too new !! Please update hardware config tool !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_HDRNOTMATCH == iRet) {
		WARNING_MSG("%s:[WARNING]Config header error !! \n",__FUNCTION__);
	}
	
	if(iRet>=0) {
		unsigned char *pb = (unsigned char *)&pHdr->m_val;
		//printf("[%d/%d]iFieldVal = %d\n",iFieldIdx,_TOTOAL_HWCONFIG_FIELDS,iFieldVal);
		#ifndef _X86_//[
		if( !(gtHwConfigFields[iFieldIdx].wFieldFlags & FIELD_FLAGS_SW) ) {
			iRet = HWCFG_RET_FIELDTRDONLY;
			WARNING_MSG("%s:[WARNING] field \"%s\" is read only !! \n",
				__FUNCTION__,gtHwConfigFields[iFieldIdx].szFieldName);
		}
		#endif //] !_X86_		
		if(iFieldIdx<_TOTOAL_HWCONFIG_FIELDS) {
		
			pb[iFieldIdx] = (unsigned char)iFieldVal;
		}
		else {
			iRet = HWCFG_RET_NOTHISFIELDIDX;
		}
	} 
	return iRet;
}

int NtxHwCfg_SetCfgFldStrVal(NTX_HWCONFIG *pHdr,int iFieldIdx,const char *pszFieldStrVal)
{
	int iRet ;
	
	iRet = NtxHwCfg_ChkCfgHeader(pHdr);
	if(HWCFG_RET_CFGVERTOOOLD == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too old !! Please update config file !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_CFGVERTOONEW == iRet) {
		WARNING_MSG("%s:[WARNING]Config version too new !! Please update hardware config tool !!\n",__FUNCTION__);
		iRet = 0;
	}
	else if(HWCFG_RET_HDRNOTMATCH == iRet) {
		WARNING_MSG("%s:[WARNING]Config header error !! \n",__FUNCTION__);
	}
	
	if(iRet>=0) {
		unsigned char *pb = (unsigned char *)&pHdr->m_val;
		if(gtHwConfigFields[iFieldIdx].wFieldType != FIELD_TYPE_IDXSTR) {
			iRet = HWCFG_RET_FIELDTYPEERROR;
		}
		#ifndef _X86_//[
		if( !(gtHwConfigFields[iFieldIdx].wFieldFlags & FIELD_FLAGS_SW) ) {
			iRet = HWCFG_RET_FIELDTRDONLY;
			WARNING_MSG("%s:[WARNING] field \"%s\" is read only !! \n",
				__FUNCTION__,gtHwConfigFields[iFieldIdx].szFieldName);
		}
		#endif //] !_X86_
		if(iFieldIdx<_TOTOAL_HWCONFIG_FIELDS) {
			unsigned char bFldVal;
			
			bFldVal = NtxHwCfg_FldStrVal2Val( iFieldIdx, (char *)pszFieldStrVal);
			if(bFldVal==0xff) {
				iRet = HWCFG_RET_NOTHISFIELDNAME;
			}
			else {
				pb[iFieldIdx] = bFldVal;
			}
		}
		else {
			iRet = HWCFG_RET_NOTHISFIELDIDX;
		}
	} 
	
	return iRet;
}

int NtxHwCfg_CompareHdrFldVersion(NTX_HWCONFIG *pHdr,int iFieldIdx)
{
	int iRet = NtxHwCfg_ChkCfgHeaderEx(pHdr,1);
	
	if( iRet >= 0 ) {
		iRet = _compare_hdrver_fldver(pHdr,iFieldIdx);
	}
	
	return iRet;
}

int NtxHwCfg_SetCfgFldValDefs(NTX_HWCONFIG *I_pHdr,int I_iFieldIdx,
		const char **I_pszFieldValDefs,int I_iTotalVals)
{
	int iRet ;
	
	iRet = NtxHwCfg_ChkCfgHeaderEx(I_pHdr,1);

	if(iRet>=0) {
		
		if(I_iFieldIdx>=_TOTOAL_HWCONFIG_FIELDS) {
			iRet = HWCFG_RET_NOTHISFIELDIDX;
			goto exit;
		}
		
		if(gtHwConfigFields[I_iFieldIdx].wFieldType != FIELD_TYPE_IDXSTR) {
			iRet = HWCFG_RET_FIELDTYPEERROR;
			goto exit;
		}
		
		gtHwConfigFields[I_iFieldIdx].szFieldValueA = (char **)I_pszFieldValDefs;
		gtHwConfigFields[I_iFieldIdx].iFieldValueCnt = I_iTotalVals;
	}


exit:	
	
	return iRet;
}

int NtxHwCfg_is_HW_Fld(int I_iFieldIdx)
{
	int iRet = \
		(gtHwConfigFields[I_iFieldIdx].wFieldFlags&FIELD_FLAGS_SW)?0:1;
	return iRet ;
}
int NtxHwCfg_is_SW_Fld(int I_iFieldIdx)
{
	int iRet = \
		(gtHwConfigFields[I_iFieldIdx].wFieldFlags&FIELD_FLAGS_SW)?1:0;
	return iRet;
}


