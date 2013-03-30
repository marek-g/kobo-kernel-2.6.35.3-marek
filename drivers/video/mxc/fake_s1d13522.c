#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/delay.h>


#include "DataType.h"
#include "s1d13521ioctl.h"

#include "s1d13521.h"
#include "s1d13521fb.h"

#ifndef GDEBUG //[
	#define GDEBUG 0
#endif //]GDEBUG
#include <linux/gallen_dbg.h>

#include "epdfb_dc.h"
#include "fake_s1d13522.h"
#include "ntx_hwconfig.h"


#define _PVI_IOCTL_INTERFACE	1
#define _QT_SUPPORT				1


//#define OUTPUT_IMGFILE_ENABLE		1
#define OUTPUT_IMGFILE "/dev/shm/epdtemp"


#define BusIssueReadReg(wRegIdx) S1D13522_reg_read(wRegIdx)
#define BusIssueWriteReg(wRegIdx,wValue) S1D13522_reg_write(wRegIdx,wValue)



extern volatile NTX_HWCONFIG *gptHWCFG;



// for s1d13522 hardware registers .
#define S1D13522_REGADDR_PRODUCTCODE		0x0002 // Product code register .
#define S1D13522_REGADDR_LUTSTATUS			0x0336 // Lookup table status register .
#define S1D13522_REGADDR_HOSTIFMACONF		0x0140 // Host interface memory access configuration .
#define S1D13522_REGADDR_GENCONF			0x032C // General configuration register .
#define S1D13522_REGADDR_HOSTMEMPIXSWAP		0x015E // Host memory access pixel swap configuration .
#define S1D13522_REGADDR_PUBC				0x0330 // Panel Update Buffer Configuration .
#define S1D13522_REGADDR_DSPENGINTRSR		0x033A // Display Engine Interrupt Raw Status Register .
//static uint16_t gwS1d13522_hostifmaconf = 0x0000; //register 0x140, host packed pixel is 4 bits.
static uint16_t gwS1d13522_GenConf = 0x0000; //register 0x32C, General config configuration register .
static uint16_t gwS1d13522_hostmempixswapconf = 0x0000; //register 0x15E, host packed pixel is 4 bits.
static uint16_t gwS1d13522_DspEngIntrRSR = 0x0000;// Display Engine Interrupt Raw Status Register .


// for epdfb_dc .
static epdfb_rotate_t gtRotate = epdfb_rotate_0;
static int giPixelBits = 4;//
static EPDFB_DC *gptEPD_dc_current ;// epd device content .

static gwScrW=800,gwScrH=600;

//static DECLARE_COMPLETION(fake13522_inited);

//static u32 currHeight=800,currWidth=600;
#define S1D13522_PRODUCTCODE	0x004d


#ifdef _QT_SUPPORT //[
	// vars ...
	static int resolve_conflict_step;
	static unsigned long gLastUpdParm0;


	static BOOL bConflictTimerRunning; // KEG 20090825
	static struct timer_list conflict_resolution_timer;	// KEG 20090825

	// functions ...
	static void s1d13521fb_resolve_conflict(unsigned long dummy);
	
	#define _ES_LUTNOWAIT_	1
	
#endif //] _QT_SUPPORT

#ifdef _PVI_IOCTL_INTERFACE	//[
#define _PVI_IMAGE_SIZE		(S1D_DISPLAY_HEIGHT*S1D_DISPLAY_WIDTH/4)
#define _PVI_IMAGE_SIZE_800x600		(800*600/4)

//static BYTE *ImagePVI_BufferA[_PVI_IMAGE_SIZE];
//static BYTE *ImagePVI = ImagePVI_BufferA;
static BYTE *ImagePVI ;

//static BYTE gbTemp4BitsFBA[_PANEL_HEIGHT*_PANEL_WIDTH/(8/S1D_DISPLAY_BPP)];


static struct timer_list 	pvi_refersh_timer;

int pvi_Init(VOID);
void pvi_Deinit(VOID);
#endif //] _PVI_IOCTL_INTERFACE

static ST_PVI_CONFIG	ConfigPVI={
	.CurRotateMode =(_INIT_ROTMODE << 8),
	// .PastRotateMode=(_INIT_ROTMODE << 8),
	.PastRotateMode=0xffff,
	.Deepth=2,
	.StartX=0,
	.StartY=0,
	.Width =_PANEL_HEIGHT,
	.Height=_PANEL_WIDTH,

	.Reg0x140=0,
	.Reg0x330=0,

	.ReverseGrade=0
};


#ifdef SHOW_PROGRESS_BAR
static DECLARE_WAIT_QUEUE_HEAD(progress_WaitQueue);
volatile static int gIsProgessRunning=1;
static VOID PROGRESS_BAR(EPDFB_DC *pDC);
#endif



/*
* 
*  for boot argument .
* 
*/  
#define _TEST_CMDLINE	0
#define _MYCMDLINE_PARSE	1

#if (_MYCMDLINE_PARSE==1) //[
#define _MYINIT_TEXT	__init
#define _MYINIT_DATA	__initdata
#else//][
#define _MYINIT_TEXT	__init
#define _MYINIT_DATA	__initdata
#endif//]

volatile unsigned char *gpbLOGO_paddr;
volatile unsigned char *gpbLOGO_vaddr;
volatile unsigned long gdwLOGO_size;

volatile unsigned char *gpbWF_paddr;
volatile unsigned char *gpbWF_vaddr;
volatile unsigned long gdwWF_size; 

volatile int giRootDevNum=0;
volatile int giRootPartNum=1;





/**
 * \fn _MemoryRequest
 * \brief allocate memory region for kermit
 * \param   address,
 * \param   memory length
 * \param   name assigned to memory region
 * \return  none
 */
static void * _MemoryRequest(u32 addr, u32 len, const char * name)
{
    void * mem = NULL;
    do {
        printk(KERN_DEBUG "***%s:%d: request memory region! addr=0x%08x, len=%d***\n",
                __FUNCTION__, __LINE__, addr, len);
        if (!request_mem_region(addr, len, name)) {
            printk(KERN_CRIT "fake_s1d13522:  request memory region failed! addr=0x%08x, len %d\n", addr, len);
            break;
        }
        mem = (void *) ioremap_nocache(addr, len);
        if (!mem) {
            printk(KERN_CRIT "***%s:%d: could not ioremap %s***\n", __FUNCTION__, __LINE__, name);
            release_mem_region(addr, len);
            break;
        }
    } while (0);
    return mem;
}

/**
 * \fn _MemoryRelease
 * \brief free kermit's memory region
 * \param  memory region address
 * \param  address
 * \param  memory length
 * \return none
 */
void _MemoryRelease(void * unmap_addr, unsigned long addr, unsigned long len)
{
    iounmap(unmap_addr);
    release_mem_region(addr, len);
    printk(KERN_INFO "fake_s1d13522:  release memory region!\n");
}



static int _MYINIT_TEXT waveform_p_setup(char *str)
{
	#if (_TEST_CMDLINE == 0)
	gpbWF_paddr = (unsigned char *)simple_strtoul(str,NULL,0);
	if(NULL==gpbWF_vaddr) {
		gpbWF_vaddr = (u32 *)_MemoryRequest((u32)gpbWF_paddr, gdwWF_size, "waveform_p");
		if(!gpbWF_vaddr) {
			return 0;
		}
	}
	printk("%s() wf_p=%p,vaddr=%p,size=%lu\n",__FUNCTION__,
		gpbWF_paddr,gpbWF_vaddr,gdwWF_size);
	
	#if 0//[
	{
		volatile u8 *pbTemp;
		int i,irequestsize=32;
		
		printk("gpbWF_vaddr (%p),size=%u = [\n\t",gpbWF_vaddr,irequestsize);
		if(gpbWF_vaddr) {
			pbTemp = (u8*)gpbWF_vaddr;
			for(i=0;i<irequestsize;i++) {
				printk("%02X,",pbTemp[i]);
			}
		}
		printk("]\n");		
	}
	#endif //]
	#else
	printk("%s() str=%s\n",__FUNCTION__,str);
	#endif
	return 1;
}


static int _MYINIT_TEXT waveform_size_setup(char *str)
{
	#if (_TEST_CMDLINE == 0)
	gdwWF_size = (unsigned int)simple_strtoul(str,NULL,0);
	printk("%s() wf_size=%lu\n",__FUNCTION__,gdwWF_size);
	#else
	printk("%s() str=%s\n",__FUNCTION__,str);
	#endif
	return 1;
}

static int _MYINIT_TEXT logo_p_setup(char *str)
{
	#if (_TEST_CMDLINE == 0)
	gpbLOGO_paddr = (unsigned char *)simple_strtoul(str,NULL,0);
	if(NULL==gpbLOGO_vaddr) {
		gpbLOGO_vaddr = (u32 *)_MemoryRequest((u32)gpbLOGO_paddr, gdwLOGO_size, "logo_p");
		if(!gpbWF_vaddr) {
			return 0;
		}
	}
	printk("%s() logo_p=%p,vaddr=%p,size=%lu\n",__FUNCTION__,
		gpbLOGO_paddr,gpbLOGO_vaddr,gdwLOGO_size);
	#else
	printk("%s() str=%s\n",__FUNCTION__,str);
	#endif
	return 1;
}

static int _MYINIT_TEXT logo_size_setup(char *str)
{
	#if (_TEST_CMDLINE == 0)
	gdwLOGO_size = (int)simple_strtoul(str,NULL,0);
	printk("%s() logo_szie=%lu\n",__FUNCTION__,gdwLOGO_size);
	#else
	printk("%s() str=%s\n",__FUNCTION__,str);
	#endif
	return 1;
}



static int _MYINIT_TEXT root_path_setup(char *str)
{
	#if (_TEST_CMDLINE == 0)
	if(str[5]=='m'&&str[6]=='m'&&str[7]=='c'&&\
			str[8]=='b'&&str[9]=='l'&&str[10]=='k') 
	{
		giRootDevNum=(int)(str[11]-'0');
		giRootPartNum=(int)simple_strtoul(&str[13],NULL,0);
	}
	else {
	}
	printk("%s() rootdev=%d,rootpart=%d\n",__FUNCTION__,giRootDevNum,giRootPartNum);
	#else
	printk("%s() str=%s\n",__FUNCTION__,str);
	#endif
	return 1;
}

#if (_MYCMDLINE_PARSE==0) //[
__setup("waveform_p=",waveform_p_setup);
__setup("waveform_sz=",waveform_size_setup);
__setup("logo_p=",logo_p_setup);
__setup("logo_sz=",logo_size_setup);
#else //][
void fake_s1d13522_parse_epd_cmdline(void)
{
	static int iParseCnt = 0;
	char *pcPatternStart,*pcPatternVal,*pcPatternValEnd,cTempStore='\0';
	unsigned long ulPatternLen;

	char *szParsePatternA[]={"waveform_sz=","waveform_p=","logo_sz=","logo_p=","root="};
	int ((*pfnDispatchA[])(char *str))={ \
		waveform_size_setup,waveform_p_setup,\
		logo_size_setup,logo_p_setup,root_path_setup };
		
	int i;
	char *pszCmdLineBuf;
	
	
	if(iParseCnt++>0) {
		WARNING_MSG("%s : epd cmdline parse already done .\n",__FUNCTION__);
		return ;
	}
	//printk("%s():cmdline(%d)=%s\n",__FUNCTION__,strlen(saved_command_line),saved_command_line);
		
	pszCmdLineBuf = kmalloc(strlen(saved_command_line)+1,GFP_KERNEL);
	ASSERT(pszCmdLineBuf);
	strcpy(pszCmdLineBuf,saved_command_line);
	//printk("%s():cp cmdline=%s\n",__FUNCTION__,pszCmdLineBuf);
	
	for(i=0;i<sizeof(szParsePatternA)/sizeof(szParsePatternA[0]);i++) {
		ulPatternLen = strlen(szParsePatternA[i]);
		pcPatternStart = strstr(pszCmdLineBuf,szParsePatternA[i]);
		if(pcPatternStart) {
			pcPatternVal=pcPatternStart + ulPatternLen ;
			pcPatternValEnd = strchr(pcPatternVal,' ');
			if(pcPatternValEnd) {
				cTempStore = *pcPatternValEnd;
				*pcPatternValEnd = '\0';
			}
			//printk("%s():pattern \"%s\" ,val=\"%s\"\n",__FUNCTION__,szParsePatternA[i],pcPatternVal);
			pfnDispatchA[i](pcPatternVal);
			if(pcPatternValEnd) {
				*pcPatternValEnd = cTempStore;
			}
		}
	}
	
	if(pszCmdLineBuf) {
		kfree(pszCmdLineBuf);
	}
}
#endif //]





//
// file operation to emulate S1D13521 flash read/write .
//
static int read_file(char *filename, char *pBuffer)
{
  struct file *file;
  loff_t pos = 0;
  int result = 0;
  mm_segment_t old_fs = get_fs();

  file = filp_open(filename, O_RDONLY, 0);
  
  if (IS_ERR(file)) {
  	printk ("[%s-%d] failed open %s,(%p)\n",__func__,__LINE__,filename,file);
  }
  else {
		ssize_t tSz;
    set_fs(KERNEL_DS);
    printk(KERN_DEBUG);
    tSz = file->f_op->read(file, pBuffer, 4096, &file->f_pos);
		if(tSz!=4096) {
  		printk ("[%s-%d] failed read %s,%d!=4096\n",__func__,__LINE__,filename,tSz);
			result = 0;
		}
		else {
			result = 1;
		}
    filp_close(file, NULL);
    set_fs(old_fs);
  }
  return result;
}

static int write_file_ex(char *filename, char *data,unsigned long dwSize)
{
  struct file *file;
  loff_t pos = 0;
  int fd;
	int iRet = 1;

  file = filp_open(filename, O_WRONLY|O_CREAT, 0644);
  mm_segment_t old_fs = get_fs();

	if (IS_ERR(file)) {
  	printk ("[%s-%d] failed open %s,(%p)\n",__func__,__LINE__,filename,file);
	}
	else {
		ssize_t tSz;
		set_fs(KERNEL_DS);
		tSz = file->f_op->write(file, (char *)data, dwSize, &file->f_pos);
		if(dwSize!=tSz) {
			printk ("[%s-%d] failed write %s,%d!=4096\n",__func__,__LINE__,filename,tSz);
			iRet = 0;
		}
		filp_close(file, NULL);
		set_fs(old_fs);
	}
  return iRet;

}

static int write_file(char *filename, char *data)
{
	return write_file_ex(filename, data,4096);
}

static char *gFlashBuffer;
static BOOL Flash_File_Erase(DWORD FlashAddress, DWORD Length)
{
	BOOL fRet = TRUE;
	unsigned char FilenameA[256];
	
	if (!gFlashBuffer)
		gFlashBuffer = __get_free_pages(GFP_KERNEL, 1);
	sprintf(FilenameA,"/etc/epson_flash_%08X",(FlashAddress&0xFFFFF000));
	memset (gFlashBuffer, 0xFF, 4096);
	write_file (FilenameA, gFlashBuffer);
	return fRet;
}

static BOOL Flash_File_Write(DWORD FlashAddress, DWORD Length, PBYTE pData)
{
	BOOL fRet = TRUE;
	unsigned char FilenameA[256];
	
	if (!gFlashBuffer) {
		gFlashBuffer = __get_free_pages(GFP_KERNEL, 1);
		ASSERT(gFlashBuffer);
	}
	sprintf(FilenameA,"/etc/epson_flash_%08X",(FlashAddress&0xFFFFF000));
	
	if (!read_file (FilenameA, gFlashBuffer)) {
		memset (gFlashBuffer, 0xFF, 4096);
	}
	memcpy (gFlashBuffer+(FlashAddress&0x0FFF), pData, Length);
	if(!write_file (FilenameA, gFlashBuffer)) {
		printk("[kenel]%s-%d : write_file %s fail !!\n",__FUNCTION__,__LINE__,FilenameA);
		fRet = 0;
	}
	return fRet;
}

static BOOL Flash_File_Read(DWORD FlashAddress, DWORD Length, PBYTE pData)
{
	BOOL fRet = TRUE;
	unsigned char FilenameA[256];
	
	if (!gFlashBuffer) {
		gFlashBuffer = __get_free_pages(GFP_KERNEL, 1);
		ASSERT(gFlashBuffer);
	}
	
	
	sprintf(FilenameA,"/etc/epson_flash_%08X",(FlashAddress&0xFFFFF000));
	if (read_file (FilenameA, gFlashBuffer)) {
		memcpy (pData, gFlashBuffer+(FlashAddress&0x0FFF), Length);
	}
	else {
		memset (pData, 0xFF, Length);
	}
	return fRet;
}

#define WAVEFORM_4BIT_MARK 0x20000
static int BusIssueFlashOperation(PS1D13532_FLASH_PACKAGE pFlashControl,EPDFB_DC *pDC)
{
int RetCode=-1;
	//DEBUGMSG(_DUMP_IOCTL_FLASH, "pFlashControl->Command=%u\n", 				pFlashControl->Command);
	//DEBUGMSG(_DUMP_IOCTL_FLASH, "pFlashControl->StartAddr=0x%08x\n", 	pFlashControl->StartAddr);
	//DEBUGMSG(_DUMP_IOCTL_FLASH, "pFlashControl->DataLength=0x%08x\n", pFlashControl->DataLength);
	//DEBUGMSG(_DUMP_IOCTL_FLASH, "pFlashControl->Buf=0x%p\n", 				pFlashControl->Buf);

	switch(pFlashControl->Command) {
		case _FLASH_CMD_GET_INFO:
			printk ("[%s-%d] Flash_GetID (0x%08X);\n",__func__, __LINE__, pFlashControl->Buf);	
			//DEBUGMSG(_DUMP_IOCTL_FLASH, "IDCode is 0x%08x\n", *((PDWORD) pFlashControl->Buf));
			RetCode=4;
			break;

		case _FLASH_CMD_ERASE:
			if(Flash_File_Erase(pFlashControl->StartAddr, pFlashControl->DataLength) == FALSE)
				break;
			RetCode=0;
			break;

		case _FLASH_CMD_WRITE:
			if(Flash_File_Write(pFlashControl->StartAddr, pFlashControl->DataLength, pFlashControl->Buf) == FALSE)
				break;
			RetCode=0;
			break;

		case _FLASH_CMD_READ:
			Flash_File_Read(pFlashControl->StartAddr, pFlashControl->DataLength, pFlashControl->Buf);
			if (WAVEFORM_4BIT_MARK == pFlashControl->StartAddr) {
				ASSERT(pDC);
				
				if(4==pDC->pfnGetWaveformBpp()) {
					printk ("[%s-%d] return wf 4 bit flag\n",__func__, __LINE__);
					pFlashControl->Buf[0] = 0x0a;
				}
				else {
					printk ("[%s-%d] return wf 3 bit flag\n",__func__, __LINE__);
					pFlashControl->Buf[0] = 0x0f;
				}
			}
			break;

		default:
			return FALSE;
	};
	return RetCode;
}


static void fake_s1d13522_setwfmode(EPDFB_DC *pDC,int iWFMode)
{
	DBG_MSG("%s(),wf.mode=%d\n",__FUNCTION__,iWFMode);
	if(pDC->pfnSetWaveformMode) {
		pDC->pfnSetWaveformMode(iWFMode);
	}
	pDC->iWFMode = iWFMode;
}


void fake_s1d13522_progress_start(EPDFB_DC *pDC)
{
#ifdef SHOW_PROGRESS_BAR//[
	int ret;
	ret = kernel_thread(PROGRESS_BAR,pDC,CLONE_KERNEL);
	if(ret < 0){
		printk("Progress bar thread creat error\n");
	}
#endif //]SHOW_PROGRESS_BAR
}

void fake_s1d13522_progress_stop(void)
{
	DBG_MSG("====%s()====!\n",__FUNCTION__);
	gIsProgessRunning = 0;
}

/*********************************************************
 * 
 * 
 *********************************************************/
EPDFB_DC *fake_s1d13522_initEx3(unsigned char bBitsPerPixel,unsigned char *pbDCBuf,
	unsigned short wScrW,unsigned short wScrH,unsigned short wFBW,unsigned short wFBH)
{
	EPDFB_DC *pDC = 0;

#ifdef _PVI_IOCTL_INTERFACE//[
	pvi_Init();
#endif//]_PVI_IOCTL_INTERFACE

#if (_MYCMDLINE_PARSE==1) //[
	fake_s1d13522_parse_epd_cmdline();
#endif //]

	gwScrW = wScrW;
	gwScrH = wScrH;
	pDC = epdfbdc_create_ex2(wFBW,wFBH,gwScrW,gwScrH,bBitsPerPixel,pbDCBuf,\
		EPDFB_DC_FLAG_REVERSEDRVDATA|EPDFB_DC_FLAG_SKIPRIGHTPIXEL);
	

	//currWidth = _INIT_HSIZE;
	//currHeight = _INIT_VSIZE;
	if(pDC) {
		//ImagePVI = pDC->pbDCbuf;
		gptEPD_dc_current = pDC;
	}
	
#ifdef _QT_SUPPORT//[
	#ifndef _ES_LUTNOWAIT_ //[
	init_timer(&conflict_resolution_timer);
	conflict_resolution_timer.function = s1d13521fb_resolve_conflict;
	#endif //]
	bConflictTimerRunning = FALSE;
#endif //] _QT_SUPPORT
	
	if(1==gptHWCFG->m_val.bUIStyle) {
		gtRotate = epdfb_rotate_90;
	}

	//complete_all(&fake13522_inited);
	return pDC;
}
EPDFB_DC *fake_s1d13522_initEx2(unsigned char bBitsPerPixel,unsigned char *pbDCBuf,
	unsigned short wScrW,unsigned short wScrH)
{
	return fake_s1d13522_initEx3(bBitsPerPixel,pbDCBuf,wScrW,wScrH,wScrW,wScrH);
}
EPDFB_DC *fake_s1d13522_initEx(unsigned char bBitsPerPixel,unsigned char *pbDCBuf)
{
	return fake_s1d13522_initEx2(bBitsPerPixel,pbDCBuf,800,600);
}


EPDFB_DC *fake_s1d13522_init(void)
{
	return fake_s1d13522_initEx(4,0);
}

int fake_s1d13522_release(EPDFB_DC *pDC)
{
	int iRet = 0;

#ifdef _QT_SUPPORT//[
	#ifndef _ES_LUTNOWAIT_ //[
	del_timer(&conflict_resolution_timer);	// KEG 20090825
	#endif //]
	bConflictTimerRunning = FALSE;
#endif //] _QT_SUPPORT
	
	if(pDC) {
		epdfbdc_delete(pDC);
		gptEPD_dc_current = 0;
	}
	if(gpbLOGO_vaddr) {
		//kermitMemoryRelease(gpbLOGO_vaddr,,);
		//gpbLOGO_vaddr = NULL;
	}
	if(gpbWF_vaddr) {
		//gpbWF_vaddr = NULL;
	}
	pvi_Deinit();
	return iRet;
}

int fake_s1d13522_display_img(u16 wX,u16 wY,u16 wW,u16 wH,u8 *pbImgBuf,
	EPDFB_DC *pDC,int iPixelBits,epdfb_rotate_t I_tRotate)
{
	int iRet = 0;
	EPDFB_IMG tEPD_img;
	EPDFB_ROTATE_T tRotate;
	u16 wTemp ;
	unsigned int uiCnt;
	int iIsFullScreen = 0;
	volatile u8 *pbRealFB = 0;
	
	
	ASSERT(pDC);
	
	DBG_MSG("%s() : x=%d,y=%d,w=%d,h=%d,r=%d,bit=%d,p=%p,dc.w=%d,dc.h=%d,dc.fb.w+=%d,dc.fb.h+=%d\n",__FUNCTION__,\
		wX,wY,wW,wH,I_tRotate,iPixelBits,pbImgBuf,pDC->dwWidth,pDC->dwHeight,pDC->dwFBWExtra,pDC->dwFBHExtra);
	
	uiCnt = 0 ;

	
	
	if(0==wW||0==wH) {
		DBG_MSG("[%s] skip !!,W or H =0 !!\n",__FUNCTION__);
		return -1;
	}
	
	if( 1==pDC->iIsForceWaitUpdateFinished || pDC->iWFMode!=pDC->iLastWFMode ) 
	{
		if(pDC->pfnWaitUpdateComplete) {
			pDC->pfnWaitUpdateComplete();
		}
		else {
			while(pDC->pfnIsUpdating()) {
				schedule_timeout(1);
				if(printk_ratelimit()) {
				printk("EPD updating ... (t=%u,cnt=%lu)\n",jiffies,++uiCnt);
				}
			}
		}
	}
	
	//tEPD_img.dwX = wX&0xfffe;
	//tEPD_img.dwY = wY&0xfffe;
	
	tEPD_img.dwX = wX;
	tEPD_img.dwY = wY;
	
	tEPD_img.dwW = wW;
	if(wW&0x1) {
		tEPD_img.dwW += 1;
	}
	else {
	}
	
	tEPD_img.dwH = wH;
	
	
	tEPD_img.pbImgBuf = pbImgBuf;
	tEPD_img.bPixelBits = (unsigned char)(iPixelBits);
	switch (I_tRotate) {
	case epdfb_rotate_0:
		tRotate = EPDFB_R_0;
		break;
	case epdfb_rotate_90:
		tRotate = EPDFB_R_90;
		break;
	case epdfb_rotate_180:
		tRotate = EPDFB_R_180;
		break;
	case epdfb_rotate_270:
		tRotate = EPDFB_R_270;
		break;
	default:
		ASSERT(0);
		break;
	}
	
	
	if(pDC->pfnVcomEnable) {
		pDC->pfnVcomEnable(1);
		//VCOM_TRUNON_BY_TIMER(20,pDC);
	}
	else {
	}
	
	if(pDC->pfnSetPartialUpdate) {
		#ifdef __KERNEL__//[
		//if( gptHWCFG && 0==gptHWCFG->m_val.bUIStyle ) 
		if( gptHWCFG ) 
		{
			if(wW * wH == pDC->dwWidth * pDC->dwHeight) {
				DBG_MSG("%s : fullscree update!\n",__FUNCTION__);
				pDC->pfnSetPartialUpdate(0);
			}
			else {
				pDC->pfnSetPartialUpdate(1);
			}
		}
		#endif //]
	}




	if(pDC->pfnPutImg) {
		pDC->pfnPutImg(&tEPD_img,tRotate);
	}
	else {
		//epdfbdc_fbimg_normallize(pDC,&tEPD_img);
		
		epdfbdc_put_fbimg(pDC,&tEPD_img,tRotate);

	epdfbdc_get_rotate_active(pDC,&tEPD_img.dwX,&tEPD_img.dwY,
		&tEPD_img.dwW,&tEPD_img.dwH,tRotate);
	
	if(pDC->pfnSetUpdateRect) {
		pDC->pfnSetUpdateRect(tEPD_img.dwX,tEPD_img.dwY,tEPD_img.dwW,tEPD_img.dwH);
	}
	
	if(pDC->pfnGetRealFrameBufEx) {
		pbRealFB = (u8 *)pDC->pfnGetRealFrameBufEx(0);
	}
	else if(pDC->pfnGetRealFrameBuf) {
		pbRealFB = (u8 *)pDC->pfnGetRealFrameBuf();
	}
	
	if(pDC->pbDCbuf==(unsigned char *)pbRealFB) {
	}
	else
	if(pDC->dwFlags&EPDFB_DC_FLAG_FLASHDIRTY)
	{
		// dirty update ...
		unsigned long dwDirtyOffsetEnd,dwDirtyOffsetStart;
		
		epdfbdc_get_dirty_region(pDC,&dwDirtyOffsetStart,&dwDirtyOffsetEnd);
		if(pbRealFB) {
			
			memcpy((void *)(volatile u32 *)(pbRealFB+dwDirtyOffsetStart),\
				pDC->pbDCbuf+dwDirtyOffsetStart, dwDirtyOffsetEnd-dwDirtyOffsetStart);
			
			if(pDC->dwFlags&EPDFB_DC_FLAG_OFB_RGB565) {
				epdfbdc_dcbuf_to_RGB565(pDC,pbRealFB,(pDC->dwWidth*pDC->dwHeight)>>1);
			}
		}
	}
	else 
	{
		// full update ...
		//extern unsigned int marvell_logo_800x600_size;
		//extern unsigned const char marvell_logo_800x600[] ;		
		if(pDC->pfnGetRealFrameBufEx) {
			pbRealFB = (u8 *)pDC->pfnGetRealFrameBufEx(0);
		}
		else if(pDC->pfnGetRealFrameBuf) {
			pbRealFB = (u8 *)pDC->pfnGetRealFrameBuf();
		}
		
		if(pbRealFB) {
			if(pDC->dwFlags&EPDFB_DC_FLAG_OFB_RGB565) {
				epdfbdc_dcbuf_to_RGB565(pDC,pbRealFB,(pDC->dwWidth*pDC->dwHeight)>>1);
			}
			else {
				memcpy((void *)(volatile u32 *)(pbRealFB),\
					pDC->pbDCbuf, pDC->dwDCSize);
			
				//memcpy((void *)(volatile u32 *)((u8 *)pDC->pfnGetRealFrameBuf()),\
				//	marvell_logo_800x600, marvell_logo_800x600_size);
			}
		}
	}
	}
	
#ifdef OUTPUT_IMGFILE_ENABLE//[

	{
		EPDFB_DC *ptDC_temp;
		char cfnbufA[256];
		int iBits=8;
		unsigned long _w=pDC->dwWidth,_h=pDC->dwHeight;


		ptDC_temp = epdfbdc_create_ex ( _w,_h,iBits,0,pDC->dwFlags);
		epdfbdc_put_dcimg(ptDC_temp,pDC,0,0,0,_w,_h,0,0);
		sprintf(cfnbufA,"%s_%ux%u.raw%d",OUTPUT_IMGFILE,_w,_h,iBits);

		printk("write raw img -> \"%s\" ,%d bits,%u bytes,w=%u,h=%u\n",cfnbufA,iBits,ptDC_temp->dwDCSize,_w,_h);
		write_file_ex(cfnbufA,ptDC_temp->pbDCbuf, ptDC_temp->dwDCSize) ;
		
		printk("rawtoppm %u %u %s > xxx.ppm \n",pDC->dwWidth,pDC->dwHeight,cfnbufA);

		epdfbdc_delete(ptDC_temp);
	}
#endif //]OUTPUT_IMGFILE_ENABLE
		
	
	
	if(pDC->pfnDispStart) {
		pDC->pfnDispStart(1);
	}
	
	if(pDC->pfnVcomEnable) {
		pDC->pfnVcomEnable(0);
		//VCOM_TRUNOFF_BY_TIMER(20,pDC);
	}
	else {
	}
	
	pDC->iLastWFMode = pDC->iWFMode;
	pDC->iIsForceWaitUpdateFinished = 0;
	
	return iRet;
}


int _Epson_displayCMD(PST_IMAGE_PGM pt,EPDFB_DC *pDC)
{
	int iRet = 0;
	u32 newMode ;
	u32 currMode ;
	int iCurWfBpp;
	
	GALLEN_DBGLOCAL_BEGIN();
	
	ASSERT(pDC->pfnGetWaveformMode);
	currMode = pDC->pfnGetWaveformMode();
	ASSERT(pDC->pfnGetWaveformBpp);
	iCurWfBpp = pDC->pfnGetWaveformBpp();
	
	
//					setWaveform(par,pt->WaveForm+1);
	if (pt->Width*pt->Height == pDC->dwWidth*pDC->dwHeight) {
		GALLEN_DBGLOCAL_RUNLOG(2);
		ASSERT(pDC->pfnGetWaveformBpp);
		if(4==iCurWfBpp) 
		{
			GALLEN_DBGLOCAL_RUNLOG(3);
			newMode = 2;// GC16 .
		}
		else if(3==iCurWfBpp) {
			GALLEN_DBGLOCAL_RUNLOG(4);
			newMode = 3;// 
		}
		else {
			GALLEN_DBGLOCAL_RUNLOG(5);
			ASSERT(0);
		}
	}
	else 
	{
		GALLEN_DBGLOCAL_RUNLOG(6);
		newMode = pt->WaveForm;
	}
	
	if (newMode != currMode) {
		GALLEN_DBGLOCAL_RUNLOG(7);
		/* set wafeforms new mode */
		ASSERT(pDC->pfnSetWaveformMode);
		fake_s1d13522_setwfmode(pDC,(int)newMode);
	}

	fake_s1d13522_display_img(pt->StartX,pt->StartY,pt->Width,pt->Height,\
		pt->Data,pDC,giPixelBits,gtRotate);
	GALLEN_DBGLOCAL_END();
	
	return iRet;
}

static EN_EPSON_ERROR_CODE S1D13522_reg_write(u16 wIndex, u16 wValue)
{
	EN_EPSON_ERROR_CODE tRet = _EPSON_ERROR_SUCCESS;
	
	GALLEN_DBGLOCAL_BEGIN();
	
	switch (wIndex) {
	case S1D13522_REGADDR_HOSTIFMACONF:GALLEN_DBGLOCAL_RUNLOG(10);
	{
		unsigned short wRotate;
		int iPixelBits;
		
		GALLEN_DBGLOCAL_RUNLOG(11);
		//gwS1d13522_hostifmaconf = wValue;
		ConfigPVI.Reg0x140 = wValue;
		
		
		//wRotate = gwS1d13522_hostifmaconf&0x0300;
		wRotate = ConfigPVI.Reg0x140&0x0300;
		
		#if 1 //[ auto rotate by ntx hw config .
		if(gptHWCFG)
		{
			
			if(0x0000==wRotate) {
				//ASSERT(gptHWCFG);
				if(0==gptHWCFG->m_val.bDisplayPanel||3==gptHWCFG->m_val.bDisplayPanel||\
						6==gptHWCFG->m_val.bDisplayPanel||8==gptHWCFG->m_val.bDisplayPanel)
				{
					// Left Out EPD Panel ...
					GALLEN_DBGLOCAL_RUNLOG(12);
				}
				else {
					// Right Out EPD Panel (E60MT2,E60820,E60822,E60830,E60832,...)
					GALLEN_DBGLOCAL_RUNLOG(13);
					wRotate = 0x0200;// 0->180
				}
			}
			else if(0x0200==wRotate) {
				ASSERT(gptHWCFG);
				if(0==gptHWCFG->m_val.bDisplayPanel||3==gptHWCFG->m_val.bDisplayPanel||\
						6==gptHWCFG->m_val.bDisplayPanel||8==gptHWCFG->m_val.bDisplayPanel)
				{
					// Left Out EPD Panel ...
					GALLEN_DBGLOCAL_RUNLOG(12);
				}
				else {
					// Right Out EPD Panel (E60MT2,E60820,E60822,E60830,E60832,...)
					GALLEN_DBGLOCAL_RUNLOG(13);
					wRotate = 0x0000;// 180->0
				}
			}
			else if(0x0100==wRotate) {
				ASSERT(gptHWCFG);
				if(0==gptHWCFG->m_val.bDisplayPanel||3==gptHWCFG->m_val.bDisplayPanel||\
						6==gptHWCFG->m_val.bDisplayPanel||8==gptHWCFG->m_val.bDisplayPanel)
				{
					// Left Out EPD Panel ...
					GALLEN_DBGLOCAL_RUNLOG(24);
					wRotate = 0x0300;// 90->270
				}
				else {
					// Right Out EPD Panel (E60MT2,E60820,E60822,E60830,E60832,...)
					GALLEN_DBGLOCAL_RUNLOG(25);
				}
			}
			else if(0x0300==wRotate) {
				ASSERT(gptHWCFG);
				if(0==gptHWCFG->m_val.bDisplayPanel||3==gptHWCFG->m_val.bDisplayPanel||\
						6==gptHWCFG->m_val.bDisplayPanel||8==gptHWCFG->m_val.bDisplayPanel)
				{
					// Left Out EPD Panel ...
					GALLEN_DBGLOCAL_RUNLOG(26);
					wRotate = 0x0100;// 270->90
				}
				else {
					// Right Out EPD Panel (E60MT2,E60820,E60822,E60830,E60832,...)
					GALLEN_DBGLOCAL_RUNLOG(27);
				}
			}
		}
		else {
			// without ntx hwconfig .
			if(0x0000==wRotate) {
			}
			else if(0x0200==wRotate) {
			}
			else if(0x0100==wRotate) {
				wRotate = 0x0300;// 90->270
			}
			else if(0x0300==wRotate) {
				wRotate = 0x0100;// 270->90
			}
		}
		#endif //]
		
		wRotate = wRotate >> 8;
		
		//wRotate = ((gwS1d13522_hostifmaconf>>8) & 0x0003);
		
		DBG_MSG("%s(%d):rotate change %d=>%d\n",__FILE__,__LINE__,\
			gtRotate,wRotate);
		gtRotate = (epdfb_rotate_t)wRotate;
		
		
		if((0x0020==(ConfigPVI.Reg0x140&0x0030))) {
			GALLEN_DBGLOCAL_RUNLOG(14);
			iPixelBits = 4;
		}
		else 
		if((0x0030==(ConfigPVI.Reg0x140&0x0030))) {
			GALLEN_DBGLOCAL_RUNLOG(15);
			iPixelBits = 8;
		}
		else 
		if((0x0010==(ConfigPVI.Reg0x140&0x0030))) {
			GALLEN_DBGLOCAL_RUNLOG(16);
			iPixelBits = 2;
		}
		else {
			GALLEN_DBGLOCAL_RUNLOG(17);
			iPixelBits = 1;
		}
		giPixelBits = iPixelBits;
	}break;
	case S1D13522_REGADDR_GENCONF:GALLEN_DBGLOCAL_RUNLOG(18);
	{
		GALLEN_DBGLOCAL_RUNLOG(19);
		gwS1d13522_GenConf = wValue;
		
		//wRotate = ((gwS1d13522_GenConf>>4) & 0x0003);
	}break;
	case S1D13522_REGADDR_HOSTMEMPIXSWAP:GALLEN_DBGLOCAL_RUNLOG(20);
	{
		
		gwS1d13522_hostmempixswapconf = wValue;
		if(0x0000==(gwS1d13522_hostmempixswapconf&0x0003)) {
			GALLEN_DBGLOCAL_RUNLOG(21);
			epdfbdc_set_host_dataswap(gptEPD_dc_current,0);
		}
		else if(0x0003==(gwS1d13522_hostmempixswapconf&0x0003)) {
			GALLEN_DBGLOCAL_RUNLOG(22);
			epdfbdc_set_host_dataswap(gptEPD_dc_current,1);
		}
	}break;
	default :
		WARNING_MSG("%s %d(0x%x) not support !\n",__FUNCTION__,wIndex,wIndex);
		GALLEN_DBGLOCAL_RUNLOG(23);
		break;
	}
	
	GALLEN_DBGLOCAL_END();
	return tRet;
}

static u16 S1D13522_reg_read(u16 wIndex) 
{
	u16 wRet = 0;
	
	//DBG_MSG("%s(0x%x)\n",__FUNCTION__,wIndex);
	switch (wIndex) {
	case S1D13522_REGADDR_HOSTIFMACONF:
		wRet = ConfigPVI.Reg0x140;
		break;
		
	case S1D13522_REGADDR_PRODUCTCODE:
		wRet = S1D13522_PRODUCTCODE;
		break;
		
	case S1D13522_REGADDR_LUTSTATUS:
		ASSERT(gptEPD_dc_current);
		ASSERT(gptEPD_dc_current->pfnIsUpdating);
		if(gptEPD_dc_current->pfnIsUpdating()) {
			wRet = 0xffff;
		}
		else {
			wRet = 0x0000;
		}
		break;
		
	case S1D13522_REGADDR_GENCONF:
		wRet = gwS1d13522_GenConf;
		break;
	case S1D13522_REGADDR_HOSTMEMPIXSWAP:
		wRet = gwS1d13522_hostmempixswapconf;
		break;
	
	case S1D13522_REGADDR_PUBC:
		wRet = ConfigPVI.Reg0x330;
		break;
	case S1D13522_REGADDR_DSPENGINTRSR:
		wRet = gwS1d13522_DspEngIntrRSR;
		break;
	
	default :
		WARNING_MSG("%s %d(0x%x) not support !\n",__FUNCTION__,wIndex,wIndex);
		break;
	}
	return wRet;
}



#ifdef _PVI_IOCTL_INTERFACE//[
//////////////////////////////////////////////////////////////
//
// pvi ioctl interface ...
// 

// pvi ioctl helper functions ... 

static WORD packed_bpp2_to_bbp3_dot3[] = {
	(0x0 << 1),
	(0x2 << 1),
	(0x5 << 1),
	(0x7 << 1)
};

static WORD packed_bpp2_to_bbp3_dot2[] = {
	(0x0 << (1+4)),
	(0x2 << (1+4)),
	(0x5 << (1+4)),
	(0x7 << (1+4))
};

static WORD packed_bpp2_to_bbp3_dot1[] = {
	(0x0 << (1+4+4)),
	(0x2 << (1+4+4)),
	(0x5 << (1+4+4)),
	(0x7 << (1+4+4))
};

static WORD packed_bpp2_to_bbp3_dot0[] = {
	(0x0 << (1+4+4+4)),
	(0x2 << (1+4+4+4)),
	(0x5 << (1+4+4+4)),
	(0x7 << (1+4+4+4))
};

static WORD packed_bpp1_to_bbp3_dot3[] = {
	(0x0 << 1),
	(0x7 << 1)
};

static WORD packed_bpp1_to_bbp3_dot2[] = {
	(0x0 << (1+4)),
	(0x7 << (1+4))
};

static WORD packed_bpp1_to_bbp3_dot1[] = {
	(0x0 << (1+4+4)),
	(0x7 << (1+4+4))
};

static WORD packed_bpp1_to_bbp3_dot0[] = {
	(0x0 << (1+4+4+4)),
	(0x7 << (1+4+4+4))
};



static BOOL SPI_FlashWaitReady(VOID)
{
  return TRUE;
}

static EN_EPSON_ERROR_CODE PutImage_PVI2EPSON(PBYTE pData, BOOL fPartial, UINT16 WaveForm)
{
int length, cc;
s1d13521_ioctl_cmd_params CommandParam;
PBYTE   pSource;
PUINT16 pDest;
BYTE data;
//EN_EPSON_ERROR_CODE ret;
EN_EPSON_ERROR_CODE ret=_EPSON_ERROR_SUCCESS;
// int save_index=-1; // KEG 20090914 commented out to stop compiler warning 

	//DBG_MSG("%s() skip \n",__FUNCTION__);
	//return _EPSON_ERROR_SUCCESS;
	
	#if 0//[ gallen remove .
  if(BusWaitForHRDY() != _EPSON_ERROR_SUCCESS)
  	return _EPSON_ERROR_NOT_READY;
  	#endif //]

// Joseph 100714, clear soft reset flag
//  ConfigPVI.Reg0x330=BusIssueReadReg(0x330);
  ConfigPVI.Reg0x330=BusIssueReadReg(0x330)&0x00FF;

   ret=BusIssueWriteReg(0x330, 0x84);// LUT auto,auto waveform disable ...
  if(ret != _EPSON_ERROR_SUCCESS)
  	{
	    //DEBUGMSG(_MSG_ERROR, "%s] write 0x330=0x0x84 fail [0x330]=0x%04x !!!!!!!!!!!!\n", __FUNCTION__, BusIssueReadReg(0x330));
  	  goto _PutImage_PVI2EPSON_Fail;
  	}
	DBG_MSG("%s() : CurRotateMod=0x%x,PastRotateMod=0x%x\n",__FUNCTION__,ConfigPVI.CurRotateMode,ConfigPVI.PastRotateMode);
  if(ConfigPVI.PastRotateMode != ConfigPVI.CurRotateMode)
  	{
		epdfb_rotate_t tRotateInPVI_modA[4] = {epdfb_rotate_0,epdfb_rotate_270,epdfb_rotate_180,epdfb_rotate_90};
	  	CommandParam.param[0]=ConfigPVI.CurRotateMode;
	  	
   	  //ret=BusIssueCmd(INIT_ROTMODE,  &CommandParam, 1);
 		DBG_MSG("%s(%d):rotate change %d=>%d\n",__FILE__,__LINE__,\
			gtRotate,tRotateInPVI_modA[(epdfb_rotate_t)(ConfigPVI.CurRotateMode>>8)]);
  	  
   	  gtRotate = tRotateInPVI_modA[(epdfb_rotate_t)(ConfigPVI.CurRotateMode>>8)];
    	  //gtRotate = (epdfb_rotate_t)(ConfigPVI.CurRotateMode>>8);
  	  
   	  
	  	ConfigPVI.PastRotateMode=ConfigPVI.CurRotateMode;
  	}


	CommandParam.param[0] = (0x2 << 4);// image 4 bits/pixel .
	if(fPartial == FALSE)
	{
		length=(gwScrH*gwScrW) >> 2;
	}
	else
	{
		// gallen remove : CommandParam.param[1]=ConfigPVI.StartX;
		// gallen remove : CommandParam.param[2]=ConfigPVI.StartY;
		// gallen remove : CommandParam.param[3]=ConfigPVI.Width;
		// gallen remove : CommandParam.param[4]=ConfigPVI.Height;
		// gallen remove : ret=BusIssueCmd(LD_IMG_AREA, &CommandParam, 5);
		length=(ConfigPVI.Width*ConfigPVI.Height) >> 2;
	}
	if(ret != _EPSON_ERROR_SUCCESS)
	{
		//DEBUGMSG(_MSG_ERROR, "%s] LD_IMG/LD_IMG_AREA fail !!!!!!!!!!!!\n", __FUNCTION__);
		goto _PutImage_PVI2EPSON_Fail;
	}
	pSource=(PBYTE) pData;
	
	
	DBG_MSG("%s() : ConfigPVI.Deepth=0x%x,ReverseGrade=0x%x\n",\
		__FUNCTION__,ConfigPVI.Deepth,ConfigPVI.ReverseGrade);
 
 	#if 0//[ gallen remove 20110315: no more hardware check ,this is fake hardware .
  //pDest=(PUINT16) s1d13521fb_info.VirtualFramebufferAddr;
  pDest=(PUINT16) gbTemp4BitsFBA;
  
  if(ConfigPVI.Deepth == 2) {
    for(cc=length; cc--; pDest++, pSource++)
      {
        data=*pSource ^ ConfigPVI.ReverseGrade;
  	    *pDest=(packed_bpp2_to_bbp3_dot0[(data >> 0) & 3]
  			  | packed_bpp2_to_bbp3_dot1[(data >> 2) & 3]
  			  | packed_bpp2_to_bbp3_dot2[(data >> 4) & 3]
  			  | packed_bpp2_to_bbp3_dot3[(data >> 6) & 3]);
      }
  }
  else {
    for(cc=length >> 1; cc--; pDest++, pSource++)
      {
        data=*pSource ^ ConfigPVI.ReverseGrade;
  	    *pDest=(packed_bpp1_to_bbp3_dot0[(data >> 0) & 1]
  			  | packed_bpp1_to_bbp3_dot1[(data >> 1) & 1]
  			  | packed_bpp1_to_bbp3_dot2[(data >> 2) & 1]
  			  | packed_bpp1_to_bbp3_dot3[(data >> 3) & 1]);
  		pDest++;
  	    *pDest=(packed_bpp1_to_bbp3_dot0[(data >> 4) & 1]
  			  | packed_bpp1_to_bbp3_dot1[(data >> 5) & 1]
  			  | packed_bpp1_to_bbp3_dot2[(data >> 6) & 1]
  			  | packed_bpp1_to_bbp3_dot3[(data >> 7) & 1]);
      }
  }
      
 ret=BusIssueWriteRegBuf(0x154, (PUINT16) s1d13521fb_info.VirtualFramebufferAddr, length);
  if(ret != _EPSON_ERROR_SUCCESS)
  	{
	    //DEBUGMSG(_MSG_ERROR, "%s] Download image fail !!!!!!!!!!!!\n", __FUNCTION__);
  	  goto _PutImage_PVI2EPSON_Fail;
  	}
  ret=BusIssueCmd(LD_IMG_END, &CommandParam, 0);
  if(ret != _EPSON_ERROR_SUCCESS)
  	{
	    //DEBUGMSG(_MSG_ERROR, "%s] LD_IMG_END fail !!!!!!!!!!!!\n", __FUNCTION__);
  	  goto _PutImage_PVI2EPSON_Fail;
  	}
  // FRANKLIN ret=WaitDPEIdle();
  ret=WaitDPEIdle();
  EpsonReady();	// Joseph 100222
  if(fPartial == false) {
	  //mdelay (1000);
		_msleep(1000);
	}
  else {
	  //mdelay (100);
		_msleep(100);
	}

  CommandParam.param[0] = (WaveForm << 8);
  if(fPartial == FALSE)
  	{
#if (defined(__5_INCH__) || defined(__6_INCH__))
printk ("[%s-%d] ............\n",__func__,__LINE__);
      ret=BusIssueCmd(UPD_FULL, &CommandParam, 1);
	  if (g_is_s1d13522) {
	      BusIssueCmd(WAIT_DSPE_TRG,&CommandParam,0);
	      BusIssueCmd(WAIT_DSPE_FREND,&CommandParam,0);
      }
#elif defined(__8_INCH__)
#elif defined(__9D7_INCH__)
  	  CommandParam.param[1]=(1200-800)/2;
  	  CommandParam.param[2]=(826-600)/2;
  	  CommandParam.param[3]=800;
  	  CommandParam.param[4]=600;
   	  ret=BusIssueCmd(UPD_PART_AREA, &CommandParam, 5);
	  if (g_is_s1d13522) 
      	BusIssueCmd(WAIT_DSPE_TRG,&CommandParam,0);
#endif
  	}
  else
  	{
  	  CommandParam.param[1]=ConfigPVI.StartX;
  	  CommandParam.param[2]=ConfigPVI.StartY;
  	  CommandParam.param[3]=ConfigPVI.Width;
  	  CommandParam.param[4]=ConfigPVI.Height;
   	  ret=BusIssueCmd(UPD_PART_AREA, &CommandParam, 5);
	  if (g_is_s1d13522) 
      	BusIssueCmd(WAIT_DSPE_TRG,&CommandParam,0);
  	}
  if(ret != _EPSON_ERROR_SUCCESS)
  	{
  		//DEBUGMSG(_MSG_ERROR, "%s] UPD_FULL fail !!!!!!!!!!!!!!\n", __FUNCTION__);
  	  goto _PutImage_PVI2EPSON_Fail;
  	}
  BusUpdateThermal();			// adon, 2009-04-09
  ret=BusIssueWriteReg(0x330, ConfigPVI.Reg0x330);
  if(ret != _EPSON_ERROR_SUCCESS)
  	{
  		//DEBUGMSG(_MSG_ERROR, "%s] update 0x330 fail !!!!!!!!!!!!!!\n", __FUNCTION__);
  	  goto _PutImage_PVI2EPSON_Fail;
  	}
  	#else //][ gallen 
  	//gptEPD_dc_current->pfnIsUpdating
	printk("%s() : w.f=%d,partial=%d\n",__FUNCTION__,WaveForm,fPartial);
	fake_s1d13522_setwfmode(gptEPD_dc_current,(int)WaveForm);
	if(gptEPD_dc_current->pfnSetPartialUpdate) {
		//DBG_MSG("[%s] %s update mode\n",__FUNCTION__,fPartial?"partial":"full");
		gptEPD_dc_current->pfnSetPartialUpdate(fPartial);
	}
		
  ret=BusIssueWriteReg(0x330, ConfigPVI.Reg0x330);
  
	#if 0

	fake_s1d13522_display_img(ConfigPVI.StartX,ConfigPVI.StartY,
		ConfigPVI.Width,ConfigPVI.Height,gbTemp4BitsFBA,
		gptEPD_dc_current,4,gtRotate);
	#else
	fake_s1d13522_display_img(ConfigPVI.StartX,ConfigPVI.StartY,
		ConfigPVI.Width,ConfigPVI.Height,pSource,
		gptEPD_dc_current,ConfigPVI.Deepth,gtRotate);
	#endif
	
	#endif //] gallen remove 20110315.
	
	
_PutImage_PVI2EPSON_Fail:
  ret=BusIssueWriteReg(0x310,0xa600);
  ret=BusIssueWriteReg(0x312, 0x0e);  
  return ret;
}

static BOOL SPI_FlashWriteData(BYTE Data)
{
  BusIssueWriteReg(_EPSON_REG_SPI_WRITE_DATA, (Data | _EPSON_REG_SPI_WRITE_DATA_ENABLE));
  return SPI_FlashWaitReady();
}
static BOOL SPI_FlashWriteDummy(VOID)
{
  BusIssueWriteReg(_EPSON_REG_SPI_WRITE_DATA, 0x00);
  return SPI_FlashWaitReady();
}

/////
#if 1
static void pvi_DoTimerForcedRefresh(ULONG dummy)
{
  if(ConfigPVI.fNormalMode == FALSE)
  	return;
  pvi_ioctl_ForcedRefresh(NULL);
  //pvi_refersh_timer.expires = jiffies+HZ*ConfigPVI.AutoRefreshTimer;
  //add_timer();
  mod_timer(&pvi_refersh_timer,jiffies+HZ*ConfigPVI.AutoRefreshTimer);
}

int pvi_Init(VOID)
{
  int temp,ret;
  
	if(gpbLOGO_vaddr&&gdwLOGO_size>=_PVI_IMAGE_SIZE) {
		ImagePVI = gpbLOGO_vaddr;
	}
	else {
		ImagePVI = kmalloc(_PVI_IMAGE_SIZE,GFP_KERNEL);
		ASSERT(ImagePVI);
	}
  
  ConfigPVI.fAutoRefreshMode=FALSE;
  init_timer(&pvi_refersh_timer);
  pvi_refersh_timer.function=pvi_DoTimerForcedRefresh;
  pvi_ioctl_Reset(NULL);
  ConfigPVI.fNormalMode=true;

/*	Joseph 100222
  //Battery Low is GPG1
  temp=__raw_readl(S3C2410_GPGCON);
  temp&=~(0x03<<2);
  temp|=0x01<<2;
  //20090803 david use to check GPG1
  printk("GPGCON1=%x",temp);
  __raw_writel(temp, S3C2410_GPGCON);
*/
  return 0;
}

void pvi_Deinit(VOID)
{
	if(ConfigPVI.fAutoRefreshMode != FALSE) {
		del_timer_sync(&pvi_refersh_timer);
	}

	if(ImagePVI&&ImagePVI!=gpbLOGO_vaddr) {
		kfree(ImagePVI);ImagePVI=0;
	}
}
#endif // FRANKLIN

int pvi_ioctl_DisplayImage(PTDisplayCommand puDisplayCommand)
{
  //char buf[128];

	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

	ASSERT(gptEPD_dc_current);
	ASSERT(gptEPD_dc_current->pfnGetWaveformBpp);
	
	// gallen add 20110323 [ 
	ConfigPVI.StartX = 0;
	ConfigPVI.StartY = 0;
	if( ((ConfigPVI.CurRotateMode)==0x00) || 
		((ConfigPVI.CurRotateMode)==0x02<<8) ) 
	{
		ConfigPVI.Width = gptEPD_dc_current->dwWidth;
		ConfigPVI.Height = gptEPD_dc_current->dwHeight;
	}
	else {
		ConfigPVI.Width = gptEPD_dc_current->dwHeight;
		ConfigPVI.Height = gptEPD_dc_current->dwWidth;
	}
	
	DBG_MSG("%s(): rotate=0x%x,w=%d,h=%d\n",__FUNCTION__,\
		ConfigPVI.CurRotateMode,ConfigPVI.Width,ConfigPVI.Height);
	if (ConfigPVI.Width & 1) {
		ConfigPVI.Width += 1;
	}
	//]
	
	if(4==gptEPD_dc_current->pfnGetWaveformBpp()){//this is 4bit
	   printk("4bit waveform mark!\n");
	   PutImage_PVI2EPSON(ImagePVI, FALSE, WF_MODE_GU);
   }else{
	   printk("3bit waveform mark!\n");       
	   PutImage_PVI2EPSON(ImagePVI, FALSE, WF_MODE_GC);       
   }

  return 0;
}

int pvi_ioctl_NewImage(PTDisplayCommand puDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}
	//DEBUGMSG(_MSG_SEND_COMMAND, "dc_NewImage");
	dbgENTER();
	if(!gptHWCFG||0==gptHWCFG->m_val.bDisplayResolution) {
		memcpy(ImagePVI, puDisplayCommand->Data, _PVI_IMAGE_SIZE_800x600);
	}
	else {
		memcpy(ImagePVI, puDisplayCommand->Data, _PVI_IMAGE_SIZE);
	}
	dbgLEAVE();
	return 0;
}

int pvi_ioctl_StopNewImage(PTDisplayCommand puDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}
	return 0;
}


int pvi_ioctl_PartialImage(PTDisplayCommand puDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}
  ConfigPVI.StartX=(puDisplayCommand->Data  [0] << 8)+puDisplayCommand->Data[1];
  ConfigPVI.StartY=(puDisplayCommand->Data  [2] << 8)+puDisplayCommand->Data[3];
  ConfigPVI.Width =((puDisplayCommand->Data [4] << 8)+puDisplayCommand->Data[5])+1-ConfigPVI.StartX;
  ConfigPVI.Height=((puDisplayCommand->Data [6] << 8)+puDisplayCommand->Data[7])+1-ConfigPVI.StartY;
  //DEBUGMSG(_MSG_SEND_COMMAND, "X=%u ", ConfigPVI.StartX);
  //DEBUGMSG(_MSG_SEND_COMMAND, "Y=%u ", ConfigPVI.StartY);
  //DEBUGMSG(_MSG_SEND_COMMAND, "W=%u ", ConfigPVI.Width);
  //DEBUGMSG(_MSG_SEND_COMMAND, "H=%u",  ConfigPVI.Height);
  /////////////////////////////////////////////////////////
  // align pexil to modula 4
	//DEBUGMSG(_MSG_SEND_COMMAND, " [dc_PartialImage end 1] %u", ConfigPVI.Height*((ConfigPVI.Width+3) & ~3)/4);
  memcpy(ImagePVI, &puDisplayCommand->Data[8], ConfigPVI.Height*((ConfigPVI.Width+3) & ~3)/4);
	//DEBUGMSG(_MSG_SEND_COMMAND, " [dc_PartialImage end 2] ");
  return 0;
}

int pvi_ioctl_DisplayPartial(PTDisplayCommand puDisplayCommand)
{
//	printk("pvi_ioctl_DisplayPartial()\n");
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}
	//DEBUGMSG(1, "zzzzz\n");
#if 0
	BusIssueWriteReg(0x360, 0x1001);
  BusIssueWriteReg(0x364, 0x1001);
  PutImage_PVI2EPSON(ImagePVI, TRUE, WF_MODE_MU);
	//DEBUGMSG(1, " [0x360]=0x%04x\n", BusIssueReadReg(0x360));
	//DEBUGMSG(1, " [0x364]=0x%04x\n", BusIssueReadReg(0x364));
	//DEBUGMSG(1, " [0x368]=0x%04x\n", BusIssueReadReg(0x368));
	//DEBUGMSG(1, " [0x36c]=0x%04x\n", BusIssueReadReg(0x36c));
#else
  PutImage_PVI2EPSON(ImagePVI, TRUE, WF_MODE_MU);
#endif
  return 0;
}

int pvi_ioctl_DisplayPartialGU(PTDisplayCommand puDisplayCommand)
{
//	printk("pvi_ioctl_DisplayPartial()\n");
  if(ConfigPVI.fNormalMode == FALSE)
  	return -EFAULT;
	//DEBUGMSG(1, "yyyyy\n");
  //if (!g_is_s1d13522) 
  {
	BusIssueWriteReg(0x360, 0x1001);
  	BusIssueWriteReg(0x364, 0x1001);
  }
  PutImage_PVI2EPSON(ImagePVI, TRUE, WF_MODE_GU);
#ifndef _S1D13522_	//Joseph 100629
	//DEBUGMSG(1, " [0x360]=0x%04x\n", BusIssueReadReg(0x360));
	//DEBUGMSG(1, " [0x364]=0x%04x\n", BusIssueReadReg(0x364));
	//DEBUGMSG(1, " [0x368]=0x%04x\n", BusIssueReadReg(0x368));
	//DEBUGMSG(1, " [0x36c]=0x%04x\n", BusIssueReadReg(0x36c));
#endif
  return 0;
}

int pvi_ioctl_Reset(PTDisplayCommand puDisplayCommand)
{
	ConfigPVI.PastRotateMode = 0xffff;
	ConfigPVI.CurRotateMode=(_INIT_ROTMODE << 8);
	ConfigPVI.ReverseGrade=0;
	ConfigPVI.Deepth=2;
	ConfigPVI.fNormalMode=FALSE;
	ConfigPVI.AutoRefreshTimer=100;
	ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
	if(ConfigPVI.fAutoRefreshMode != FALSE) {
		del_timer_sync(&pvi_refersh_timer);
	}
	//printk("\n==== [%s] 0x0a=0x%04x====\n", __FUNCTION__, BusIssueReadReg(0x0a));
	return 0;
}

int pvi_ioctl_SetDepth(PTDisplayCommand puDisplayCommand)
{
int RetCode=0;
	printk("%s()\n",__FUNCTION__);

	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}
		
	if((puDisplayCommand->Data[0] == 0) || (puDisplayCommand->Data[0] == 2)) {
		ConfigPVI.Deepth=puDisplayCommand->Data[0];
	}
	else {
		ERR_MSG("%s() : parameter value error (0x%x)!\n",__FUNCTION__,puDisplayCommand->Data[0]);
		RetCode=-EFAULT;
	}
	return RetCode;
}

int pvi_ioctl_EraseDisplay(PTDisplayCommand pDisplayCommand)
{
BYTE Color=0xff;

	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  switch(pDisplayCommand->Data[0]) {
  	case 0:
  		Color=0x00;
  		break;
  	case 1:
  		Color=0xff;
  		break;
  	case 2:
  		Color=0x55;
  		break;
  	case 3:
  		Color=0xaa;
  		break;
  };
  memset(ImagePVI, Color, _PVI_IMAGE_SIZE);
  PutImage_PVI2EPSON(ImagePVI, FALSE, WF_MODE_GC);
//  PutImage_PVI2EPSON(ImagePVI, FALSE, WF_MODE_MU);
  return 0;
}

int pvi_ioctl_Rotate(PTDisplayCommand puDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}
	DBG_MSG("%s():puDisplayCommand->Data[0]=0x%02x \n",__FUNCTION__,puDisplayCommand->Data[0]);

	switch(puDisplayCommand->Data[0]) {
	case 1:
		ConfigPVI.CurRotateMode=3<<8;
		break;
	case 2:
		ConfigPVI.CurRotateMode=2<<8;
		break;
	case 3:
		ConfigPVI.CurRotateMode=1<<8;
		break;
	case 0:
		ConfigPVI.CurRotateMode=0<<8;
		break;
	};
	return 0;
}

int pvi_ioctl_Positive(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  ConfigPVI.ReverseGrade=0x0000;
  return 0;
}

int pvi_ioctl_Negative(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  ConfigPVI.ReverseGrade=0xffff;
  return 0;
}

int pvi_ioctl_GoToNormal(PTDisplayCommand pDisplayCommand)
{
	DBG_MSG("%s()\n",__FUNCTION__);

	//BusIssueWriteRegX(0x0a, BusIssueReadReg(0x0a) | 0x1000);
	//	printk("pvi_ioctl_GoToNormal\n");
	//BusIssueWriteReg(0x16, 0);
	//	printk("pvi_ioctl_GoToNormal\n");
	pvi_ioctl_AutoRefreshOn(NULL);
	ConfigPVI.fNormalMode=TRUE;
	//BusIssueCmd(RUN_SYS, NULL, 0);
	return 0;
}
int pvi_ioctl_GoToSleep(PTDisplayCommand pDisplayCommand)
{
	DBG_MSG("\n===== %s() =====\n",__FUNCTION__);
	
	ConfigPVI.fNormalMode=FALSE;
	if(ConfigPVI.fAutoRefreshMode != FALSE) {
		del_timer_sync(&pvi_refersh_timer);
	}
//printk("+pvi_ioctl_GoToSleep\n");
  //BusIssueCmd(SLP, NULL, 0);
//printk("-pvi_ioctl_GoToSleep\n");
  return 0;
}

int pvi_ioctl_GoToStandBy(PTDisplayCommand pDisplayCommand)
{
	DBG_MSG("\n====== %s() =======\n",__FUNCTION__);
	ConfigPVI.fNormalMode=FALSE;
	//BusIssueCmd(STBY, NULL, 0);
	return 0;
}

int pvi_ioctl_WriteToFlash(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  switch(ConfigPVI.StepSPI) {
  	case _STEP_NOR_BYPASS:
  	//DEBUGMSG(_MSG_SPI_WRITE, "_STEP_NOR_BYPASS ");
	  if((pDisplayCommand->Data[0] == 0x00) && (pDisplayCommand->Data[1] == 0x0a) && (pDisplayCommand->Data[2] == 0xaa) && (pDisplayCommand->Data[3] == 0xaa))
  	  	ConfigPVI.StepSPI=_STEP_NOR_WRITE_AA_INTO_555;
  	  break;

  	case _STEP_NOR_WRITE_AA_INTO_555:
  	//DEBUGMSG(_MSG_SPI_WRITE, "_STEP_NOR_WRITE_AA_INTO_555 ");
	  if((pDisplayCommand->Data[0] == 0x00) && (pDisplayCommand->Data[1] == 0x05) && (pDisplayCommand->Data[2] == 0x55) && (pDisplayCommand->Data[3] == 0x55))
  	  	ConfigPVI.StepSPI=_STEP_NOR_WRITE_55_INTO_2AA;
  	  else
  	  	ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
  	  break;

  	case _STEP_NOR_WRITE_55_INTO_2AA:
  	//DEBUGMSG(_MSG_SPI_WRITE, "_STEP_NOR_WRITE_55_INTO_2AA ");
	  if((pDisplayCommand->Data[0] == 0x00) && (pDisplayCommand->Data[1] == 0x0a) && (pDisplayCommand->Data[2] == 0xaa))
	  	{
	  	  if(pDisplayCommand->Data[3] == 0x90)
  	  		ConfigPVI.StepSPI=_STEP_NOR_CMD_STATUS;
	  	  else if(pDisplayCommand->Data[3] == 0xa0)
  	  		ConfigPVI.StepSPI=_STEP_NOR_CMD_PROGRAM;
	  	  else if(pDisplayCommand->Data[3] == 0x80)
  	  		ConfigPVI.StepSPI=_STEP_NOR_CMD_ERASE;
	  	  else
  	  		ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
	  	}
  	  else
  	  	ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
  	  break;

  	case _STEP_NOR_CMD_PROGRAM:
  	//DEBUGMSG(_MSG_SPI_WRITE, "_STEP_NOR_CMD_PROGRAM ");
  	  //spi_FlashWriteByte(pDisplayCommand);
  	  ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
  	  break;

  	case _STEP_NOR_CMD_ERASE:
  	//DEBUGMSG(_MSG_SPI_WRITE, "_STEP_NOR_CMD_ERASE ");
	  if((pDisplayCommand->Data[0] == 0x00) && (pDisplayCommand->Data[1] == 0x0a) && (pDisplayCommand->Data[2] == 0xaa) && (pDisplayCommand->Data[3] == 0xaa))
  	  	ConfigPVI.StepSPI=_STEP_NOR_CMD_ERASE_WRITE_AA_INTO_555;
  	  else
  	  	ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
  	  break;

  	case _STEP_NOR_CMD_ERASE_WRITE_AA_INTO_555:
  	//DEBUGMSG(_MSG_SPI_WRITE, "_STEP_NOR_CMD_ERASE_WRITE_AA_INTO_555 ");
	  if((pDisplayCommand->Data[0] == 0x00) && (pDisplayCommand->Data[1] == 0x05) && (pDisplayCommand->Data[2] == 0x55) && (pDisplayCommand->Data[3] == 0x55))
  	  	ConfigPVI.StepSPI=_STEP_NOR_CMD_ERASE_WRITE_55_INTO_2AA;
  	  else
  	  	ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
  	  break;

  	case _STEP_NOR_CMD_ERASE_WRITE_55_INTO_2AA:
  	//DEBUGMSG(_MSG_SPI_WRITE, "_STEP_NOR_CMD_ERASE_WRITE_55_INTO_2AA ");
	  if((pDisplayCommand->Data[0] == 0x00) && (pDisplayCommand->Data[1] == 0x0a) && (pDisplayCommand->Data[2] == 0xaa) && (pDisplayCommand->Data[3] == 0x10)) {
  	  	//spi_FlashEraseChip();
		}
	  else if(pDisplayCommand->Data[3] == 0x30) {
  	  	//spi_FlashEraseSector(pDisplayCommand);
		}
  	  ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
  	  break;

  	case _STEP_NOR_CMD_STATUS:
  	//DEBUGMSG(_MSG_SPI_WRITE, "_STEP_NOR_CMD_STATUS ");
	  if(pDisplayCommand->Data[3] == 0xf0)
  	  	ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
  	  break;

	default:
  	//DEBUGMSG(_MSG_SPI_WRITE, "_STEP_UNDEFINE ");
	  ConfigPVI.StepSPI=_STEP_NOR_BYPASS;
  };
  return 0;
}

int pvi_ioctl_ReadFromFlash(PTDisplayCommand pDisplayCommand)
{
int RetCode=-EFAULT;

	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  if(ConfigPVI.StepSPI == _STEP_NOR_CMD_STATUS)
  	{
  	  RetCode=0;
  	  if((pDisplayCommand->Data[2] & 3) == 0)
  		pDisplayCommand->Data[0]=0x3e;
  	  else if((pDisplayCommand->Data[2] & 3) == 1)
  		pDisplayCommand->Data[0]=0xc2;
  	  else
  		pDisplayCommand->Data[0]=0;
  	  return RetCode;
  	}
  	#if 0// gallen disable .
  SPI_FlashSwitchToHost();
  if(SPI_FlashWriteData(0x03) == FALSE)
  	goto  _pvi_ioctl_ReadFromFlashFail;
  if(SPI_FlashWriteData(pDisplayCommand->Data[0]) == FALSE)
  	goto  _pvi_ioctl_ReadFromFlashFail;
  if(SPI_FlashWriteData(pDisplayCommand->Data[1]) == FALSE)
  	goto  _pvi_ioctl_ReadFromFlashFail;
  if(SPI_FlashWriteData(pDisplayCommand->Data[2]) == FALSE)
  	goto  _pvi_ioctl_ReadFromFlashFail;
  if(SPI_FlashWriteDummy() == FALSE)
  	goto  _pvi_ioctl_ReadFromFlashFail;
  pDisplayCommand->Data[0]=(BYTE) BusIssueReadReg(_EPSON_REG_SPI_READ_DATA);
  //DEBUGMSG(_MSG_SPI_READ, "ReadData=0x%02x", pDisplayCommand->Data[0]);
	#endif 
  RetCode=0;
_pvi_ioctl_ReadFromFlashFail:
  //SPI_FlashSwitchToDPE();
  return RetCode;
}

int pvi_ioctl_Init(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

//  BusIssueInitDisplay();
  return 0;
}

int pvi_ioctl_AutoRefreshOn(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  if((ConfigPVI.AutoRefreshTimer != 0) && (ConfigPVI.fAutoRefreshMode == FALSE))
  	{
      //pvi_refersh_timer.expires = jiffies+HZ*ConfigPVI.AutoRefreshTimer;
      //add_timer(&pvi_refersh_timer);
      mod_timer(&pvi_refersh_timer,jiffies+HZ*ConfigPVI.AutoRefreshTimer);
      ConfigPVI.fAutoRefreshMode=TRUE;
  	}
  return 0;
}

int pvi_ioctl_AutoRefreshOff(PTDisplayCommand pDisplayCommand)
{
  del_timer_sync(&pvi_refersh_timer);
  ConfigPVI.fAutoRefreshMode=FALSE;
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  return 0;
}

int pvi_ioctl_SetRefresh(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}
  ConfigPVI.AutoRefreshTimer=pDisplayCommand->Data[0];
  return 0;
}

int pvi_ioctl_ForcedRefresh(PTDisplayCommand pDisplayCommand)
{
#ifdef __TURN_ON_FORCE_DISPLAY__
s1d13521_ioctl_cmd_params CommandParam;

	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  CommandParam.param[0]=((WF_MODE_GC << 8) | 0x4000);
  BusIssueCmd(UPD_FULL, &CommandParam, 1);
//  pvi_ioctl_AutoRefreshOn (NULL);
#endif
  return 0;
}

int pvi_ioctl_GetRefresh(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  pDisplayCommand->Data[0]=ConfigPVI.AutoRefreshTimer;
  return 0;
}

int pvi_ioctl_RestoreImage(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  return 0;
}

int pvi_ioctl_ControllerVersion(PTDisplayCommand puDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}
  puDisplayCommand->Data[0]=0x06;
  return 0;
}

int pvi_ioctl_SoftwareVersion(PTDisplayCommand puDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  puDisplayCommand->Data[0]=0x06;
  return 0;
}

int pvi_ioctl_DisplaySize(PTDisplayCommand puDisplayCommand)
{
  puDisplayCommand->Data[0]=0x22;
  return 0;
}

int pvi_ioctl_GetStatus(PTDisplayCommand puDisplayCommand)
{
  puDisplayCommand->Data[0]=0x06;
  if(ConfigPVI.fNormalMode == FALSE)
    puDisplayCommand->Data[0]|=_BIT0;
  if(ConfigPVI.Deepth != 0)
    puDisplayCommand->Data[0]|=_BIT4;
  return 0;
}

int pvi_ioctl_Temperature(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  return 0;
}

int pvi_ioctl_WriteRegister(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  return 0;
}

int pvi_ioctl_ReadRegister(PTDisplayCommand pDisplayCommand)
{
	if(ConfigPVI.fNormalMode == FALSE) {
		ERR_MSG("[WARNING] %s() : not in normal mode .\n",__FUNCTION__);
		return -EFAULT;
	}

  return 0;
}



int pvi_SwitchCommand(PTDisplayCommand pDisplayCommand)
		//printk("ubStatus =%d\n",PHMSCD_USB_STATUS_FAIL);
{
	static int partial=0;
	
int ReturnCode=0;

	GALLEN_DBGLOCAL_MUTEBEGIN_EX(48);
  DBG0_MSG("%s,cmd=0x%02x,wr=%u,rd=%u\n",__FUNCTION__,pDisplayCommand->Command,
	pDisplayCommand->BytesToWrite,pDisplayCommand->BytesToRead);

 
  lock_kernel();
  switch(pDisplayCommand->Command)  {
		case dc_NewImage:GALLEN_DBGLOCAL_RUNLOG(0);
      ReturnCode=pvi_ioctl_NewImage(pDisplayCommand);
//=============================================================================
// Arron
			partial = 0;
//------------------------------------------------------------------------------
			break;

		case dc_StopNewImage:GALLEN_DBGLOCAL_RUNLOG(1);
      ReturnCode=pvi_ioctl_StopNewImage(pDisplayCommand);
			break;

		case dc_DisplayImage:GALLEN_DBGLOCAL_RUNLOG(2);
		//printk("David: dc_DisplayImage\n");
//==============================================================================
		if(partial==0)
		{
			GALLEN_DBGLOCAL_RUNLOG(3);
      			ReturnCode=pvi_ioctl_DisplayImage(pDisplayCommand);
		}else{
			GALLEN_DBGLOCAL_RUNLOG(4);
      			ReturnCode=pvi_ioctl_DisplayPartial(pDisplayCommand);
		}
		//kay add battery detect
                // FRANKLIN s3c2410_adc_read_once();
		//check HARDWARE BATTERY LOW PIN ( GPF4 )
//#if defined (__COOKIE__)
 		//printk("Check Cookie Battery State.\n");
	
		//pvi_CheckHwBatteryLow();		
//#endif
//------------------------------------------------------------------------------
			break;

		case dc_PartialImage:GALLEN_DBGLOCAL_RUNLOG(5);
                // FRANKLIN s3c2410_adc_read_once();
		  ReturnCode=pvi_ioctl_PartialImage(pDisplayCommand);
//=============================================================================
// Arron
			partial = 1;
//------------------------------------------------------------------------------
			break;

		case dc_DisplayPartial:GALLEN_DBGLOCAL_RUNLOG(6);
      ReturnCode=pvi_ioctl_DisplayPartial(pDisplayCommand);
			break;

		case dc_DisplayPartialGU:GALLEN_DBGLOCAL_RUNLOG(7);
      ReturnCode=pvi_ioctl_DisplayPartialGU(pDisplayCommand);
			break;

		case dc_Reset:GALLEN_DBGLOCAL_RUNLOG(8);
      ReturnCode=pvi_ioctl_Reset(pDisplayCommand);
			break;

		case dc_SetDepth:GALLEN_DBGLOCAL_RUNLOG(9);
     ReturnCode=pvi_ioctl_SetDepth(pDisplayCommand);
			break;

		case dc_EraseDisplay:GALLEN_DBGLOCAL_RUNLOG(10);
      ReturnCode=pvi_ioctl_EraseDisplay(pDisplayCommand);
			break;

		case dc_Rotate:GALLEN_DBGLOCAL_RUNLOG(11);
      ReturnCode=pvi_ioctl_Rotate(pDisplayCommand);
			break;

		case dc_Positive:GALLEN_DBGLOCAL_RUNLOG(12);
      ReturnCode=pvi_ioctl_Positive(pDisplayCommand);
			break;

		case dc_Negative:GALLEN_DBGLOCAL_RUNLOG(13);
      ReturnCode=pvi_ioctl_Negative(pDisplayCommand);
			break;

		case dc_GoToNormal:GALLEN_DBGLOCAL_RUNLOG(14);
      ReturnCode=pvi_ioctl_GoToNormal(pDisplayCommand);
			break;

		case dc_GoToSleep:GALLEN_DBGLOCAL_RUNLOG(15);
     ReturnCode=pvi_ioctl_GoToSleep(pDisplayCommand);
			break;

		case dc_GoToStandBy:GALLEN_DBGLOCAL_RUNLOG(16);
      ReturnCode=pvi_ioctl_GoToStandBy(pDisplayCommand);
			break;

		case dc_WriteToFlash:GALLEN_DBGLOCAL_RUNLOG(17);
      ReturnCode=pvi_ioctl_WriteToFlash(pDisplayCommand);
			break;

		case dc_ReadFromFlash:GALLEN_DBGLOCAL_RUNLOG(18);
      ReturnCode=pvi_ioctl_ReadFromFlash(pDisplayCommand);
			break;

		case dc_Init:GALLEN_DBGLOCAL_RUNLOG(19);
      ReturnCode=pvi_ioctl_Init(pDisplayCommand);
			break;

		case dc_AutoRefreshOn:GALLEN_DBGLOCAL_RUNLOG(20);
      ReturnCode=pvi_ioctl_AutoRefreshOn(pDisplayCommand);
			break;

		case dc_AutoRefreshOff:GALLEN_DBGLOCAL_RUNLOG(21);
      ReturnCode=pvi_ioctl_AutoRefreshOff(pDisplayCommand);
			break;

		case dc_SetRefresh:GALLEN_DBGLOCAL_RUNLOG(22);
     ReturnCode=pvi_ioctl_SetRefresh(pDisplayCommand);
			break;

		case dc_ForcedRefresh:GALLEN_DBGLOCAL_RUNLOG(23);
      ReturnCode=pvi_ioctl_ForcedRefresh(pDisplayCommand);
			break;

		case dc_GetRefresh:GALLEN_DBGLOCAL_RUNLOG(24);
      ReturnCode=pvi_ioctl_GetRefresh(pDisplayCommand);
			break;

		case dc_RestoreImage:GALLEN_DBGLOCAL_RUNLOG(25);
     ReturnCode=pvi_ioctl_RestoreImage(pDisplayCommand);
			break;

		case dc_ControllerVersion:GALLEN_DBGLOCAL_RUNLOG(26);
      ReturnCode=pvi_ioctl_ControllerVersion(pDisplayCommand);
			break;

		case dc_SoftwareVersion:GALLEN_DBGLOCAL_RUNLOG(27);
     ReturnCode=pvi_ioctl_SoftwareVersion(pDisplayCommand);
			break;

		case dc_DisplaySize:GALLEN_DBGLOCAL_RUNLOG(28);
      ReturnCode=pvi_ioctl_DisplaySize(pDisplayCommand);
			break;

		case dc_GetStatus:GALLEN_DBGLOCAL_RUNLOG(29);
      ReturnCode=pvi_ioctl_GetStatus(pDisplayCommand);
			break;

		case dc_Temperature:GALLEN_DBGLOCAL_RUNLOG(30);
      ReturnCode=pvi_ioctl_Temperature(pDisplayCommand);
			break;

		case dc_WriteRegister:GALLEN_DBGLOCAL_RUNLOG(31);
     ReturnCode=pvi_ioctl_WriteRegister(pDisplayCommand);
			break;

		case dc_ReadRegister:GALLEN_DBGLOCAL_RUNLOG(32);
      ReturnCode=pvi_ioctl_ReadRegister(pDisplayCommand);
			break;

	  default:GALLEN_DBGLOCAL_RUNLOG(33);
	    ReturnCode=-EFAULT;
  }
  unlock_kernel();
  GALLEN_DBGLOCAL_END();
  return ReturnCode;
}

#endif //] _PVI_IOCTL_INTERFACE



#ifdef _QT_SUPPORT //[


static void s1d13521fb_resolve_conflict(unsigned long dummy)
{
#ifndef _ES_LUTNOWAIT_
	struct s1d13521_ioctl_hwc ioctl_hwc;
	s1d13521_ioctl_cmd_params cmd_params;
	extern unsigned long gLastUpdParm0;

	if (BusIssueReadReg(0x0a) & 0x20)
	{
//		printk("[%s-%d] \n",__func__,__LINE__);
		if (resolve_conflict_step)
			mod_timer(&conflict_resolution_timer, jiffies+(HZ/10));
		else
			mod_timer(&conflict_resolution_timer, jiffies+(HZ/4));
		return;
	}
//	printk("[%s-%d] \n",__func__,__LINE__);
	switch (resolve_conflict_step) {
	case 0:
		// check the register to see if we have a conflict
		ioctl_hwc.addr = 0x33A; 
		ioctl_hwc.value = BusIssueReadReg(ioctl_hwc.addr);

		if (ioctl_hwc.value & 0x0080)  // if we have a conflict
		{
			resolve_conflict_step++;
			printk("evidentpoint: In %s, conflict found\n", __FUNCTION__);
			// wait for frame end
			BusIssueCmd(WAIT_DSPE_FREND, &cmd_params, 0);
	

			mod_timer(&conflict_resolution_timer, jiffies+(HZ/10));
		}
		break;
	case 1:
		cmd_params.param[0] = gLastUpdParm0;
	//	printk("[%s-%d] update parm 0 0x%04X\n", __FUNCTION__, __LINE__,cmd_params.param[0]);
		BusIssueCmd(UPD_PART,&cmd_params,1);
		mod_timer(&conflict_resolution_timer, jiffies+(HZ/10));
		resolve_conflict_step++;
		break;
	case 2:
		BusIssueWriteReg(0x33A, 0x80);
		printk("[%s-%d] done with 0x%04X\n", __FUNCTION__, __LINE__,gLastUpdParm0);
	default:
		resolve_conflict_step = 0;
		bConflictTimerRunning = FALSE;
		break;
	}
#else
		bConflictTimerRunning = FALSE;
#endif
}

static int ImagePGM(PST_IMAGE_PGM pPGM,EPDFB_DC *pDC)
{
	int ret=-1;

	DBG_MSG("[%s](%d) : w=%ld,h=%ld,r=%d,data=0x%p\n",__FUNCTION__,__LINE__,\
			pPGM->Width,pPGM->Height,gtRotate,pPGM->Data);
	giPixelBits = 8;
	ret = fake_s1d13522_display_img(0,0,pPGM->Width,pPGM->Height,pPGM->Data,
		pDC,8,gtRotate) ;

	return ret;
}



// ES modify for 4 bbp support. 
// Qookie usage
static int Epson_LoadImageArea(PTloadImageArea area,EPDFB_DC *pDC)
{
	unsigned char *pbArea;
	unsigned long dwAreaWidthBytes;
	unsigned long dwFBSize;
	
	unsigned long dwStoreDCFlags = 0;
	int iIsSetDCFlags = 0;
	int iShifBits;
	
	ASSERT(pDC);
	ASSERT(area);
	
	GALLEN_DBGLOCAL_MUTEBEGIN();
	GALLEN_DBGLOCAL_PRINTMSG("%s():x=%d,y=%d,w=%d,h=%d,mode=%d,cmd=%x\n",__FUNCTION__,
		area->XStart,area->YStart,area->Width,area->Height,area->mode,area->cmd);
	
	if(pDC->pfnSetWaveformMode) {
		pDC->pfnSetWaveformMode(area->mode);
	}
	
	if(UPD_FULL==area->cmd) {
		pDC->pfnSetPartialUpdate(0);
	}
	else {
		pDC->pfnSetPartialUpdate(1);
	}
	
	
	if(pDC->pfnGetRealFrameBufEx) {
		GALLEN_DBGLOCAL_RUNLOG(0);
		
		if(!(pDC->dwFlags&EPDFB_DC_FLAG_REVERSEINPDATA)) {
			dwStoreDCFlags = pDC->dwFlags;
			iIsSetDCFlags = 1;
			pDC->dwFlags |= EPDFB_DC_FLAG_REVERSEINPDATA;
		}
		
		pbArea = pDC->pfnGetRealFrameBufEx(&dwFBSize);
		GALLEN_DBGLOCAL_PRINTMSG("fbsize=%u,depth=%d bit\n",dwFBSize,giPixelBits);
		
		switch(giPixelBits) {
		default :GALLEN_DBGLOCAL_RUNLOG(1);
			ERR_MSG("%s:depth %d not supported !\n",__FUNCTION__,giPixelBits);
			GALLEN_DBGLOCAL_ESC();
			return -1;
		case 8:GALLEN_DBGLOCAL_RUNLOG(2);
			iShifBits = 0;
			break;
		case 4:GALLEN_DBGLOCAL_RUNLOG(3);
			iShifBits = 1;
			break;
		case 2:GALLEN_DBGLOCAL_RUNLOG(4);
			iShifBits = 2;
			break;
		case 1:GALLEN_DBGLOCAL_RUNLOG(5);
			iShifBits =	3;
			break;
		}
		
		pbArea += dwFBSize ;

		#if 1
		
		#if 1 //[
		{
			EPDFB_DC *ptEPD_dcimg ;

			ptEPD_dcimg = epdfbdc_create_ex(pDC->dwHeight,pDC->dwWidth,\
				giPixelBits,pbArea,pDC->dwFlags);
			
			epdfbdc_put_dcimg(pDC,ptEPD_dcimg,gtRotate,area->XStart,area->YStart,
				area->Width,area->Height,area->XStart,area->YStart);
			
			epdfbdc_delete(ptEPD_dcimg);
		}
		#else //][!
		{

			int i;
			unsigned char *pbRow;
			EPDFB_IMG tImg;
			
			tImg.dwX = area->XStart;
			tImg.dwW = area->Width;
			tImg.dwH = 1;
			tImg.bPixelBits = (unsigned char)giPixelBits;
			for(i=0;i<area->Height;i++)
			{
				pbRow = pbArea + (area->XStart>>iShifBits)+((area->YStart+i)*(pDC->dwHeight>>iShifBits));
				
				tImg.dwY = area->YStart+i;
				tImg.pbImgBuf = pbRow;
				
				epdfbdc_put_fbimg(pDC,&tImg,gtRotate);
				//fake_s1d13522_display_img(area->XStart,area->YStart+i,area->Width,1,\
				//	pbRow,pDC,giPixelBits,gtRotate);
			}
			

		}
		#endif//]
		
			if(pDC->pfnDispStart) {
				if(pDC->pfnVcomEnable) {
					pDC->pfnVcomEnable(1);
				}

				if(pDC->pfnSetUpdateRect) {
					unsigned long dwX,dwY,dwW,dwH;
					dwX = area->XStart;
					dwY = area->YStart;
					dwW = area->Width;
					dwH = area->Height;
					epdfbdc_get_rotate_active(pDC,&dwX,&dwY,&dwW,&dwH,gtRotate);
		
					pDC->pfnSetUpdateRect(dwX,dwY,dwW,dwH);
				}
				
				pDC->pfnDispStart(1);

				if(pDC->pfnVcomEnable) {
					pDC->pfnVcomEnable(0);
				}
			}
		
		#else
		switch(gtRotate) {
		case epdfb_rotate_0:
		case epdfb_rotate_180:
		case epdfb_rotate_90:
		case epdfb_rotate_270:
			fake_s1d13522_display_img(0,0,pDC->dwHeight,pDC->dwWidth,\
				pbArea,pDC,giPixelBits,gtRotate);
			break;
		}
		#endif
			
		if(iIsSetDCFlags) {
			pDC->dwFlags = dwStoreDCFlags;
		}
	}
		
	GALLEN_DBGLOCAL_END();
	
	return 0;
}
#endif //] _QT_SUPPORT


int fake_s1d13522_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	GALLEN_DBGLOCAL_BEGIN();
	if(gptHWCFG&&1==gptHWCFG->m_val.bDisplayResolution) {
		switch(gtRotate)
		{
		case epdfb_rotate_0:
		case epdfb_rotate_180:
			var->xres = 1024;
			var->yres = 758;
			break;	
		case epdfb_rotate_90:
		case epdfb_rotate_270:
			var->xres = 758;
			var->yres = 1024;
			break;
		}
	}
	else {
		switch(gtRotate)
		{
		case epdfb_rotate_0:
		case epdfb_rotate_180:
			var->xres = 800;
			var->yres = 600;
			break;	
		case epdfb_rotate_90:
		case epdfb_rotate_270:
			var->xres = 600;
			var->yres = 800;
			break;
		}
	}
	
	var->xres_virtual       = var->xres;
	var->yres_virtual       = var->yres;
	var->xoffset            = var->yoffset = 0;
	var->bits_per_pixel     = S1D_DISPLAY_BPP;
	var->grayscale          = 1;
	var->nonstd             = 0;                    /* != 0 Non standard pixel format */
	var->activate           = FB_ACTIVATE_NOW;      /* see FB_ACTIVATE_*             */
	var->height             = -1;                   /* height of picture in mm       */
	var->width              = -1;                   /* width of picture in mm        */
	var->accel_flags        = 0;                    /* acceleration flags (hints     */
	var->pixclock           = S1D_DISPLAY_PCLK;
	var->right_margin       = 0;
	var->lower_margin       = 0;
	var->hsync_len          = 0;
	var->vsync_len          = 0;
	var->left_margin        = 0;
	var->upper_margin       = 0;
	var->sync               = 0;
	var->vmode              = FB_VMODE_NONINTERLACED;
	var->red.msb_right      = var->green.msb_right = var->blue.msb_right = 0;
	var->transp.offset      = var->transp.length = var->transp.msb_right = 0;

	//s1d13521fb_fix.visual = FB_VISUAL_TRUECOLOR;

	switch (info->var.bits_per_pixel)
	{
		case 1:GALLEN_DBGLOCAL_RUNLOG(0);
		case 2:GALLEN_DBGLOCAL_RUNLOG(1);
		case 4:GALLEN_DBGLOCAL_RUNLOG(2);
		case 5:GALLEN_DBGLOCAL_RUNLOG(3);
		case 8:GALLEN_DBGLOCAL_RUNLOG(4);
			var->red.offset  = var->green.offset = var->blue.offset = 0;
			var->red.length  = var->green.length = var->blue.length = S1D_DISPLAY_BPP;
			break;

		default:
			printk(KERN_WARNING "%dbpp is not supported.\n",
					info->var.bits_per_pixel);
			GALLEN_DBGLOCAL_ESC();
			return -EINVAL;
	}

	GALLEN_DBGLOCAL_END();
	return 0;
}

// gallen 2011/05/24 copy from s1d13521base.c .
int fake_s1d13522_setcolreg(unsigned regno, unsigned red, unsigned green,
                                unsigned blue, unsigned transp, struct fb_info *info)
{
#if 1 //[

	printk("%s(): r.no=%02Xh (r,g,b,t)=(%04X,%04X,%04X,%04X)h\n",
					__FUNCTION__, regno, red, green, blue, transp);
	return 0;  // ensure that setcolreg does nothing for now
#else //][
        // Make the first 16 LUT entries available to the console
				 
	if (info->var.bits_per_pixel == 8 && s1d13521fb_fix.visual == FB_VISUAL_TRUECOLOR)
	{
		if (regno < 16)
		{
			// G= 30%R + 59%G + 11%B
			unsigned gray = (red*30 + green*59 + blue*11)/100;
			gray = (gray>>8) & 0xFF;

#ifdef CONFIG_FB_EPSON_BLACK_AND_WHITE
			if (gray != 0) {
				gray = 0xFF;
			}
#endif

			// invert: black on white
			gray = 0xFF-gray;
			((u32*)info->pseudo_palette)[regno] = gray;

			dbg_info("%s(): regno=%02Xh red=%04Xh green=%04Xh blue=%04Xh transp=%04Xh ->gray=%02Xh\n",
					__FUNCTION__, regno, red, green, blue, transp,gray);
		}
	}
	else {
		return 1;
	}

	return 0;
#endif //]
}


int32_t fake_s1d13522_ioctl(unsigned int cmd,unsigned long arg,EPDFB_DC *pDC)
{
	
	int32_t ret=0;
	struct s1d13521_ioctl_hwc ioctl_hwc;
	
	
	GALLEN_DBGLOCAL_MUTEBEGIN_EX(64);
	GALLEN_DBGLOCAL_PRINTMSG("%s(%x,%x)\n",__FUNCTION__,cmd,arg);
	
	#ifdef SHOW_PROGRESS_BAR//[
	gIsProgessRunning = 0;
	#endif //] SHOW_PROGRESS_BAR

	switch(cmd)
	{
	  case S1D13521_DISPLAY:GALLEN_DBGLOCAL_RUNLOG(0);
		{
			
			int iBpp = 4;
			ST_IMAGE_PGM *pt = (ST_IMAGE_PGM *)arg;
			
			
			pt = kmalloc(sizeof(*pt),GFP_KERNEL);
			if(pt) {
				GALLEN_DBGLOCAL_RUNLOG(1);

				if(gptHWCFG&&1==gptHWCFG->m_val.bDisplayResolution) {
					copy_from_user(pt,(ST_IMAGE_PGM *)arg,sizeof(ST_IMAGE_PGM));
					GALLEN_DBGLOCAL_PRINTMSG("DISPLAY INFO : X=%ld,Y=%ld,W=%ld,H=%ld\n\twaveform=%ld,LUT no.=%ld,datap=%p,size=%ld\n",
							pt->StartX,pt->StartY,pt->Width,pt->Height,
							pt->WaveForm,pt->LUT_NO,pt->Data,sizeof(ST_IMAGE_PGM));
				}
				else {
					copy_from_user(pt,(ST_IMAGE_PGM_800x600 *)arg,sizeof(ST_IMAGE_PGM_800x600));
					GALLEN_DBGLOCAL_PRINTMSG("DISPLAY INFO : X=%ld,Y=%ld,W=%ld,H=%ld\n\twaveform=%ld,LUT no.=%ld,datap=%p,size=%ld\n",
							pt->StartX,pt->StartY,pt->Width,pt->Height,
							pt->WaveForm,pt->LUT_NO,pt->Data,sizeof(ST_IMAGE_PGM_800x600));
				}
				
				
				
				_Epson_displayCMD(pt,pDC);

				
				#if 0//(GDEBUG>0)//[
				{
					int iShowCnt = 100,i;
					DBG_MSG("data={{{\n\t");
					for(i=0;i<iShowCnt;i++) {
						DBG_MSG("0x02,",pt->Data[i]);
					}
					DBG_MSG("\n}}}\n");
				}
				#endif //]
				
				/*
				//fr.pointer = ;
				if((ConfigPVI.Reg0x140&0x0030)==0x0030) {
					GALLEN_DBGLOCAL_RUNLOG(0);
					iBpp = 8;
					fr.offset = 0 ;
					fr.count = pt->Width * pt->Height * iBpp / 8;
					par->board->write_gray8(fr.offset,pt->Data,fr.count);
				}
				else {
					GALLEN_DBGLOCAL_RUNLOG(1);
					iBpp = 4;
					fr.offset = 0 ;
					fr.count = pt->Width * pt->Height * iBpp / 8;
					par->board->write(fr.offset,pt->Data,fr.count);
				}
				*/
				
				kfree(pt);
			}
			else {
				ERR_MSG("%s(%d):memory not enough !!\n",__func__,__LINE__);
				GALLEN_DBGLOCAL_RUNLOG(8);
				ret = -ENOMEM;
			}
			
				
		}
		break;

		case S1D13521_REGWRITE:GALLEN_DBGLOCAL_RUNLOG(9);
		{
			struct s1d13521_ioctl_hwc *pIoctl_hwc = &ioctl_hwc;
			
			if (copy_from_user(&ioctl_hwc, arg, sizeof(ioctl_hwc))) {
				GALLEN_DBGLOCAL_ESC();
				return -EFAULT;
			}

			GALLEN_DBGLOCAL_PRINTMSG("REG[%x] <== 0x%04x\n",pIoctl_hwc->addr,pIoctl_hwc->value);
			S1D13522_reg_write(pIoctl_hwc->addr,pIoctl_hwc->value);
		}
		break;

		case S1D13521_REGREAD:GALLEN_DBGLOCAL_RUNLOG(10);
		{
			struct s1d13521_ioctl_hwc *pIoctl_hwc = &ioctl_hwc;
			
			//DBG0_MSG("--- REGREAD 1----\n");
			
			if (copy_from_user(&ioctl_hwc, arg, sizeof(ioctl_hwc))) {
				GALLEN_DBGLOCAL_ESC();
				return -EFAULT;
			}
			//DBG0_MSG("--- REGREAD 2,addr=0x%x----\n",pIoctl_hwc->addr);
			pIoctl_hwc->value = S1D13522_reg_read(pIoctl_hwc->addr);
			if (copy_to_user(arg,&ioctl_hwc,sizeof(ioctl_hwc))) {
				GALLEN_DBGLOCAL_ESC();
				return -EFAULT;
			}
			GALLEN_DBGLOCAL_PRINTMSG("REG[%x] ==> 0x%04x\n",pIoctl_hwc->addr,pIoctl_hwc->value);
		}
		break;

		case S1D13521_FLASH:GALLEN_DBGLOCAL_RUNLOG(11);
		{
			//S1D13532_FLASH_PACKAGE tFlashPack ;
			PS1D13532_FLASH_PACKAGE pFlashPack=(PS1D13532_FLASH_PACKAGE)arg;
			
			//if (copy_from_user(pFlashPack, arg, sizeof(tFlashPack))) {
			//	GALLEN_DBGLOCAL_ESC();
			//	return -EFAULT;
			//}
			
			GALLEN_DBGLOCAL_PRINTMSG("Cmd=0x%x,StartAddr=0x%08x,DataLength=%lu\n,Buf=0x%02x,0x%02x,0x%02x,0x%02x\n",
					pFlashPack->Command,pFlashPack->StartAddr,pFlashPack->DataLength,
					pFlashPack->Buf[0],pFlashPack->Buf[1],pFlashPack->Buf[2],pFlashPack->Buf[3]);
			BusIssueFlashOperation(pFlashPack,pDC);
		}
		break;

		case S1D13521_SETDEPTH:GALLEN_DBGLOCAL_RUNLOG(12);
		{
			s1d13521_ioctl_cmd_params cmd_params ,*pcmd_params = &cmd_params;
			
			if (copy_from_user(pcmd_params, arg, sizeof(cmd_params))) {
				GALLEN_DBGLOCAL_ESC();
				return -EFAULT;
			}
			
			GALLEN_DBGLOCAL_PRINTMSG("S1D13521 SETDEPTH param[0]=%x\n",pcmd_params->param[0]);
		}
		break;
		
		case S1D13521_LOAD_AREA:GALLEN_DBGLOCAL_RUNLOG(13);
		{
			int status =  Epson_LoadImageArea((PTloadImageArea) arg,pDC);
			
			#ifndef _ES_LUTNOWAIT_//[
			// start a timer if there is not one already running.
			 
			while (resolve_conflict_step) {
				msleep (100);
			}
			if (bConflictTimerRunning) {
				GALLEN_DBGLOCAL_RUNLOG(14);
	//		    printk("evidentpoint: Pushing the conflict resolution timer forward\n");
				mod_timer(&conflict_resolution_timer, jiffies+(HZ/4));
			} else {
				GALLEN_DBGLOCAL_RUNLOG(15);
	//		    printk("evidentpoint: Starting the conflict resolution timer\n");
				bConflictTimerRunning = TRUE;
				conflict_resolution_timer.expires = jiffies+(HZ/4);
				add_timer(&conflict_resolution_timer);
			}
			#endif //] _ES_LUTNOWAIT_
			GALLEN_DBGLOCAL_ESC();
			return status;
			// end KEG 20090814

		}
		break;
		case S1D13521_PGM:GALLEN_DBGLOCAL_RUNLOG(16);
			ImagePGM((PST_IMAGE_PGM) arg,pDC);
			GALLEN_DBGLOCAL_ESC();
			return 0;
	
		case S1D13521_WAIT_DSPE_TRG:GALLEN_DBGLOCAL_RUNLOG(18);
			GALLEN_DBGLOCAL_ESC();
			return 0;
		case S1D13521_WAIT_DSPE_FREND:GALLEN_DBGLOCAL_RUNLOG(19);
			pDC->iIsForceWaitUpdateFinished=1;
			GALLEN_DBGLOCAL_ESC();
			return 0;
		
		#ifdef _PVI_IOCTL_INTERFACE //[

		case CMD_SendCommand:GALLEN_DBGLOCAL_RUNLOG(17);
		{
			ret = pvi_SwitchCommand((PTDisplayCommand) arg);
		}
		break;
		#endif //] _PVI_IOCTL_INTERFACE
		
		case EPDC_PWR_CTRL:GALLEN_DBGLOCAL_RUNLOG(20);
		{
			int iPwrCtrlCmd=*((int*)arg);
			DBG_MSG("EPDC_PWR_CTRL %d\n",iPwrCtrlCmd);
			switch(iPwrCtrlCmd) {
			case EPDC_AUTOPWR_INTERVAL_MAX:// set auto off interval max .
			case EPDC_AUTOPWR_INTERVAL_NORMAL:// set auto off interval mormal .
				if(pDC->pfnPwrAutoOffIntervalMax) {
					pDC->pfnPwrAutoOffIntervalMax(EPDC_AUTOPWR_INTERVAL_MAX==iPwrCtrlCmd?1:0);
					ret = 0;
				}
				break;
				
			case EPDC_PWR_ON:
			case EPDC_PWR_OFF:
				if(pDC->pfnPwrOnOff) {
					pDC->pfnPwrOnOff(EPDC_PWR_ON==iPwrCtrlCmd?1:0);
					ret = 0;
				}
				break;
				
			case EPDC_VCOM_ON:
			case EPDC_VCOM_OFF:
				if(pDC->pfnVcomEnable) {
					pDC->pfnVcomEnable(EPDC_VCOM_ON==iPwrCtrlCmd?1:0);
					ret = 0;
				}
				break;
				
			case EPDC_AUTOOFF_ENABLE:
			case EPDC_AUTOOFF_DISABLE:
				if(pDC->pfnAutoOffEnable) {
					pDC->pfnAutoOffEnable(EPDC_AUTOOFF_ENABLE==iPwrCtrlCmd?1:0);
					ret = 0;
				}
				break;
			}
		}
		break;
		
		case EPDC_VCOM_SET:GALLEN_DBGLOCAL_RUNLOG(21);
		{
			int iVCOM_set_val=(int)arg;
			if(pDC->pfnSetVCOM) {
				ret = pDC->pfnSetVCOM(iVCOM_set_val)>=0?0:-ENOTTY;
			}
			else {
				ret = -ENOTTY;
			}
		}
		break;

		case EPDC_VCOM_SET_TO_FLASH:GALLEN_DBGLOCAL_RUNLOG(22);
		{
			int iVCOM_set_val=(int)arg;
			if(pDC->pfnSetVCOMToFlash) {
				ret = pDC->pfnSetVCOMToFlash(iVCOM_set_val)>=0?0:-ENOTTY;
			}
			else {
				ret = -ENOTTY;
			}
		}
		break;

		case EPDC_VCOM_GET:GALLEN_DBGLOCAL_RUNLOG(23);
		{
			int iVCOM_get_val;
			if(pDC->pfnGetVCOM) {
				pDC->pfnGetVCOM(&iVCOM_get_val);
				copy_to_user(arg,&iVCOM_get_val,sizeof(iVCOM_get_val));
			}
			else {
				ret = -ENOTTY;
			}
		}
		break;

		default :
			ERR_MSG("[fake_21d13522] %s() : unsupported cmd (0x%x)\n",__FUNCTION__,cmd);
			ret = -ENOTTY;
			break;
	}


	GALLEN_DBGLOCAL_END();
	return ret;
}

#ifdef SHOW_PROGRESS_BAR //[

static VOID PROGRESS_BAR(EPDFB_DC *pDC)
{
	int i;

	int startX = _PROGRESS_BAR_X_;
	int startY = _PROGRESS_BAR_Y_;
	int icons = _PROGRESS_BAR_ICONS_;
	const int width = 16;
	const int height = 16;
	int space = 20;
	unsigned long dwDrawDataSize ;
	unsigned char *pbDrawData;
	epdfb_rotate_t L_tRotate ;

	if(!gptHWCFG) {
		WARNING_MSG("%s(%d): %s() skip without ntx hwconfig !\n",
			__FILE__,__LINE__,__FUNCTION__);
		return ;
	}
	
	ASSERT(pDC);
	//gIsProgessRunning = 1;
	DBG_MSG("%s() ,progresscnt=%d\n",__FUNCTION__,gIsProgessRunning);

	icons = gptHWCFG->m_val.bProgressCnts;
	startX = (int)(gptHWCFG->m_val.bProgressXHiByte<<8|gptHWCFG->m_val.bProgressXLoByte);
	startY = (int)(gptHWCFG->m_val.bProgressYHiByte<<8|gptHWCFG->m_val.bProgressYLoByte);

	if (! icons) {
		ERR_MSG("[%s-%d] No progess ...\n",__func__,__LINE__);
		return;
	}
	else {
		DBG_MSG("progress icons=%d,x=%d,y=%d\n",icons,startX,startY);
	}


	dwDrawDataSize = width*height;
	pbDrawData = kmalloc(dwDrawDataSize,GFP_KERNEL);
	ASSERT(pbDrawData);

	

	
	if(pDC->pfnWaitUpdateComplete) {
		pDC->pfnWaitUpdateComplete();
	}
	else {
		while(pDC->pfnIsUpdating()) {
			//DBG0_MSG("%s(%d):wait for update done .\n");
			schedule();
		}
	}
	
	msleep(500);
	
	fake_s1d13522_setwfmode(pDC,1);
	pDC->pfnSetPartialUpdate(1);

	if(0==gptHWCFG->m_val.bDisplayPanel||3==gptHWCFG->m_val.bDisplayPanel||\
			6==gptHWCFG->m_val.bDisplayPanel||8==gptHWCFG->m_val.bDisplayPanel)
	{
		L_tRotate = epdfb_rotate_270;
	}
	else {
		L_tRotate = epdfb_rotate_90;
	}
	//DBG0_MSG("pbDrawData=%p,size=%u,rotate=%d\n",pbDrawData,dwDrawDataSize,L_tRotate);


	
	while (gIsProgessRunning) {
		for(i=0;i<icons && gIsProgessRunning;){
		//DBG0_MSG("%s[%d]: rotate=%d\n",__FUNCTION__,__LINE__,L_tRotate);
			if(pDC->pfnIsUpdating()) {
				DBG_MSG("%s[%d]: wait update done ...\n",__FUNCTION__,__LINE__);
				schedule();
				continue;
			}
			else {
				//fake_s1d13522_setwfmode(pDC,1);
				memset(pbDrawData , 0 , dwDrawDataSize);
				fake_s1d13522_display_img(startX+(space*i),startY,width,height,\
					pbDrawData,pDC,4,L_tRotate);
			}
			sleep_on_timeout(&progress_WaitQueue,HZ/2);
			i++;
		}
		for(i=0;i<icons && gIsProgessRunning;){
		//DBG0_MSG("%s[%d]: rotate=%d\n",__FUNCTION__,__LINE__,L_tRotate);
			if(pDC->pfnIsUpdating()) {
				DBG_MSG("%s[%d]: wait update done ...\n",__FUNCTION__,__LINE__);
				schedule();
				continue;
			}
			else {
				//fake_s1d13522_setwfmode(pDC,1);
				memset(pbDrawData , 0xFF , dwDrawDataSize);
				fake_s1d13522_display_img(startX+(space*i)+1,startY+1,\
					width-2,height-2,pbDrawData,pDC,4,L_tRotate);
			}
			sleep_on_timeout(&progress_WaitQueue,HZ/2);
			i++;
		}	      
	}
	kfree(pbDrawData);

}
#endif //] SHOW_PROGRESS_BAR

