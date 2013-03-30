//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson/s1d13521fb.h --
// Function header for Epson S1D13521 controller frame buffer drivers.
//
// Copyright(c) Seiko Epson Corporation 2000-2008.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------

#ifndef __S1D13521FB_H__
  #define __S1D13521FB_H__

  #include <linux/kernel.h>
  #include <linux/fb.h>
  #include "DataType.h"

  #include "s1d13521ioctl.h"
  #include "s1d13521.h"

  #include "s1d13521_debug.h"

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  #define HRDY_TIMEOUT_MS 5000

  //#define CONFIG_FB_EPSON_PROC

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // In Indirect Mode, a copy of the framebuffer is kept in system memory.
  // A timer periodically writes this copy to the "real" framebuffer in
  // hardware. This copy is called a virtual framebuffer.

  //----------------------------------------------------------------------------
  // Global structures used by s1d13521fb frame buffer code
  //----------------------------------------------------------------------------
  typedef struct
  {
          volatile unsigned char *RegAddr;
          unsigned 								RegAddrMappedSize;
          volatile unsigned char *DataPort;
          unsigned 								DataPortSzie;
          volatile unsigned char *CommandPort;
          unsigned 								CommandPortSzie;
          
          u32 VirtualFramebufferAddr;
          int blank_mode;
          u32 pseudo_palette[16];
  }FB_INFO_S1D13521;

  extern struct fb_fix_screeninfo s1d13521fb_fix;
  extern struct fb_info s1d13521_fb;
  extern FB_INFO_S1D13521 s1d13521fb_info;
  extern char *s1d13521fb_version;

  //-----------------------------------------------------------------------------
  // Global Function Prototypes
  //-----------------------------------------------------------------------------

  #ifdef CONFIG_FB_EPSON_PROC
    int  __devinit s1d13521proc_init(void);
    void __devexit s1d13521proc_terminate(void);
  #endif

  #ifdef CONFIG_FB_EPSON_PCI
    int  __devinit s1d13521pci_init(long *physicalAddress);
    void __devexit s1d13521pci_terminate(void);
  #endif

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  typedef struct  TAG_ST_A_GPIO_INFO{
	const UINT32 	Info;
	const char 		*Name;
  } ST_A_GPIO_INFO, *PST_A_GPIO_INFO;

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  typedef enum {
  	_EPSON_ERROR_SUCCESS=0,
  	_EPSON_ERROR_NOT_READY,
  	_EPSON_ERROR_TIMEOUT,
  	_EPSON_ERROR_WRITE_FAIL,
  } EN_EPSON_ERROR_CODE;

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  typedef struct TAG_ST_DESC_ACCESS_BUF {
  	UINT32	Len;
  	PUINT		pData;
  } ST_DESC_ACCESS_BUF, *PST_DESC_ACCESS_BUF;

  #pragma pack(1)
  typedef struct TAG_ST_COMMAND_PACKAGE {
  	ST_DESC_ACCESS_BUF	Write;
  	ST_DESC_ACCESS_BUF	Read;
  	UINT16							Command;
  } ST_COMMAND_PACKAGE, *PST_COMMAND_PACKAGE;
  #pragma pack()

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  typedef struct TAG_ST_PARTIAL_RECT {
  	UINT16 StartX;
  	UINT16 StartY;
  	UINT16 Width;
  	UINT16 Height;
  } ST_PARTIAL_RECT, *PST_PARTIAL_RECT;

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  int  __devinit 				s1d13521if_InterfaceInit(FB_INFO_S1D13521 *info);
  void __devexit 				s1d13521if_InterfaceTerminate(FB_INFO_S1D13521 *info);

  int  									BusIssueCmd(unsigned ioctlcmd,s1d13521_ioctl_cmd_params *params,int numparams);

  void 									BusIssueWriteBuf(u16 *ptr16, unsigned copysize16);
  void 									BusIssueReadBuf(u16 *ptr16, unsigned copysize16);

  u16  									BusIssueReadReg(u16 Index);
  EN_EPSON_ERROR_CODE		BusIssueWriteReg(u16 Index, u16 Value);
  EN_EPSON_ERROR_CODE		BusIssueWriteRegX(u16 Index, u16 Value);  
  EN_EPSON_ERROR_CODE 	BusIssueCmdX(UINT16 Cmd);

  EN_EPSON_ERROR_CODE		BusIssueWriteRegBuf(u16 Index, PUINT16 pData, UINT32 Length);
  EN_EPSON_ERROR_CODE  	BusWaitForHRDY(void);
  void 									BusIssueDoRefreshDisplay(unsigned cmd,unsigned mode);
  void 									BusIssueInitDisplay(void);
  void 									BusIssueInitRegisters(void);
  
  int 									pvi_GoToSleep(void);
  int 									pvi_GoToNormal(void);


//  int 						s1d13521if_CmdPackage(PST_COMMAND_PACKAGE pCmdPkg);

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  int  pvi_Init(VOID);
  VOID pvi_Deinit(VOID);

  int  pvi_ioctl_NewImage(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_StopNewImage(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_DisplayImage(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_PartialImage(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_DisplayPartial(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_Reset(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_SetDepth(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_EraseDisplay(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_Rotate(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_Positive(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_Negative(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_GoToNormal(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_GoToSleep(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_GoToStandBy(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_WriteToFlash(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_ReadFromFlash(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_Init(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_AutoRefreshOn(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_AutoRefreshOff(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_SetRefresh(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_ForcedRefresh(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_GetRefresh(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_RestoreImage(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_ControllerVersion(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_SoftwareVersion(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_DisplaySize(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_GetStatus(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_Temperature(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_WriteRegister(PTDisplayCommand puDisplayCommand);
  int  pvi_ioctl_ReadRegister(PTDisplayCommand puDisplayCommand);

  int  pvi_SwitchCommand(PTDisplayCommand pDisplayCommand);
  //int Epson_displayCMD(PST_IMAGE_PGM PST);
  //int Epson_displayCMD_MEM(PST_IMAGE_PGM_MEM PST);  //KEG 20090914
  //int Epson_LoadImageArea(PTloadImageArea area);	// KEG 20090814
  
	//BOOL BusIssueFlashOperation(PS1D13532_FLASH_PACKAGE pFlashControl);
	BOOL BusIssueReset(void);
	//int ImagePGM(PST_IMAGE_PGM pPGM);
	
	VOID DeviceReset(VOID);


#endif  //__S1D13521FB_H__
