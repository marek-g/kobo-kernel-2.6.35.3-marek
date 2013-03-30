//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson/s1d1352ioctl.h -- IOCTL definitions for Epson
// S1D13521 controller frame buffer driver.
//
// Copyright(c) Seiko Epson Corporation 2008.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------
#ifndef __EPSON_IOCTL_H__
  #define __EPSON_IOCTL_H__
  
  #include "ntx_s1d13521fb.h"
  
/* ioctls
    0x45 is 'E'                                                          */
struct s1d13521_ioctl_hwc
{
        unsigned addr;
        unsigned value;
        void* buffer;
};

#define S1D13521_REGREAD                0x4540
#define S1D13521_REGWRITE               0x4541
#define S1D13521_MEMBURSTREAD           0x4546
#define S1D13521_MEMBURSTWRITE          0x4547
#define S1D13521_VBUF_REFRESH		0x4548	// KEG 20090807 unused, 
						// but included to stay compatible with latest epson driver
						


#define S1D13521_DISPLAY		0x4539
#define S1D13521_SETDEPTH		0x4550
#define S1D13521_DISPLAY_KAY		0x4542 //kay 20090511
#define S1D13521_FORCE_REFRESH	0x4543  

#define S1D13521_FORCE_REFRESH_FULL_AREA	0x4544
#define S1D13521_FORCE_REFRESH_FULL_FULL	       0x4545
#define S1D13521_FORCE_REFRESH_PART_AREA	0x4556
#define S1D13521_FORCE_REFRESH_PART_FULL 	0x4557

#define S1D13521_FORCE_VFB_TO_SCREEN	0x4560	// KEG 20090805
#define S1D13521_SET_AUTOREFRESH_TIME	0x4561	// KEG 20090807
#define S1D13521_GET_AUTOREFRESH_TIME	0x4562	// KEG 20090807
#define S1D13521_ERASE_SCREEN		0x4563	// KEG 20090807
#define S1D13521_LOAD_AREA		0x4564	// KEG 20090814

// virtual hardware controll .
#define EPDC_PWR_CTRL		0x4570	// epdc power controll .
	#define EPDC_AUTOPWR_INTERVAL_MAX		1
	#define EPDC_AUTOPWR_INTERVAL_NORMAL	2
	#define EPDC_PWR_ON						3
	#define EPDC_PWR_OFF					4
	#define EPDC_VCOM_ON					5
	#define EPDC_VCOM_OFF					6
	#define EPDC_AUTOOFF_ENABLE				7
	#define EPDC_AUTOOFF_DISABLE			8
	
#define EPDC_VCOM_SET				0x4571
#define EPDC_VCOM_SET_TO_FLASH		0x4572
#define EPDC_VCOM_GET				0x4573
	


//#define S1D13521_FLASH          				0x0100
#define S1D13521_FLASH          				0x4700
#define S1D13521_RESET	         				0x4701
#define S1D13521_PGM		         				0x4702

// System commands
#define INIT_CMD_SET                    0x00
#define INIT_PLL_STANDBY                0x01
#define RUN_SYS                         0x02
#define STBY                            0x04
#define SLP                             0x05
#define INIT_SYS_RUN                    0x06
#define INIT_SYS_STBY                   0x07
#define INIT_SDRAM                      0x08
#define INIT_DSPE_CFG                   0x09
#define INIT_DSPE_TMG                   0x0A
#define INIT_ROTMODE                    0x0B
//#define SET_REFRESH_CNT                 0x0C
#define INIT_WAVEDEV										0x0C


// Register and memory access commands
#define RD_REG                          0x10
#define WR_REG                          0x11
#define RD_SFM                          0x12
#define WR_SFM                          0x13
#define END_SFM                         0x14

// Burst access commands
#define BST_RD_SDR                      0x1C
#define BST_WR_SDR                      0x1D
#define BST_END_SDR                     0x1E

// Image loading commands
#define LD_IMG                          0x20
#define LD_IMG_AREA                     0x22
#define LD_IMG_END                      0x23
#define LD_IMG_WAIT                     0x24
#define LD_IMG_SETADR                   0x25
#define LD_IMG_DSPEADR                  0x26

// Polling commands
#define WAIT_DSPE_TRG                   0x28
#define WAIT_DSPE_FREND                 0x29
#define WAIT_DSPE_LUTFREE               0x2A
#define WAIT_DSPE_MLUTFREE              0x2B

// Waveform update commands
#define RD_WFM_INFO                     0x30
#define UPD_INIT                        0x32
#define UPD_FULL                        0x33
#define UPD_FULL_AREA                   0x34
#define UPD_PART                        0x35
#define UPD_PART_AREA                   0x36
#define UPD_GDRV_CLR                    0x37
#define UPD_SET_IMGADR                  0x38

#pragma pack(1)

typedef struct
{
        u16 param[5];
}s1d13521_ioctl_cmd_params;

#pragma pack()

#define S1D13521_INIT_CMD_SET           (0x4500 | INIT_CMD_SET)
#define S1D13521_INIT_PLL_STANDBY       (0x4500 | INIT_PLL_STANDBY)
#define S1D13521_RUN_SYS                (0x4500 | RUN_SYS)
#define S1D13521_STBY                   (0x4500 | STBY)
#define S1D13521_SLP                    (0x4500 | SLP)
#define S1D13521_INIT_SYS_RUN           (0x4500 | INIT_SYS_RUN)
#define S1D13521_INIT_SYS_STBY          (0x4500 | INIT_SYS_STBY)
#define S1D13521_INIT_SDRAM             (0x4500 | INIT_SDRAM)
#define S1D13521_INIT_DSPE_CFG          (0x4500 | INIT_DSPE_CFG)
#define S1D13521_INIT_DSPE_TMG          (0x4500 | INIT_DSPE_TMG)
#define S1D13521_INIT_ROTMODE           (0x4500 | INIT_ROTMODE)
#define S1D13522_INIT_WAVEDEV						(0x4500 | INIT_WAVEDEV)
#define S1D13521_RD_REG                 (0x4500 | RD_REG)
#define S1D13521_WR_REG                 (0x4500 | WR_REG)
#define S1D13521_RD_SFM                 (0x4500 | RD_SFM)
#define S1D13521_WR_SFM                 (0x4500 | WR_SFM)
#define S1D13521_END_SFM                (0x4500 | END_SFM)

// Burst access commands
#define S1D13521_BST_RD_SDR             (0x4500 | BST_RD_SDR)
#define S1D13521_BST_WR_SDR             (0x4500 | BST_WR_SDR)
#define S1D13521_BST_END_SDR            (0x4500 | BST_END_SDR)

// Image loading IOCTL commands
#define S1D13521_LD_IMG                 (0x4500 | LD_IMG)
#define S1D13521_LD_IMG_AREA            (0x4500 | LD_IMG_AREA)
#define S1D13521_LD_IMG_END             (0x4500 | LD_IMG_END)
#define S1D13521_LD_IMG_WAIT            (0x4500 | LD_IMG_WAIT)
#define S1D13521_LD_IMG_SETADR          (0x4500 | LD_IMG_SETADR)
#define S1D13521_LD_IMG_DSPEADR         (0x4500 | LD_IMG_DSPEADR)

// Polling commands
#define S1D13521_WAIT_DSPE_TRG          (0x4500 | WAIT_DSPE_TRG)
#define S1D13521_WAIT_DSPE_FREND        (0x4500 | WAIT_DSPE_FREND)
#define S1D13521_WAIT_DSPE_LUTFREE      (0x4500 | WAIT_DSPE_LUTFREE)
#define S1D13521_WAIT_DSPE_MLUTFREE     (0x4500 | WAIT_DSPE_MLUTFREE)

// Waveform update IOCTL commands
#define S1D13521_RD_WFM_INFO            (0x4500 | RD_WFM_INFO)
#define S1D13521_UPD_INIT               (0x4500 | UPD_INIT)
#define S1D13521_UPD_FULL               (0x4500 | UPD_FULL)
#define S1D13521_UPD_FULL_AREA          (0x4500 | UPD_FULL_AREA)
#define S1D13521_UPD_PART               (0x4500 | UPD_PART)
#define S1D13521_UPD_PART_AREA          (0x4500 | UPD_PART_AREA)
#define S1D13521_UPD_GDRV_CLR           (0x4500 | UPD_GDRV_CLR)
#define S1D13521_UPD_SET_IMGADR         (0x4500 | UPD_SET_IMGADR)

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  typedef unsigned int		HWND;
//  #define MaxBufferSize     (((800 * 600) / 2) + 50)
  #define MaxBufferSize     (((_PANEL_WIDTH * _PANEL_HEIGHT) / 2) + 50)

  typedef enum {
  	_STEP_NOR_BYPASS=0,
  	_STEP_NOR_WRITE_AA_INTO_555,
  	_STEP_NOR_WRITE_55_INTO_2AA,

  	_STEP_NOR_CMD_PROGRAM,
  	_STEP_NOR_CMD_PROGRAM_AA_WRITE_INTO_555,
  	_STEP_NOR_CMD_PROGRAM_55_WRITE_INTO_2AA,
  	_STEP_NOR_CMD_PROGRAM_DATA,

  	_STEP_NOR_CMD_ERASE,
  	_STEP_NOR_CMD_ERASE_WRITE_AA_INTO_555,
  	_STEP_NOR_CMD_ERASE_WRITE_55_INTO_2AA,

  	_STEP_NOR_CMD_STATUS,


  } STEP_NOR_2_SPI_FLASH;

//  #pragma pack(1)
  typedef struct TAG_ST_PVI_CONFIG {
  	UINT16 					CurRotateMode;
  	UINT16 					PastRotateMode;

  	STEP_NOR_2_SPI_FLASH	StepSPI;

  	UINT16 					ReverseGrade;

  	BOOL   					fAutoRefreshMode;
  	BYTE   					AutoRefreshTimer;

  	UINT16 					fNormalMode;

  	BYTE   					Deepth;

  	UINT16 					StartX;
  	UINT16 					StartY;
  	UINT16 					Width;
  	UINT16 					Height;

  	UINT16 					Reg0x140;
  	UINT16 					Reg0x330;

  } ST_PVI_CONFIG, *PST_PVI_CONFIG;

  #pragma pack()

  typedef struct ST_IMAGE_PGM_TAG{
   unsigned int StartX;
   unsigned int StartY;
   unsigned int Width;
   unsigned int Height;
   unsigned int WaveForm;
   unsigned int LUT_NO;
   BYTE Data[_PANEL_WIDTH*_PANEL_HEIGHT];
  } ST_IMAGE_PGM, *PST_IMAGE_PGM;
  
  typedef struct ST_IMAGE_PGM_800x600_TAG{
   unsigned int StartX;
   unsigned int StartY;
   unsigned int Width;
   unsigned int Height;
   unsigned int WaveForm;
   unsigned int LUT_NO;
   BYTE Data[800*600];
  } ST_IMAGE_PGM_800x600, *PST_IMAGE_PGM_800x600;


/****kay*******/
//kay 20090428 for epson memory control
typedef enum {
  	_EPSON_MEM_CLEAR=0,
  	_EPSON_MEM_CLEAR_BY_INDEX=1,
  	_EPSON_MEM_CLEAR_BY_NAME=2,
  	_EPSON_MEM_SAVE_BY_INDEX=3,
  	_EPSON_MEM_SAVE_BY_NAME=4,
	_EPSON_MEM_RESTORE_BY_INDEX=5,  	
  	_EPSON_MEM_RESTORE_BY_NAME=6,
  	_EPSON_MEM_SAVE_BY_NAME_NOT_SHOW=7,
  	_EPSON_MEM_SHOW_PIC=8,

} E_EPSON_MEM_CMD;

#define MAX_EPSON_MEMORY_NAME_LENGTH 64
#define COUNT_FOR_32M 60
#define COUNT_FOR_64M 120
//#define EPSON_MEMORY_CTL_BASE_START 800*600*2
//#define EPSON_MEMORY_CTL_UNIT_SIZE 800*600
#define MAX_EPSON_MEMORY_CTL_SIZE  COUNT_FOR_32M
  struct EPSON_MEMORY_CTL {
  	U8 cmd;
  	UINT16  x;
  	UINT16  y;
  	UINT16  width;
  	UINT16  height;
	UINT16  rotate;
	U8 index;
  	char name[MAX_EPSON_MEMORY_NAME_LENGTH];
	U8 waveform;
  };


//

  typedef struct ST_IMAGE_PGM_TAG_MEM{
   U8 cmd;
   U8 index;
   char name[MAX_EPSON_MEMORY_NAME_LENGTH];
   unsigned int StartX;
   unsigned int StartY;
   unsigned int Width;
   unsigned int Height;
   unsigned int WaveForm;
   unsigned int LUT_NO;
   unsigned int Depth;   
   BYTE Data[_PANEL_WIDTH*_PANEL_HEIGHT];
  } ST_IMAGE_PGM_MEM, *PST_IMAGE_PGM_MEM;
  typedef struct ST_IMAGE_PGM_TAG_MEM_KERNEL{
   U8 cmd;
   U8 index;
   char name[MAX_EPSON_MEMORY_NAME_LENGTH];
   unsigned int StartX;
   unsigned int StartY;
   unsigned int Width;
   unsigned int Height;
   unsigned int WaveForm;
   unsigned int LUT_NO;
   unsigned int Depth;   
//   BYTE Data[800*600]; we don't save image data in kernel :P
  } ST_IMAGE_PGM_MEM_KERNEL, *PST_IMAGE_PGM_MEM_KERNEL;

/*******kay*******/



  typedef struct TAG_TDisplayCommand{
     int Owner;
     BYTE Command;
     int BytesToWrite ;
     int BytesToRead;
     BYTE Data[MaxBufferSize];
  } TDisplayCommand, *PTDisplayCommand;

  typedef struct TAG_TDisplayCommandSmall{
     int Owner;
     BYTE Command;
     int BytesToWrite ;
     int BytesToRead;
     BYTE Data[16];
  } TDisplayCommandSmall, *PTDisplayCommandSmall;

  typedef struct TAG_TDisplayCommandHeader{
     int Owner;
     BYTE Command;
     int BytesToWrite ;
     int BytesToRead;
  } TDisplayCommandHeader, *PTDisplayCommandHeader;

  #define _OFFSET_DISP_COMMAND_FDATA	(((PTDisplayCommand) 0)->Data)

  #define  dc_NewImage          0xA0
  #define  dc_StopNewImage      0xA1
  #define  dc_DisplayImage      0xA2
  #define  dc_PartialImage      0xB0
  #define  dc_DisplayPartial    0xB1
  #define  dc_DisplayPartialGU  0xB2
  #define  dc_Reset             0xEE
  #define  dc_SetDepth          0xF3
  #define  dc_EraseDisplay      0xA3
  #define  dc_Rotate            0xF5
  #define  dc_Positive          0xF7
  #define  dc_Negative          0xF8
  #define  dc_GoToNormal        0xF0
  #define  dc_GoToSleep         0xF1
  #define  dc_GoToStandBy       0xF2
  #define  dc_WriteToFlash      0x01
  #define  dc_ReadFromFlash     0x02
  #define  dc_Init              0xA4
  #define  dc_AutoRefreshOn     0xF9
  #define  dc_AutoRefreshOff    0xFA
  #define  dc_SetRefresh        0xFB
  #define  dc_ForcedRefresh     0xFC
  #define  dc_GetRefresh        0xFD
  #define  dc_RestoreImage      0xA5
  #define  dc_ControllerVersion 0xE0
  #define  dc_SoftwareVersion   0xE1
  #define  dc_DisplaySize       0xE2
  #define  dc_GetStatus         0xAA
  #define  dc_Temperature       0x21
  #define  dc_WriteRegister     0x10
  #define  dc_ReadRegister      0x11
  #define  dc_Abort             0xA1
  #define CMD_SendCommand 1
  #define CMD_EPSON_MEMORY_CTL 3
  
  typedef struct _S1D13532_FLASH_PACKAGE_TAG {
  	DWORD 		Command;
  	DWORD 		StartAddr;
  	DWORD 		DataLength;
  	BYTE	 		Buf[4];
  } S1D13532_FLASH_PACKAGE, *PS1D13532_FLASH_PACKAGE;
  
  #define _FLASH_CMD_GET_INFO			0
  #define _FLASH_CMD_ERASE				1
  #define _FLASH_CMD_WRITE				2
  #define _FLASH_CMD_READ					3

  typedef struct TAG_LOADIMAGEAREA {
	BYTE cmd;
	BYTE mode;
	int XStart;
	int YStart;
	int Width;
	int Height;
  } TLoadImageArea, *PTloadImageArea;

#endif // __EPSON_IOCTL_H__

