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

#ifndef __NTX_S1D13521FB_H__
#define __NTX_S1D13521FB_H__

  #include <linux/kernel.h>
  #include <linux/fb.h>

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  #define _LOOP_TIMEOUT_READY			500000

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//#define __EB600__
//#define  __EB500__
  //#define  __EB600EM__
  //#define  __EB600E__
  #define  __COOKIE__

        //#if (_DEVICE_CODE == _DEVICE_CODE_EB600)
        #if defined( __EB600__)
                #define _DEIVE_NAME                             "EB600"
        //#elif (_DEVICE_CODE == _DEVICE_CODE_EB500)
        #elif defined( __EB500__)
                #define _DEIVE_NAME                             "EB500"
        //#elif (_DEVICE_CODE == _DEVICE_CODE_EB600E)
        #elif defined( __EB600E__)
                #define _DEIVE_NAME                             "EB600E"
        //#elif (_DEVICE_CODE == _DEVICE_CODE_COOKIE)
        #elif defined( __COOKIE__)
                #define _DEIVE_NAME                             "COOKIE"
        //#elif (_DEVICE_CODE == _DEVICE_CODE_EB600EM)
        #elif defined( __EB600EM__)
                #define _DEIVE_NAME                             "EB600EM"
        #else
          #error "What device!!!!!!!"
        #endif

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//  #define __TURN_ON_FORCE_DISPLAY__
//  #define __TURN_ON_EPSON_INIT__

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//  #define __5_INCH__
  #define __6_INCH__
//  #define __8_INCH__
//  #define __9D7_INCH__

//  #define _WAVE_FORM_BASE							0x886
  #define _WAVE_FORM_BASE							0x1000

  #define _S1D13522_WAVE_FORM_BASE					0xb0ea
  #define _S1D13522_WAVE_FORM_BASE_H				0x0003

#if defined(__5_INCH__)
  #define _PANEL_WIDTH								600
  #define _PANEL_HEIGHT								800

  #define S1D_DISPLAY_WIDTH           600
  #define S1D_DISPLAY_HEIGHT          800

  #define _INIT_HSIZE         				800
  #define _INIT_VSIZE         				600
  #define _INIT_FSLEN         				4
  #define _INIT_FBLEN         				4
  #define _INIT_FELEN         				10
  #define _INIT_LSLEN         				10
  #define _INIT_LBLEN         				4
  #define _INIT_LELEN         				100
  #define _INIT_PIXCLKDIV     				6
  #define _INIT_SDRV_CFG      				(100 | (1<< 8) | (1<<9))
  #define _INIT_GDRV_CFG      				0x2
  #define _INIT_LUTIDXFMT     				(4 | (1<<7))

  #define _INIT_ROTMODE       				3    // rotation mode = 270 degrees

#elif defined(__6_INCH__)
  #define _PANEL_WIDTH				_INIT_VSIZE
  #define _PANEL_HEIGHT				_INIT_HSIZE

  #define S1D_DISPLAY_WIDTH           _INIT_VSIZE
  #define S1D_DISPLAY_HEIGHT          _INIT_HSIZE

  //#define _INIT_HSIZE         				800  // max panel original width .
  //#define _INIT_VSIZE         				600	 // max panel original height .
  #define _INIT_HSIZE         				1024  // max panel original width .
  #define _INIT_VSIZE         				768	 // max panel original height .
  
  #define _INIT_FSLEN         				4
  #define _INIT_FBLEN         				4
  #define _INIT_FELEN         				10
  #define _INIT_LSLEN         				10
  #define _INIT_LBLEN         				4
  #define _INIT_LELEN         				100
  #define _INIT_PIXCLKDIV     				6
  #define _INIT_SDRV_CFG      				(100 | (1<< 8) | (1<<9))
  #define _INIT_GDRV_CFG      				0x2
  #define _INIT_LUTIDXFMT     				(4 | (1<<7))

  #define _INIT_ROTMODE       				3    // rotation mode = 270 degrees
  //#define _INIT_ROTMODE       				1    // rotation mode = 270 degrees

#elif defined(__8_INCH__)
#elif defined(__9D7_INCH__)
  #define _PANEL_WIDTH								1200
  #define _PANEL_HEIGHT								826

  #define S1D_DISPLAY_WIDTH           1200
  #define S1D_DISPLAY_HEIGHT          826

  #define _INIT_HSIZE         				1200
  #define _INIT_VSIZE         				826
  #define _INIT_FSLEN         				0
  #define _INIT_FBLEN         				4
  #define _INIT_FELEN         				4
  #define _INIT_LSLEN         				4
  #define _INIT_LBLEN         				10
  #define _INIT_LELEN         				60
  #define _INIT_PIXCLKDIV     				3
  #define _INIT_SDRV_CFG      				(100 | (1<< 8) | (1<<9))
  #define _INIT_GDRV_CFG      				0x2
  #define _INIT_LUTIDXFMT     				(4 | (1<<7))

  #define _INIT_ROTMODE       				0    // rotation mode = 270 degrees
//#define _INIT_ROTMODE       				1    // rotation mode = 270 degrees
#else
  #error "give me panel size"

#endif
  #define _PANEL_PEXIL_BPP						8
  #define _PANEL_TOTAL_PEXIL					(_PANEL_WIDTH*_PANEL_HEIGHT)
  #define _PANEL_BUFFER_SIZE					(_PANEL_TOTAL_PEXIL*_PANEL_PEXIL_BPP)/8
  #define S1D_DISPLAY_BPP					4

#define SHOW_PROGRESS_BAR
// Joseph 20101116
#define _PROGRESS_BAR_X_		360
#define _PROGRESS_BAR_Y_		740
#define _PROGRESS_BAR_ICONS_	7

#define                                                 __LINUX_VERSION__ "2.6.18.2-ntx-cookie-v2.9"
#endif  //__NTX_S1D13521FB_H__
