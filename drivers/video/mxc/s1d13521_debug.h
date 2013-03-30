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

#ifndef __S1D13521_DEBUG_H__
  #define __S1D13521_DEBUG_H__

  #include <linux/kernel.h>

  //-----------------------------------------------------------------------------
  // Global Function Prototypes
  //-----------------------------------------------------------------------------
  
//  #undef CONFIG_FB_EPSON_DEBUG_PRINTK

  #ifdef CONFIG_FB_EPSON_DEBUG_PRINTK
//    #define dbg_info(fmt, args...) do { printk(KERN_INFO fmt, ## args); } while (0)
    #define dbg_info(fmt, args...) 				   do { printk(KERN_INFO fmt, ## args); } while (0)
    #define DEBUGMSG(con, fmt, args...) 		   do { if(con != 0) printk(fmt, ## args); } while (0)
    #define DEBUGMSG_FUNC_ENTER(con) 			   do { if(con != 0) printk("%s]+++\n", __FUNCTION__); } while (0)
    #define DEBUGMSG_FUNC_EXIT(con) 			   do { if(con != 0) printk("%s]---\n", __FUNCTION__); } while (0)
    #define DEBUGMSG_FUNC_EXIT_CODE(con, x) 	   do { if(con != 0) printk("%s]--- ret=%d\n", __FUNCTION__, x); } while (0)
  #else
    #define dbg_info(fmt, args...) 					do { } while (0)
    #define DEBUGMSG(con, fmt, args...) 			do { } while (0)
    #define DEBUGMSG_FUNC_ENTER(con) 				do { } while (0)
    #define DEBUGMSG_FUNC_EXIT(con) 				do { } while (0)
    #define DEBUGMSG_FUNC_EXIT_CODE(con, code) 		do { } while (0)
  #endif

  #ifdef CONFIG_FB_EPSON_DEBUG_PRINTK
    #define assert(expr) \
          if(!(expr)) { \
          printk( "Assertion failed! %s,%s,%s,line=%d\n",\
          #expr,__FILE__,__FUNCTION__,__LINE__); \
          BUG(); \
          }
  #else
    #define assert(expr)
  #endif


#endif  //__S1D13521_DEBUG_H__
