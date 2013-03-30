#ifndef _GALLEN_DBG__H//[
#define _GALLEN_DBG__H


/*****************************************************************
1. include 此檔前請先定義 GDEBUG .
	如 #define GDEBUG			1000
	
2. 請先include相關檔 .
	如 kernel.h , assert.h ....
	
3. 請定義一個GLOBAL變數,改變此變數的值即可變化除錯訊息的輸出多或少.
	int giDbgLvl;
	
******************************************************************/

//#ifdef __KERNEL__ //[ 
	//#define GALLEN_PRINT		printk
//#else //][!__KERNEL__
	//#define GALLEN_PRINT		printf
//#endif //] __KERNEL__

#ifdef __KERNEL__ //[ 
	#define GALLEN_PRINT(fmt,args...)		printk(fmt,##args)
	#define GALLEN_ERR_PRINT(fmt,args...)		printk(KERN_ERR fmt,##args)
	#define GALLEN_WARNING_PRINT(fmt,args...)		printk(KERN_WARNING fmt,##args)
	#define GET_CURRENT_TICK()									(unsigned int)(jiffies)
#else //][!__KERNEL__

	#include <sys/times.h>
	#include <stdio.h>
	
	#define GALLEN_PRINT(fmt,args...)		printf(fmt,##args)
	#define GALLEN_ERR_PRINT(fmt,args...)		fprintf(stderr,fmt,##args)
	#define GALLEN_WARNING_PRINT(fmt,args...)		fprintf(stderr,fmt,##args)
	#define GET_CURRENT_TICK()									(unsigned int)(times(0))
#endif //] __KERNEL__

#if 0
	extern int giDbgLvl; // 程式必須提供這個變數.
#else
	#define giDbgLvl 	0
#endif


#define GALLEN_DBGLVL_MAX			100 // 代表無訊息輸出.
#define GALLEN_DBGLVL_HALF			50 // .
#define GALLEN_DBGLVL_MIN			0 // 代表所有訊息都會輸出.


//
//common MACRO
//
#define BUF_SETBIT(pbBuf,iSetbit)	\
{\
	int _iBit=iSetbit&0x7,_iByte=iSetbit>>3;\
	pbBuf[_iByte] |=0x80>>_iBit;\
}

#define BUF_CLRBIT(pbBuf,iClrbit)	\
{\
	int _iBit=iClrbit&0x7,_iByte=iClrbit>>3;\
	pbBuf[_iByte] &= ~((unsigned char)(0x80>>_iBit));\
}

#define BUF_PRINT_ONBITS(pbBuf,iTotalBits)	\
{\
	int _iBit=0,_iByte=0,_iTestBits=0;\
	unsigned char bTemp;\
	GALLEN_PRINT("%s(%d): %s() %s bits on of %d bits =\n[",__FILE__,__LINE__,__FUNCTION__,#pbBuf,iTotalBits);\
	for(_iByte=0;_iTestBits<iTotalBits;_iByte++) {\
		for(_iBit=0;_iBit<8 && _iTestBits<iTotalBits;_iBit++) {\
			bTemp=0x80>>_iBit;\
			if(bTemp&pbBuf[_iByte]) {\
				GALLEN_PRINT("%d,",_iTestBits);\
			}\
			_iTestBits++;\
		}\
	}\
	GALLEN_PRINT("]\n\n");\
}

#if (GDEBUG >= 10) 
	
	#define GALLEN_DECLARE_INT(name)	volatile int i##name=0
	#define GALLEN_DECLARE_EXTINT(name)	extern volatile int i##name
	#define GALLEN_SET_INT(name,a)	i##name=(a)
	#define GALLEN_GET_INT(name)		i##name
	


	#define GALLEN_DBGLOCAL_MUTEBEGIN()	GALLEN_DBGLOCAL_MUTEBEGIN_EX(32)
	#define GALLEN_DBGLOCAL_MUTEBEGIN_EX(_bits)	\
		{	\
			const int _iRunlogBits=_bits;\
			const int _iRunlogBytes=((_bits/8)+1);\
			volatile unsigned char bRunLogsA[((_bits/8)+1)] = {0,},bLastLocalLogsA[((_bits/8)+1)] = {0,}; \
			int __iLocalDbgLvl = GALLEN_DBGLVL_MAX;\
			volatile unsigned int uiWaitTickWarning=10,uiLastTick = GET_CURRENT_TICK();\



	#define GALLEN_DBGLOCAL_BEGIN()	\
		GALLEN_DBGLOCAL_MUTEBEGIN();\
		if(giDbgLvl<=__iLocalDbgLvl){\
			GALLEN_PRINT("%s(%d):%s() Enter (t=%d)...\n",__FILE__,__LINE__,__FUNCTION__,GET_CURRENT_TICK());\
		}
	#define GALLEN_DBGLOCAL_BEGIN_EX(_bits)	\
		GALLEN_DBGLOCAL_MUTEBEGIN_EX(_bits);\
		if(giDbgLvl<=__iLocalDbgLvl){\
			GALLEN_PRINT("%s(%d):%s() Enter (t=%d)...\n",__FILE__,__LINE__,__FUNCTION__,GET_CURRENT_TICK());\
		}
	
	#define GALLEN_DBGLOCAL_END() GALLEN_DBGLOCAL_PRINTLOG_EX(Leave,1,0)		}
	#define GALLEN_DBGLOCAL_MUTEEND() \
		uiLastTick=uiLastTick;uiWaitTickWarning=uiWaitTickWarning;bLastLocalLogsA[0]=bLastLocalLogsA[0];}

		
	#define GALLEN_DBGLOCAL_ESC() GALLEN_DBGLOCAL_PRINTLOG_EX(escape,1,0)

		
	#define GALLEN_DBGLOCAL_PRINTLOG() GALLEN_DBGLOCAL_PRINTLOG_EX(show,0,0)

	#define GALLEN_DBGLOCAL_PRINTLOG_EX(msg,_is_show_tick,_iRepeatCnt) \
		if(giDbgLvl<=__iLocalDbgLvl) \
		{ \
			int _iBit=0,_iByte=0,_iTestBits=0;\
			GALLEN_PRINT("%s(%d):%s() %s RunLog@%d=\n[",__FILE__,__LINE__,__FUNCTION__,#msg,_iRepeatCnt);\
			for(_iByte=0;_iByte<_iRunlogBytes && _iTestBits<_iRunlogBits;_iByte++) {\
				for(_iBit=0;_iBit<8 && _iTestBits<_iRunlogBits;_iBit++) {\
					if(bRunLogsA[_iByte]&(0x80>>_iBit)) \
						GALLEN_PRINT("%d,",_iTestBits);\
					_iTestBits++;\
				}\
			}\
			if(_is_show_tick) {\
				GALLEN_PRINT("],(t=%d) \n\n",GET_CURRENT_TICK());\
			}\
			else {\
				GALLEN_PRINT("]\n\n");\
			}\
		}
		
	#define GALLEN_DBGLOCAL_PRINTDIFFLOG() \
		if(giDbgLvl<=__iLocalDbgLvl) \
		{ \
			int _iByte,iCompPass=1;\
			for(_iByte=0;_iByte<_iRunlogBytes;_iByte++) {\
				if(bRunLogsA[_iByte]!=bLastLocalLogsA[_iByte]) {\
					iCompPass=0;\
				}\
				bLastLocalLogsA[_iByte]=bRunLogsA[_iByte];\
			}\
			if(!iCompPass) {\
				GALLEN_DBGLOCAL_PRINTLOG_EX(ShowDiff,1,0);\
			}\
		}
		
	#define GALLEN_DBGLOCAL_PRINTMSG(fmt,args...) \
		if(giDbgLvl<=__iLocalDbgLvl) \
		{ \
			GALLEN_PRINT(fmt,##args);\
		}
		

	#define GALLEN_DBGLOCAL_RUNLOG(b)	\
		{	\
			if((b)<_iRunlogBits) {\
				BUF_SETBIT(bRunLogsA,b);\
			}\
			else {\
				GALLEN_PRINT("%s(%d):[ERROR]DBG RunLog %d cannot >= %d\n",__FILE__,__LINE__,b,_iRunlogBits);\
			}\
		}
		
	#define GALLEN_DBGLOCAL_SAVELOGS()	{ulLastLocalLogs = ulRunLogs;}
		
	#define GALLEN_DBGLOCAL_DBGLVL_INC()	++__iLocalDbgLvl
	#define GALLEN_DBGLOCAL_DBGLVL_DEC()	--__iLocalDbgLvl
	#define GALLEN_DBGLOCAL_DBGLVL_ADD(addval)	__iLocalDbgLvl+=(int)(addval)
	#define GALLEN_DBGLOCAL_DBGLVL_SET(lvl)	__iLocalDbgLvl = lvl


	#define GALLEN_GDBGLVL_INC()	{++giDbgLvl;}
	#define GALLEN_GDBGLVL_DEC()	{--giDbgLvl;}
	#define GALLEN_GDBGLVL_SET(lvl)	{giDbgLvl=(lvl);}
	#define GALLEN_GDBGLVL_GET()	(giDbgLvl)
	#define GALLEN_GDBGLVL_ADD(addval)	{giDbgLvl+=(int)(addval);}
	
	#define GALLEN_DBGLOCAL_DIFF_BEGIN(name)	GALLEN_DBGLOCAL_DIFF_BEGIN_EX(name,32)
	#define GALLEN_DBGLOCAL_DIFF_BEGIN_EX(name,_bits)	\
		GALLEN_DBGLOCAL_MUTEBEGIN_EX(_bits);\
		static volatile unsigned char gbLastRunLogsA##name[(_bits/8)+1];\
		static volatile int giNoChgCnt##name;\
			
	#define GALLEN_DBGLOCAL_DIFF_END(name) GALLEN_DBGLOCAL_DIFF_PRINT(name,Leave_LogChanged);}
	#define GALLEN_DBGLOCAL_DIFF_PRINT(name,msg) \
			if(giDbgLvl<=__iLocalDbgLvl) \
			{\
				int _iByte,iCompPass=1;\
				++giNoChgCnt##name;\
				for(_iByte=0;_iByte<_iRunlogBytes;_iByte++) {\
					if(bRunLogsA[_iByte]!=gbLastRunLogsA##name[_iByte]) {\
						iCompPass=0;\
					}\
					gbLastRunLogsA##name[_iByte]=bRunLogsA[_iByte];\
				}\
				if(!iCompPass){\
					GALLEN_DBGLOCAL_PRINTLOG_EX(msg,1,giNoChgCnt##name);\
				}else {\
					giNoChgCnt##name=0;\
				}\
			}
		
		
	#define GALLEN_DBGLOCAL_DIFF_ESC(name) GALLEN_DBGLOCAL_DIFF_PRINT(name,ESC_LogChanged)
			
	#define GALLEN_DBGLOCAL_DIFF_RUNLOG(name,b)	GALLEN_DBGLOCAL_RUNLOG(b)

		
	#define GALLEN_DBGLOCAL_L0_BEGIN()	
	#define GALLEN_DBGLOCAL_L0_END() 
	#define GALLEN_DBGLOCAL_L0_RUNLOG(b)	
	
	#define dbgENTER()		GALLEN_PRINT("%s(%d) : Enter %s\n",__FILE__,__LINE__,__FUNCTION__) 
	#define dbgLEAVE()		GALLEN_PRINT("%s(%d) : Leave %s\n",__FILE__,__LINE__,__FUNCTION__)
	#define DBG_MSG(fmt,args...)					GALLEN_PRINT(fmt,##args)
	#define DBG_MSG_FLUSH(fmt,args...)		GALLEN_PRINT(fmt,##args)
	#define ERR_MSG(fmt,args...)					GALLEN_ERR_PRINT(fmt,##args)
	#define WARNNING_MSG(fmt,args...)			GALLEN_WARNING_PRINT(fmt,##args)
	#define WARNING_MSG(fmt,args...)			GALLEN_WARNING_PRINT(fmt,##args)
#elif (GDEBUG >= 1)
	#define GALLEN_DECLARE_INT(name)	
	#define GALLEN_DECLARE_EXTINT(name)	
	#define GALLEN_SET_INT(name,a)	
	#define GALLEN_GET_INT(name)		
	
	#define GALLEN_DBGLOCAL_BEGIN()	
	#define GALLEN_DBGLOCAL_BEGIN_EX(bits)	
	#define GALLEN_DBGLOCAL_MUTEBEGIN()	
	#define GALLEN_DBGLOCAL_MUTEBEGIN_EX(bits)	
	#define GALLEN_DBGLOCAL_END() 
	#define GALLEN_DBGLOCAL_MUTEEND()
	#define GALLEN_DBGLOCAL_ESC() 
	#define GALLEN_DBGLOCAL_RUNLOG(b)	
	#define GALLEN_DBGLOCAL_SAVELOGS()
	#define GALLEN_DBGLOCAL_PRINTLOG() 
	#define GALLEN_DBGLOCAL_PRINTMSG(fmt,args...)
	#define GALLEN_DBGLOCAL_PRINTDIFFLOG()	
	#define GALLEN_DBGLOCAL_DBGLVL_INC()	
	#define GALLEN_DBGLOCAL_DBGLVL_DEC()	
	#define GALLEN_DBGLOCAL_DBGLVL_ADD(addval)
	#define GALLEN_DBGLOCAL_DBGLVL_SET(lvl)	
	#define GALLEN_GDBGLVL_INC()	
	#define GALLEN_GDBGLVL_DEC()	
	#define GALLEN_GDBGLVL_SET(lvl)	
	#define GALLEN_GDBGLVL_GET()	0
	#define GALLEN_GDBGLVL_ADD(addval)	
	#define GALLEN_DBGLOCAL_L0_BEGIN()	
	#define GALLEN_DBGLOCAL_L0_END() 
	#define GALLEN_DBGLOCAL_L0_RUNLOG(b)	
	#define GALLEN_DBGLOCAL_DIFF_BEGIN(name)	
	#define GALLEN_DBGLOCAL_DIFF_BEGIN_EX(name,bits)	
	#define GALLEN_DBGLOCAL_DIFF_ESC(name)	
	#define GALLEN_DBGLOCAL_DIFF_END(name) 
	#define GALLEN_DBGLOCAL_DIFF_RUNLOG(name,b)	
	#define dbgENTER()			GALLEN_PRINT("%s(%d) : Enter %s\n",__FILE__,__LINE__,__FUNCTION__)
	#define dbgLEAVE()			GALLEN_PRINT("%s(%d) : Leave %s\n",__FILE__,__LINE__,__FUNCTION__)
	#define DBG_MSG(fmt,args...)					GALLEN_PRINT(fmt,##args)
	#define DBG_MSG_FLUSH(fmt,args...)		GALLEN_PRINT(fmt,##args)
	#define ERR_MSG(fmt,args...)					GALLEN_ERR_PRINT(fmt,##args)
	#define WARNNING_MSG(fmt,args...)			GALLEN_WARNING_PRINT(fmt,##args)
	#define WARNING_MSG(fmt,args...)			GALLEN_WARNING_PRINT(fmt,##args)
#else
	#define GALLEN_DECLARE_INT(name)	
	#define GALLEN_DECLARE_EXTINT(name)	
	#define GALLEN_SET_INT(name,a)	
	#define GALLEN_GET_INT(name)		
	#define GALLEN_DBGLOCAL_BEGIN()	
	#define GALLEN_DBGLOCAL_BEGIN_EX(bits)	
	#define GALLEN_DBGLOCAL_MUTEBEGIN()	
	#define GALLEN_DBGLOCAL_MUTEBEGIN_EX(bits)	
	#define GALLEN_DBGLOCAL_END() 
	#define GALLEN_DBGLOCAL_MUTEEND()	
	#define GALLEN_DBGLOCAL_ESC() 
	#define GALLEN_DBGLOCAL_RUNLOG(b)	
	#define GALLEN_DBGLOCAL_SAVELOGS()	
	#define GALLEN_DBGLOCAL_PRINTLOG() 
	#define GALLEN_DBGLOCAL_PRINTMSG(fmt,args...)
	#define GALLEN_DBGLOCAL_PRINTDIFFLOG()	
	#define GALLEN_DBGLOCAL_DBGLVL_INC()	
	#define GALLEN_DBGLOCAL_DBGLVL_DEC()	
	#define GALLEN_DBGLOCAL_DBGLVL_ADD(addval)
	#define GALLEN_DBGLOCAL_DBGLVL_SET(lvl)	
	#define GALLEN_GDBGLVL_INC()	
	#define GALLEN_GDBGLVL_DEC()	
	#define GALLEN_GDBGLVL_SET(lvl)	
	#define GALLEN_GDBGLVL_GET()	0
	#define GALLEN_GDBGLVL_ADD(addval)	
	#define GALLEN_DBGLOCAL_L0_BEGIN()	
	#define GALLEN_DBGLOCAL_L0_END() 
	#define GALLEN_DBGLOCAL_L0_RUNLOG(b)	
	#define GALLEN_DBGLOCAL_DIFF_BEGIN(name)	
	#define GALLEN_DBGLOCAL_DIFF_BEGIN_EX(name,bits)	
	#define GALLEN_DBGLOCAL_DIFF_END(name) 
	#define GALLEN_DBGLOCAL_DIFF_ESC(name)	
	#define GALLEN_DBGLOCAL_DIFF_RUNLOG(name,b)	
	
	#define dbgENTER()		
	#define dbgLEAVE()		
	#define DBG_MSG(fmt,args...)					
	#define DBG_MSG_FLUSH(fmt,args...)		
	#define ERR_MSG(fmt,args...)					GALLEN_ERR_PRINT(fmt,##args)
	#define WARNNING_MSG(fmt,args...)			GALLEN_WARNING_PRINT(fmt,##args)
	#define WARNING_MSG(fmt,args...)			GALLEN_WARNING_PRINT(fmt,##args)
#endif

#define DBG0_MSG(fmt,args...)					GALLEN_PRINT(fmt,##args)
#define DBG0_ENTRY_TAG() printk("%s(%d):%s\n",__FILE__,__LINE__,__FUNCTION__)

#define ASSERT(p)	assert(p)

#ifdef __KERNEL__//[
#ifdef CONFIG_KERNEL_ASSERTS//[
	/* kgdb stuff */
	#define assert(p) KERNEL_ASSERT(#p, p)
#else//][!CONFIG_KERNEL_ASSERTS
	#define assert(p) do {  \
		if (!(p)) {     \
			GALLEN_PRINT(KERN_CRIT "BUG at %s:%d assert(%s)\n",   \
				__FILE__, __LINE__, #p);                 \
				BUG();  \
		}               \
	} while (0)
#endif//] CONFIG_KERNEL_ASSERTS
#else //][!__KERNEL__
	#include <assert.h>
#endif //]__KERNEL__




#if (GDEBUG >= 1) //[
#define DBGVAR_DECLARE(__name)	DBGVAR_DECLARE_EX(__name,32)
#define DBGVAR_DECLARE_EX(__name,_bits)	\
	const int gi##__name##RunlogBits=_bits;\
	const int gi##__name##RunlogBytes=((_bits/8)+1);\
	volatile unsigned char gb##__name##RunLogsA[((_bits/8)+1)] = {0,}
	
#define DBGVAR_SETBIT(__name,bit)	\
	if((bit)<gi##__name##RunlogBits) {\
		BUF_SETBIT(gb##__name##RunLogsA,bit);\
	}\
	else {\
		GALLEN_PRINT("%s(%d):[ERROR]DBGVAR_SETBIT %s cannot >= %d\n",__FILE__,__LINE__,#__name,gi##__name##RunlogBits);\
	}\
	
#define DBGVAR_CLRBIT(__name,bit)	\
	if((bit)<gi##__name##RunlogBits) {\
		BUF_CLRBIT(gb##__name##RunLogsA,bit);\
	}\
	else {\
		GALLEN_PRINT("%s(%d):[ERROR]DBGVAR_CLRBIT %s cannot >= %d\n",__FILE__,__LINE__,#__name,gi##__name##RunlogBits);\
	}\

#define DBGVAR_PRINT(__name)		BUF_PRINT_ONBITS(gb##__name##RunLogsA,gi##__name##RunlogBits)
	
#else //][!(GDEBUG >= 1)

#define DBGVAR_DECLARE(__name)	
#define DBGVAR_DECLARE_EX(__name,bits)	
#define DBGVAR_SETBIT(__name,bit)	
#define DBGVAR_CLRBIT(__name,bit)	
#define DBGVAR_PRINT(__name)		

#endif //] (GDEBUG >= 1)



#endif //]_GALLEN_DBG__H
