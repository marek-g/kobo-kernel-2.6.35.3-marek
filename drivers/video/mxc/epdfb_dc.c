/***************************************************
 * EPD frame buffer device content manager (4 bits).
 * 
 * Author : Gallen Lin 
 * Data : 2011/01/20 
 * Revision : 1.0
 * File Name : epdfb_dc.c 
 * 
****************************************************/

#include "epdfb_dc.h"




#ifdef __KERNEL__//[
	#include <linux/module.h>
	#include <linux/kernel.h>
	#include <linux/errno.h>
	#include <linux/string.h>
	#include <linux/delay.h>
	#include <linux/interrupt.h>
	#include <linux/fb.h>
	
	#define GDEBUG 0
	#include <linux/gallen_dbg.h>
	#define my_malloc(sz)	kmalloc(sz,GFP_KERNEL)
	#define my_free(p)		kfree(p)
	#define my_memcpy(dest,src,size)	memcpy(dest,src,size)
	
#else //][!__KERNEL__
	#include <stdio.h>
	#include <stdlib.h>
	#include <string.h>
	#define GDEBUG 0
	#include "gallen_dbg.h"
	#define my_malloc(sz)	malloc(sz)
	#define my_free(p)		free(p)
	#define my_memcpy(dest,src,size)	memcpy(dest,src,size)
#endif//] __KERNEL__


#define EPDFB_MAGIC 	0x15a815a8


#define CHK_EPDFB_DC(pEPD_DC)	\
(pEPD_DC->dwMagicPrivateBegin==EPDFB_MAGIC)&&(pEPD_DC->dwMagicPrivateEnd==EPDFB_MAGIC)



static void _fb_gray_8to4(unsigned char *data, unsigned long len)
{
	unsigned char *pb = (unsigned char *)data ;
	unsigned long dwByteRdIdx,dwByteWrIdx;
	unsigned char bTemp;
	unsigned long dwLenNew = len;

	DBG_MSG("%s(%d):data=%p,len=%d\n",\
			__FUNCTION__,__LINE__,data,len);

	for(dwByteRdIdx=0,dwByteWrIdx=0;1;)
	{
		bTemp = (pb[dwByteRdIdx]>>4)&0x0f;
		bTemp = bTemp<<4;
		if(++dwByteRdIdx>=len){
			break;
		}
		bTemp |= (pb[dwByteRdIdx]>>4)&0x0f;
		dwLenNew-=1;
		pb[++dwByteWrIdx] = bTemp;

		if(++dwByteRdIdx>=len) {
			break;
		}
	}

}

// get blue value to gray value .
#define _RGB565_to_Gray8(_wRGB_565)	(((unsigned char)(_wRGB_565)&0x001f)<<3)
#define _RGB565_to_Gray4(_wRGB_565)	(((unsigned char)(_wRGB_565)&0x001f)>>1)

#define _RGB565(_r,_g,_b)	(unsigned short)((_r)<<11|(_g)<<6|(_b))

static const unsigned short gwGray4toRGB565_TableA[] = {
	_RGB565(0x00,0x00,0x00),
	_RGB565(0x02,0x02,0x02),
	_RGB565(0x04,0x04,0x04),
	_RGB565(0x06,0x06,0x06),
	_RGB565(0x08,0x08,0x08),
	_RGB565(0x0a,0x0a,0x0a),
	_RGB565(0x0c,0x0c,0x0c),
	_RGB565(0x0e,0x0e,0x0e),
	_RGB565(0x10,0x10,0x10),
	_RGB565(0x12,0x12,0x12),
	_RGB565(0x14,0x14,0x14),
	_RGB565(0x16,0x16,0x16),
	_RGB565(0x18,0x18,0x18),
	_RGB565(0x1a,0x1a,0x1a),
	_RGB565(0x1c,0x1c,0x1c),
	_RGB565(0x1e,0x1e,0x1e),
};
static const unsigned short gwGray2toRGB565_TableA[] = {
	_RGB565(0x00,0x00,0x00),
	_RGB565(0x0a,0x0a,0x0a),
	_RGB565(0x14,0x14,0x14),
	_RGB565(0x1e,0x1e,0x1e),
};

static const unsigned char gbGray4toGray8_TableA[] = {
	0x00,
	0x10,
	0x20,
	0x30,
	0x40,
	0x50,
	0x60,
	0x70,
	0x80,
	0x90,
	0xa0,
	0xb0,
	0xc0,
	0xd0,
	0xe0,
	0xf0,
};

static void _fb_Gray4toRGB565(
	unsigned char *IO_pbRGB565Buf,unsigned long I_dwRGB565BufSize,
	unsigned char *I_pb4BitsSrc,unsigned long I_dw4BitsBufSize,int iIsPixelSwap)
{
	unsigned long dwRd;
	unsigned short *pwWrBuf = (unsigned short *)IO_pbRGB565Buf;
	unsigned char *pbRdSrc = I_pb4BitsSrc;
	
	int iIdxDot1,iIdxDot2;
	
	
	for(dwRd=0;dwRd<I_dw4BitsBufSize;dwRd++)
	{
		
		if(iIsPixelSwap) {
			iIdxDot1 = (pbRdSrc[dwRd]>>4)&0xf;
			iIdxDot2 = (pbRdSrc[dwRd])&0xf;
		}
		else {
			iIdxDot1 = (pbRdSrc[dwRd])&0xf;
			iIdxDot2 = (pbRdSrc[dwRd]>>4)&0xf;
		}
		
		*pwWrBuf++ = gwGray4toRGB565_TableA[iIdxDot1];
		*pwWrBuf++ = gwGray4toRGB565_TableA[iIdxDot2];
	}
}

static void _fb_Gray8toRGB565(
	unsigned char *IO_pbRGB565Buf,unsigned long I_dwRGB565BufSize,
	unsigned char *I_pb8BitsSrc,unsigned long I_dw8BitsBufSize)
{
	unsigned long dwRd,dwWr;
	unsigned short *pwWrBuf = (unsigned short *)IO_pbRGB565Buf;
	unsigned char *pbRdSrc = I_pb8BitsSrc;
	
	
	
	for(dwRd=0;dwRd<I_dw8BitsBufSize;dwRd++)
	{
		*pwWrBuf++ = (*pbRdSrc++)<<8;
		if(++dwWr>=I_dwRGB565BufSize) {
			ERR_MSG("%s(%d):write buffer too small \n",__FUNCTION__,__LINE__);
			break;
		}
	}
}

static void _fb_RGB565toGray4(
	unsigned char *IO_pbGray4Buf,unsigned long I_dwGray4BufSize,
	unsigned char *I_pbRGB565Buf,unsigned long I_dwRGB565BufSize,int iIsPixelSwap)
{
	unsigned long dwRd,dwWr;
	unsigned char *pbWrBuf = (unsigned char *)IO_pbGray4Buf;
	unsigned short *pwRdSrc = (unsigned short *)I_pbRGB565Buf;
	
	unsigned char bDot1,bDot2;
	
	ASSERT(!(I_dwRGB565BufSize&1));
	for(dwRd=0;dwRd<I_dwRGB565BufSize;dwRd+=2)
	{
		bDot1 = (*pwRdSrc) >> 12 ;
		pwRdSrc ++ ;
		bDot2 = (*pwRdSrc) >> 12 ;
		pwRdSrc ++ ;
		
		if(iIsPixelSwap) {
			*pbWrBuf = bDot2&(bDot1<<4) ;
		}
		else {
			*pbWrBuf = (bDot2<<4)&bDot1 ;
		}
		pbWrBuf++;
		if(++dwWr >=  I_dwGray4BufSize) {
			ERR_MSG("%s(%d):write buffer too small \n",__FUNCTION__,__LINE__);
			break;
		}
	}
}


static void _fb_gray_4to8(
	unsigned char *IO_pb8BitsBuf,unsigned long I_dw8BitsBufSize,
	unsigned char *I_pb4BitsSrc,unsigned long I_dw4BitsBufSize,int iIsPixelSwap)
{
	unsigned long dwWr,dwRd;
	
	for(dwWr=0,dwRd=0;dwRd<I_dw4BitsBufSize;dwRd++)
	{
		if(iIsPixelSwap) {
			if(dwWr>=I_dw8BitsBufSize) {
				ERR_MSG("%s(%d):write buffer too small \n",__FUNCTION__,__LINE__);
				break;
			}
			IO_pb8BitsBuf[dwWr++] = (I_pb4BitsSrc[dwRd]<<4)&0xf0;
			
			if(dwWr>=I_dw8BitsBufSize) {
				ERR_MSG("%s(%d):write buffer too small \n",__FUNCTION__,__LINE__);
				break;
			}
			IO_pb8BitsBuf[dwWr++] = I_pb4BitsSrc[dwRd]&0xf0;
		}
		else {
			if(dwWr>=I_dw8BitsBufSize) {
				ERR_MSG("%s(%d):write buffer too small \n",__FUNCTION__,__LINE__);
				break;
			}
			IO_pb8BitsBuf[dwWr++] = I_pb4BitsSrc[dwRd]&0xf0;
			
			if(dwWr>=I_dw8BitsBufSize) {
				ERR_MSG("%s(%d):write buffer too small \n",__FUNCTION__,__LINE__);
				break;
			}
			IO_pb8BitsBuf[dwWr++] = (I_pb4BitsSrc[dwRd]<<4)&0xf0;
		}
	}
}

EPDFB_DC *epdfbdc_create_ex2(unsigned long dwFBW,unsigned long dwFBH,\
	unsigned long dwW,unsigned long dwH,\
	unsigned char bPixelBits,unsigned char *pbDCbuf,unsigned long dwCreateFlag)
{
	EPDFB_DC *pDC = 0;
	unsigned long dwFBBits = (dwFBW*dwFBH*bPixelBits);
	unsigned long dwFBWBits = (dwFBW*bPixelBits);
	unsigned long dwDCSize ,dwDCWidthBytes;

	dwDCSize=dwFBBits>>3;
	dwDCWidthBytes=dwFBWBits>>3;
	
	if(dwFBWBits&0x7) {
		dwDCWidthBytes+=1;
		dwDCSize+=dwFBH;
	}
	
	DBG_MSG("%s(%d):w=%d,h=%d,bits=%d,bufp=%p\n",__FUNCTION__,__LINE__,\
		dwW,dwH,bPixelBits,pbDCbuf);
	
	pDC = my_malloc(sizeof(EPDFB_DC));
	if(0==pDC) {
		return 0;
	}

	
	if(pbDCbuf) {
		pDC->dwBufSize = 0;
		pDC->pbDCbuf = pbDCbuf;
		
		DBG_MSG("%s(%d):pDC malloc=%p\n",__FUNCTION__,__LINE__,pDC);
	}
	else {
		pbDCbuf = my_malloc(dwDCSize);
		if(0==pbDCbuf) {
			my_free(pDC);
			return 0;
		}
		DBG_MSG("%s(%d):pDC malloc=%p,pbDCbuf=%p,sz=%u\n",
			__FUNCTION__,__LINE__,pDC,pbDCbuf,dwDCSize);
		memset(pbDCbuf,0,dwDCSize);
		pDC->pbDCbuf = pbDCbuf;
		pDC->dwBufSize = dwDCSize;
	}
	ASSERT(dwFBW>=dwW);
	pDC->dwFBWExtra = dwFBW-dwW ;
	ASSERT(dwFBH>=dwH);
	pDC->dwFBHExtra = dwFBH-dwH ;
	pDC->dwWidth = dwW;
	pDC->dwHeight = dwH;
	pDC->bPixelBits = bPixelBits;
	pDC->dwDCSize = dwDCSize;
	
	pDC->dwDCWidthBytes = dwDCWidthBytes;
	
	pDC->dwDirtyOffsetStart = 0;
	pDC->dwDirtyOffsetEnd = dwDCSize;
	
	pDC->dwMagicPrivateBegin = EPDFB_MAGIC;
	pDC->dwMagicPrivateEnd = EPDFB_MAGIC;
	pDC->dwFlags = dwCreateFlag;
	
	pDC->pfnGetWaveformBpp = 0;
	pDC->pfnVcomEnable = 0;
	pDC->pfnSetPartialUpdate = 0;
	pDC->pfnGetRealFrameBuf = 0;
	pDC->pfnGetRealFrameBufEx = 0;
	pDC->pfnDispStart = 0;
	pDC->pfnGetWaveformMode = 0;
	pDC->pfnSetWaveformMode = 0;
	pDC->pfnIsUpdating = 0;
	pDC->pfnWaitUpdateComplete = 0;
	pDC->pfnSetUpdateRect = 0;
	pDC->pfnPwrAutoOffIntervalMax = 0;
	pDC->pfnAutoOffEnable = 0;
	pDC->pfnPutImg = 0;
	pDC->pfnSetVCOM = 0;
	pDC->pfnSetVCOMToFlash = 0;
	pDC->pfnGetVCOM = 0;
	
	pDC->iIsForceWaitUpdateFinished = 1;
	
	pDC->pfnPwrOnOff = 0;
	
	pDC->iWFMode = -1;
	pDC->iLastWFMode = -1;
	
	return pDC;
}

EPDFB_DC *epdfbdc_create_ex(unsigned long dwW,unsigned long dwH,\
	unsigned char bPixelBits,unsigned char *pbDCbuf,unsigned long dwCreateFlag)
{
	return epdfbdc_create_ex2(dwW,dwH,dwW,dwH,bPixelBits,pbDCbuf,dwCreateFlag);
}



EPDFB_DC *epdfbdc_create(unsigned long dwW,unsigned long dwH,\
	unsigned char bPixelBits,unsigned char *pbDCbuf)
{
	return epdfbdc_create_ex(dwW,dwH,bPixelBits,pbDCbuf,EPDFB_DC_FLAG_DEFAUT);
}

	
EPDFB_DC_RET epdfbdc_delete(EPDFB_DC *pEPD_dc)
{
	DBG_MSG("%s\n",__FUNCTION__);
	if(!CHK_EPDFB_DC(pEPD_dc)) {
		ERR_MSG("%s(%d): object handle error !\n",__FUNCTION__,__LINE__);
		return EPDFB_DC_OBJECTERR;
	}
	
	if(pEPD_dc->dwBufSize>0) {
		my_free((void *)pEPD_dc->pbDCbuf);
		pEPD_dc->pbDCbuf = 0;
	}
	my_free(pEPD_dc);
	return EPDFB_DC_SUCCESS;
}

/*
 * 
 * 
 * 
 */
EPDFB_DC_RET epdfbdc_fbimg_normallize(EPDFB_DC *I_pEPD_dc,\
	EPDFB_IMG *IO_pEPD_img)
{
	EPDFB_DC_RET tRet=EPDFB_DC_PIXELBITSNOTSUPPORT;
	unsigned char *pbBuf = 0;
	unsigned long dwBufSize ;
	
	if(!CHK_EPDFB_DC(I_pEPD_dc)) {
		return EPDFB_DC_OBJECTERR;
	}
	
	if(IO_pEPD_img->bPixelBits!=I_pEPD_dc->bPixelBits) {
		switch(IO_pEPD_img->bPixelBits) { // source image 
		case 16:
			if(4==I_pEPD_dc->bPixelBits) {
				pbBuf = IO_pEPD_img->pbImgBuf;
				dwBufSize = IO_pEPD_img->dwW*IO_pEPD_img->dwH*IO_pEPD_img->bPixelBits/8;
				_fb_RGB565toGray4(IO_pEPD_img->pbImgBuf,dwBufSize,
					pbBuf,dwBufSize,0);
				tRet=EPDFB_DC_SUCCESS;
			}
			else if(8==I_pEPD_dc->bPixelBits) {
			}
			else if (16==I_pEPD_dc->bPixelBits){
				tRet=EPDFB_DC_SUCCESS;
			}
			break;
		case 8:
			if(4==I_pEPD_dc->bPixelBits) {
				_fb_gray_8to4(IO_pEPD_img->pbImgBuf,\
					IO_pEPD_img->dwW*IO_pEPD_img->dwH );
				IO_pEPD_img->bPixelBits = 4;
				tRet=EPDFB_DC_SUCCESS;
			}
			else if(16==I_pEPD_dc->bPixelBits) {
				dwBufSize = IO_pEPD_img->dwW*IO_pEPD_img->dwH*I_pEPD_dc->bPixelBits/8;
				pbBuf = my_malloc(dwBufSize);
				if(!pbBuf) {
					return EPDFB_DC_MEMMALLOCFAIL;
				}
				memcpy(pbBuf,IO_pEPD_img->pbImgBuf,IO_pEPD_img->dwW*IO_pEPD_img->dwH);
				_fb_Gray8toRGB565(IO_pEPD_img->pbImgBuf,\
					IO_pEPD_img->dwW*IO_pEPD_img->dwH,pbBuf,dwBufSize);
				my_free(pbBuf);pbBuf=0;
				
			}
			else if(8==I_pEPD_dc->bPixelBits) {
				tRet=EPDFB_DC_SUCCESS;
			}
			break;
		case 4:
			dwBufSize = IO_pEPD_img->dwW*IO_pEPD_img->dwH*I_pEPD_dc->bPixelBits/8;
			pbBuf = my_malloc(dwBufSize);
			if(!pbBuf) {
				return EPDFB_DC_MEMMALLOCFAIL;
			}
			if(8==I_pEPD_dc->bPixelBits) {
				memcpy(pbBuf,IO_pEPD_img->pbImgBuf, \
					IO_pEPD_img->dwW*IO_pEPD_img->dwH*IO_pEPD_img->bPixelBits/8);
				_fb_gray_4to8(IO_pEPD_img->pbImgBuf,IO_pEPD_img->dwW*IO_pEPD_img->dwH,
					pbBuf,dwBufSize,0);
				tRet=EPDFB_DC_SUCCESS;
			}
			else if(16==I_pEPD_dc->bPixelBits) {
				memcpy(pbBuf,IO_pEPD_img->pbImgBuf, \
					IO_pEPD_img->dwW*IO_pEPD_img->dwH*IO_pEPD_img->bPixelBits/8);
				_fb_Gray4toRGB565(IO_pEPD_img->pbImgBuf,
					IO_pEPD_img->dwW*IO_pEPD_img->dwH*IO_pEPD_img->bPixelBits/8,
					pbBuf,dwBufSize,0);
				tRet=EPDFB_DC_SUCCESS;				
			}
			else if(4==I_pEPD_dc->bPixelBits) {
				tRet=EPDFB_DC_SUCCESS;
			}
			my_free(pbBuf);pbBuf=0;
			break;
		default :
			//tRet = EPDFB_DC_PIXELBITSNOTSUPPORT;
			break;
		}
	}
	return tRet;
}


static inline unsigned long _pixel_value_convert(unsigned long dwPixelVal,
	unsigned char bSrcBitsPerPixel,unsigned char bDestBitsPerPixel)
{
	unsigned long dwRetPixelVal;
	
	switch(bSrcBitsPerPixel) 
	{
	case 1:
		dwRetPixelVal = dwPixelVal?0x0:0xffffffff;
		break;	
	case 2:
		if(4==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal<<2;
		}
		else if(8==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal<<6;
		}
		else if(16==bDestBitsPerPixel) {
			dwRetPixelVal = (unsigned long)(gwGray2toRGB565_TableA[(dwPixelVal&0x3)]);
			//printk("%x->%x.",dwPixelVal,dwRetPixelVal);
		}
		else if(1==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal>>1;
		}
		else if(2==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal;
		}
		break;
	case 4:
		if(4==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal;
		}
		else if(8==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal<<4;
			//dwRetPixelVal = (unsigned long)gbGray4toGray8_TableA[(dwPixelVal&0xf)];
		}
		else if(16==bDestBitsPerPixel) {
			//dwRetPixelVal = dwPixelVal<<12;
			dwRetPixelVal = (unsigned long)(gwGray4toRGB565_TableA[(dwPixelVal&0xf)]);
		}
		else if(1==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal>>3;
		}
		else if(2==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal>>2;
		}
		break;
	case 8:
		if(4==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal>>4;
		}
		else if(8==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal;
		}
		else if(16==bDestBitsPerPixel) {
			//dwRetPixelVal = dwPixelVal<<8;
			dwRetPixelVal = (unsigned long)(gwGray4toRGB565_TableA[(dwPixelVal>>4)]);
		}
		else if(1==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal>>7;
		}
		else if(2==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal>>6;
		}
		break;
	case 16:
		if(4==bDestBitsPerPixel) {
			dwRetPixelVal = (unsigned long)_RGB565_to_Gray4(dwPixelVal);
		}
		else if(8==bDestBitsPerPixel) {
			dwRetPixelVal = (unsigned long)_RGB565_to_Gray8(dwPixelVal);
		}
		else if(16==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal;
		}
		else if(1==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal>>31;
		}
		else if(2==bDestBitsPerPixel) {
			dwRetPixelVal = dwPixelVal>>30;
		}
		break;
	default:
		WARNING_MSG("convert %d -> %d not supported \n",bSrcBitsPerPixel,bDestBitsPerPixel);
		dwRetPixelVal = dwPixelVal;
		break;
	}
	
	return dwRetPixelVal;
}



static inline unsigned char *_epdfbdc_get_dcimg_ptr(
	EPDFB_DC *I_pEPD_imgdc,	unsigned long I_dwImgX,unsigned long I_dwImgY)
{
	unsigned char *pbRet ;
	unsigned char bBitsPerPixel = I_pEPD_imgdc->bPixelBits;

	if(4==bBitsPerPixel) {
		pbRet = I_pEPD_imgdc->pbDCbuf + 
			(I_dwImgX>>1)+(I_dwImgY*I_pEPD_imgdc->dwDCWidthBytes);
	}
	else if(8==bBitsPerPixel) {
		pbRet = (unsigned char *)(I_pEPD_imgdc->pbDCbuf+(I_dwImgX)+\
			(I_dwImgY*I_pEPD_imgdc->dwDCWidthBytes));
	}
	else if(16==bBitsPerPixel) {
		pbRet = (unsigned char *)(I_pEPD_imgdc->pbDCbuf+(I_dwImgX>>1)+\
			(I_dwImgY*I_pEPD_imgdc->dwDCWidthBytes));
	}
	else if(1==bBitsPerPixel) {
		pbRet =(unsigned char *)(I_pEPD_imgdc->pbDCbuf+(I_dwImgX>>3)+\
			(I_dwImgY*I_pEPD_imgdc->dwDCWidthBytes));
	}
	else if(2==bBitsPerPixel) {
		pbRet =(unsigned char *)(I_pEPD_imgdc->pbDCbuf+(I_dwImgX>>2)+\
			(I_dwImgY*I_pEPD_imgdc->dwDCWidthBytes));
	}
	else {
		pbRet = 0;
	}
	return pbRet;
}

static inline unsigned long _epdfbdc_get_img_pixelvalue_from_ptr(EPDFB_DC *I_pEPD_imgdc,
	unsigned char **IO_ppbImg,unsigned long I_dwX)
{
	unsigned long dwRet;
	unsigned char bBitsPerPixel=I_pEPD_imgdc->bPixelBits;
	
	ASSERT(IO_ppbImg);
	ASSERT(*IO_ppbImg);
	ASSERT(*IO_ppbImg>=I_pEPD_imgdc->pbDCbuf);
	
	if((*IO_ppbImg)>I_pEPD_imgdc->pbDCbuf+I_pEPD_imgdc->dwDCSize) {
		printk("[Warning]*IO_ppbImg=%p>pbDCbuf=%p+dwDCSize=%u\n",(*IO_ppbImg),
			I_pEPD_imgdc->pbDCbuf,I_pEPD_imgdc->dwDCSize);
		return 0;
		ASSERT(0);
	}
	
	switch(bBitsPerPixel) {
	case 4:
	{
		unsigned char b;
		if(I_dwX&1) {
			if(I_pEPD_imgdc->dwFlags&EPDFB_DC_FLAG_REVERSEINPDATA) {
				b = (**IO_ppbImg)&0xf;
			}
			else {
				b = ((**IO_ppbImg)>>4)&0xf;
			}
			(*IO_ppbImg) += 1;
		}
		else {
			if(I_pEPD_imgdc->dwFlags&EPDFB_DC_FLAG_REVERSEINPDATA) {
				b = ((**IO_ppbImg)>>4)&0xf;
			}
			else {
				b = (**IO_ppbImg)&0xf;
			}
		}
		dwRet = (unsigned long)b;
		
		break;
	}
	case 1:
	{
		unsigned char b=(I_dwX&0x7);
		unsigned char bBitMask=0x80>>b;
		
		dwRet = (unsigned long)((**IO_ppbImg)&bBitMask);
		if(7==b) {
			(*IO_ppbImg) += 1;
		}
		break;
	}
	case 2:
	{
		unsigned char b=(I_dwX&0x3);
		unsigned char bShift=6-(b<<1);
		
		/*
		if(I_pEPD_imgdc->dwFlags&EPDFB_DC_FLAG_REVERSEINPDATA) {

		}
		else {
			switch(bShift) {
			case 0:
				bShift = 6;
				break;
			case 2:
				bShift = 2;
				break;
			case 4:
				bShift = 4;
				break;
			case 6:
				bShift = 0;
				break;
			default :
				break;
			}		
		}
		*/
		
		dwRet = (unsigned long)(((**IO_ppbImg)>>bShift)&0x03);
		
		if(3==b) {
			(*IO_ppbImg) += 1;
		}
		break;
	}
	case 8:
	{
		dwRet = (unsigned long)(**((unsigned char **)IO_ppbImg));
		(*IO_ppbImg) += 1;
		break;
	}
	case 16:
	{
		dwRet = (unsigned long)(**((unsigned short **)IO_ppbImg));
		(*IO_ppbImg) += 2;
		break;
	}
	default:
		ERR_MSG("%d Bits/Pixel not supported !!\n",bBitsPerPixel);
		dwRet=0;
		break;
	}
	return dwRet;
}

static inline void _epdfbdc_set_dcpixel_at_ptr(EPDFB_DC *I_pEPD_dc,
	unsigned char **IO_ppbDCDest,unsigned long I_dwSrcPixelVal,unsigned char I_bSrcBitsPerPixel,
	unsigned short I_dwDestX)
{
	unsigned long L_dwPixelVal;
	unsigned char bDestBitsPerPixel;
	
	ASSERT(IO_ppbDCDest);
	ASSERT(*IO_ppbDCDest);
	
	ASSERT(*IO_ppbDCDest>=I_pEPD_dc->pbDCbuf);
	ASSERT(*IO_ppbDCDest<I_pEPD_dc->pbDCbuf+I_pEPD_dc->dwDCSize);
	
	bDestBitsPerPixel = I_pEPD_dc->bPixelBits;
	L_dwPixelVal = _pixel_value_convert(I_dwSrcPixelVal,I_bSrcBitsPerPixel,bDestBitsPerPixel);
	
	switch(bDestBitsPerPixel) {
	case 4:
		{
		unsigned char bTemp,bPixelVal;
		
		bPixelVal=(unsigned char)(L_dwPixelVal&0xf);
		bTemp = **IO_ppbDCDest;
		
		if(I_pEPD_dc->dwFlags&EPDFB_DC_FLAG_REVERSEDRVDATA) {
			if(I_dwDestX&0x1) {
				bTemp &= 0x0f;
				bTemp |= (bPixelVal<<4)&0xf0 ;
				**IO_ppbDCDest = bTemp;
				(*IO_ppbDCDest) += 1;
			}
			else {
				bTemp &= 0xf0;
				bTemp |= bPixelVal;
				**IO_ppbDCDest = bTemp;
			}
		}	
		else {
			if(I_dwDestX&0x1) {
				bTemp &= 0xf0;
				bTemp |= bPixelVal;
				**IO_ppbDCDest = bTemp;
				(*IO_ppbDCDest) += 1;
			}
			else {
				bTemp &= 0x0f;
				bTemp |= (bPixelVal<<4)&0xf0 ;
				**IO_ppbDCDest = bTemp;
			}
		}
			
		
		break;
		}
	case 1:
		{
		unsigned char bTemp;
		unsigned char b=(I_dwDestX&0x7);
		
		bTemp = **IO_ppbDCDest;
		if(L_dwPixelVal) {
			bTemp |= 0x01<<b;
		}
		else {
			bTemp &= ~(0x01<<b);
		}
		**IO_ppbDCDest = bTemp;	
		if(7==b) {
			(*IO_ppbDCDest) += 1;
		}		
		break;
		}
	case 2:
		{
		unsigned char bTemp;
		unsigned char b=(I_dwDestX&0x3);
		
		bTemp = **IO_ppbDCDest;
		bTemp &= ~(0x3<<(b<<1));
		bTemp |= L_dwPixelVal<<(b<<1);
		**IO_ppbDCDest = bTemp;	
		if(3==b) {
			(*IO_ppbDCDest) += 1;
		}		
		break;
		}		
	case 8:
		{
		**((unsigned char **)IO_ppbDCDest) = (unsigned char)(L_dwPixelVal&0x000000ff);
		(*IO_ppbDCDest) += 1;
		break;
		}
	case 16:
		{
		**((unsigned short **)IO_ppbDCDest) = (unsigned short)(L_dwPixelVal&0x0000ffff);
		(*IO_ppbDCDest) += 2;
		break;
		}
		
	default:
		break;
	
	}
}



static inline unsigned long _epdfbdc_get_img_pixelvalue(EPDFB_DC *I_pEPD_dc,
	EPDFB_IMG *I_pEPD_img,unsigned long dwX,unsigned long dwY,unsigned long dwImgWidthBytes)
{
	unsigned long dwRet = 0;
	unsigned char bBitsPerPixel = I_pEPD_img->bPixelBits;
	//unsigned long dwImgWidthBytes ;
	
	if(4==bBitsPerPixel) {
		unsigned char *pb;
		unsigned char b;
		
		pb = I_pEPD_img->pbImgBuf+
			(dwX>>1)+(dwY*dwImgWidthBytes);
			
		if(dwX&1) {
			if(I_pEPD_dc->dwFlags&EPDFB_DC_FLAG_REVERSEINPDATA) {
				b = (*pb)&0xf;
			}
			else {
				b = ((*pb)>>4)&0xf;
			}
		}
		else {
			if(I_pEPD_dc->dwFlags&EPDFB_DC_FLAG_REVERSEINPDATA) {
				b = ((*pb)>>4)&0xf;
			}
			else {
				b = (*pb)&0xf;
			}
		}
		dwRet = (unsigned long)b;
	}
	else if(1==bBitsPerPixel) {
		unsigned char *pb =
		((unsigned char *)(I_pEPD_img->pbImgBuf+(dwX>>3)+(dwY*dwImgWidthBytes)));
		unsigned char bBitMask=0x80>>(dwX&0x7);
		dwRet = (unsigned long)((*pb)&bBitMask);
	}
	else if(8==bBitsPerPixel) {
		dwRet = (unsigned long)
		*((unsigned char *)(I_pEPD_img->pbImgBuf+(dwX)+(dwY*dwImgWidthBytes)));
	}
	else if(16==bBitsPerPixel) {
		dwRet = (unsigned long)
		*((unsigned short *)(I_pEPD_img->pbImgBuf+(dwX>>1)+(dwY*dwImgWidthBytes)));
	}
	else {
	}
	
	return dwRet;
}

static inline void _epdfbdc_set_pixel(EPDFB_DC *pEPD_dc,
	unsigned long dwX,unsigned long dwY,unsigned long dwPixelVal,unsigned char bSrcBitsPerPixel)
{
	unsigned long L_dwPixelVal;
	unsigned char bDestBitsPerPixel;
	
	L_dwPixelVal = _pixel_value_convert(dwPixelVal,bSrcBitsPerPixel,pEPD_dc->bPixelBits);
	bDestBitsPerPixel = pEPD_dc->bPixelBits;
	
	//DBG_MSG("(%d,%d)=0x%x",dwX,dwY,L_dwPixelVal);
	
	if( 4 == bDestBitsPerPixel ) {
		volatile unsigned char *pbDCDest;
		unsigned char bTemp,bPixelVal;
		
		
		//DBG_MSG("%s(),x=0x%x,y=0x%x,pixel=0x%x\n",__FUNCTION__,dwX,dwY,dwPixelVal);
		
		pbDCDest = pEPD_dc->pbDCbuf + \
					(dwX>>1)+(dwY*pEPD_dc->dwDCWidthBytes);
		
		bPixelVal=(unsigned char)(L_dwPixelVal&0xf);
		bTemp = *pbDCDest;
		
		if(pEPD_dc->dwFlags&EPDFB_DC_FLAG_REVERSEDRVDATA) {
			if(dwX&0x1) {
				bTemp &= 0x0f;
				bTemp |= (bPixelVal<<4)&0xf0 ;
			}
			else {
				bTemp &= 0xf0;
				bTemp |= bPixelVal;
			}
		}	
		else {
			if(dwX&0x1) {
				bTemp &= 0xf0;
				bTemp |= bPixelVal;
			}
			else {
				bTemp &= 0x0f;
				bTemp |= (bPixelVal<<4)&0xf0 ;
			}
		}
		
		*pbDCDest = bTemp;
	}
	else if( 16 == bDestBitsPerPixel ) {
		volatile unsigned short *pwDCDest;
		
		pwDCDest = (unsigned short *)(pEPD_dc->pbDCbuf + \
					(dwX<<1)+(dwY*pEPD_dc->dwDCWidthBytes));
		*pwDCDest = (unsigned short)(L_dwPixelVal&0x0000ffff);
	}
	else if( 8 == bDestBitsPerPixel ) {
		volatile unsigned char *pbDCDest;
		
		pbDCDest = (unsigned short *)(pEPD_dc->pbDCbuf + \
					(dwX)+(dwY*pEPD_dc->dwDCWidthBytes));
		*pbDCDest = (unsigned char)(L_dwPixelVal&0x000000ff);
	}
	else {
	}
}


//
#define GETIMG_PIXEL_METHOD		2

EPDFB_DC_RET epdfbdc_put_dcimg(EPDFB_DC *pEPD_dc,
	EPDFB_DC *pEPD_dcimg,EPDFB_ROTATE_T tRotateDegree,
	unsigned long I_dwDCimgX,unsigned long I_dwDCimgY,
	unsigned long I_dwDCimgW,unsigned long I_dwDCimgH,
	unsigned long I_dwDCPutX,unsigned long I_dwDCPutY)
{
	EPDFB_DC_RET tRet=EPDFB_DC_SUCCESS;
	
	unsigned long dwDCH,dwDCW;
	
	//unsigned long dwImgW,dwImgH;
	//unsigned long dwImgX,dwImgY;
	
	unsigned char bImgPixelBits;
	unsigned long dwDCWidthBytes;
	unsigned long dwX,dwY,dwEPD_dc_flags;
	unsigned long dwSrcX,dwSrcY;

	unsigned long h,w;// image offset of x,y .
	
	unsigned long dwWHidden,dwHHidden;
	
	unsigned char *pbDCRow;
	unsigned char *pbImgRow;
	
	//int tick=jiffies;
	unsigned long dwImgWidthBytes;

	
	#if (GETIMG_PIXEL_METHOD==1)
	EPDFB_IMG tEPD_img,*pEPD_img=&tEPD_img;
	pEPD_img->dwX = I_dwDCimgX;
	pEPD_img->dwY = I_dwDCimgY;
	pEPD_img->dwW = pEPD_dcimg->dwWidth+pEPD_dcimg->dwFBWExtra;
	pEPD_img->dwH = pEPD_dcimg->dwHeight+pEPD_dcimg->dwFBHExtra;
	pEPD_img->bPixelBits = pEPD_dcimg->bPixelBits;
	pEPD_img->pbImgBuf = pEPD_dcimg->pbDCbuf;
	#endif //] (GETIMG_PIXEL_METHOD==1)
	

	dwImgWidthBytes = pEPD_dcimg->dwDCWidthBytes;//pEPD_dc->bPixelBits/8
	GALLEN_DBGLOCAL_BEGIN();

	if(!CHK_EPDFB_DC(pEPD_dc)) {
		ERR_MSG("%s(%d): object handle error !\n",__FUNCTION__,__LINE__);
		GALLEN_DBGLOCAL_RUNLOG(0);
		GALLEN_DBGLOCAL_ESC();
		return EPDFB_DC_OBJECTERR;
	}
	
	
	
	dwWHidden = pEPD_dc->dwFBWExtra;
	dwHHidden = pEPD_dc->dwFBHExtra;
	dwDCH=pEPD_dc->dwHeight + dwHHidden;
	dwDCW=pEPD_dc->dwWidth + dwWHidden;
	dwEPD_dc_flags = pEPD_dc->dwFlags;
	
	bImgPixelBits=pEPD_dcimg->bPixelBits;
	
	//if(pEPD_dc->bPixelBits!=bImgPixelBits) {
	//	GALLEN_DBGLOCAL_ESC();
	//	return EPDFB_DC_PIXELBITSNOTSUPPORT;
	//}
	
	//ASSERT(4==pEPD_dc->bPixelBits);
	//ASSERT(4==bImgPixelBits);
	//ASSERT(0==(I_dwDCimgW&0x1)); // image width must be even .
	//ASSERT(0==(I_dwDCimgH&0x1)); // image heigh must be even .
	
	
	dwDCWidthBytes = pEPD_dc->dwDCWidthBytes;
	

	DBG_MSG("dc_flag=0x%08x,imgdc.w=%u,imgdc.h=%u,img.w=%u,img.h=%u,img pixbits=%d,dc pixbits=%d\n",
		dwEPD_dc_flags,pEPD_dcimg->dwWidth,pEPD_dcimg->dwHeight,I_dwDCimgW,I_dwDCimgH,pEPD_dcimg->bPixelBits,pEPD_dc->bPixelBits);
	DBG_MSG("IMG_Wbytes=%u,DC_Wbytes=%u\n",dwImgWidthBytes,dwDCWidthBytes);

	switch(tRotateDegree) {
	case EPDFB_R_0:GALLEN_DBGLOCAL_RUNLOG(2);
		// left->right,up->down .

		{
			unsigned long dwPixelVal;
			
			pEPD_dc->dwDirtyOffsetStart = I_dwDCimgY*dwDCWidthBytes;
			pEPD_dc->dwDirtyOffsetEnd = (I_dwDCimgY+I_dwDCimgH)*dwDCWidthBytes;
			
			//dwImgPixelIdx = 0;
			for(h=0;h<I_dwDCimgH;h+=1) {
				
				dwSrcY = I_dwDCimgY+h;
				dwY = I_dwDCPutY+h;
				if(dwY>=dwDCH) {
					continue ;
				}
				//GALLEN_DBGLOCAL_PRINTMSG("y=%d,x=",h);
				
				pbImgRow = _epdfbdc_get_dcimg_ptr(pEPD_dcimg,I_dwDCimgX,dwSrcY);
				
				for(w=0;w<I_dwDCimgW;w+=1) {
					
					dwSrcX = I_dwDCimgX+w;
					dwX = I_dwDCPutX+w;
					if(dwX>=dwDCW) {
						continue ;
					}
					
					//GALLEN_DBGLOCAL_PRINTMSG("%d,",w,h);
					#if (GETIMG_PIXEL_METHOD==1)//[
					dwPixelVal = _epdfbdc_get_img_pixelvalue(pEPD_dc,pEPD_img,w,h,dwImgWidthBytes);
					#else //][!(GETIMG_PIXEL_METHOD==1)
					dwPixelVal = _epdfbdc_get_img_pixelvalue_from_ptr(pEPD_dcimg,&pbImgRow,dwSrcX);
					#endif //]
					
					if(dwEPD_dc_flags&EPDFB_DC_FLAG_SKIPLEFTPIXEL && 0==w) {
						// skip left pixel output.
					}
					else if(dwEPD_dc_flags&EPDFB_DC_FLAG_SKIPRIGHTPIXEL && I_dwDCimgW==w+1) {
						// skip right pixel output.
					}
					else {
						_epdfbdc_set_pixel(pEPD_dc,dwX,dwY,dwPixelVal,bImgPixelBits);
					}
				}
				//GALLEN_DBGLOCAL_PRINTMSG("\n,",w,h);
			}
		}		
		break;
	case EPDFB_R_90:GALLEN_DBGLOCAL_RUNLOG(3);
		// up->down,right->left .
		{
			unsigned long dwPixelVal;

			pEPD_dc->dwDirtyOffsetStart = I_dwDCimgX*dwDCWidthBytes;
			pEPD_dc->dwDirtyOffsetEnd = (I_dwDCimgX+I_dwDCimgW)*dwDCWidthBytes;

			
			//dwImgPixelIdx = 0;

			for(h=0;h<I_dwDCimgH;h+=1) {
				dwSrcY = I_dwDCimgY+h;
				dwX = dwDCW-h-1-I_dwDCPutY-dwWHidden;
				if(dwX>=dwDCW) {
					continue ;
				}
				
				pbImgRow = _epdfbdc_get_dcimg_ptr(pEPD_dcimg,I_dwDCimgX,dwSrcY);

				for(w=0;w<I_dwDCimgW;w+=1) {

					dwSrcX = I_dwDCimgX+w;
					dwY = I_dwDCPutX+w;
					if(dwY>=dwDCH) {
						continue ;
					}
					
					#if (GETIMG_PIXEL_METHOD==1) //[
					dwPixelVal = _epdfbdc_get_img_pixelvalue(pEPD_dc,pEPD_img,w,h,dwImgWidthBytes);
					#else //][
					dwPixelVal = _epdfbdc_get_img_pixelvalue_from_ptr(pEPD_dcimg,&pbImgRow,dwSrcX);
					#endif //]
				
					if(dwEPD_dc_flags&EPDFB_DC_FLAG_SKIPLEFTPIXEL && 0==w) {
						// skip left pixel output.
						//DBG_MSG("[[%d]]",w);
					}
					else if(dwEPD_dc_flags&EPDFB_DC_FLAG_SKIPRIGHTPIXEL && I_dwDCimgW==w+1) {
						// skip right pixel output.
						//DBG_MSG("{{%d}}",w);
					}
					else {
						_epdfbdc_set_pixel(pEPD_dc,dwX,dwY,dwPixelVal,bImgPixelBits);
					}
				}
			}// image height loop .
		}
		break;
	case EPDFB_R_180:GALLEN_DBGLOCAL_RUNLOG(4);
		// right->left,down->up .
		{
			volatile unsigned long dwPixelVal;
			
			pEPD_dc->dwDirtyOffsetStart = (dwDCH-I_dwDCimgY-I_dwDCimgH)*dwDCWidthBytes;
			pEPD_dc->dwDirtyOffsetEnd = (dwDCH-I_dwDCimgY)*dwDCWidthBytes;
			
			//dwImgPixelIdx = 0;
			for(h=0;h<I_dwDCimgH;h+=1) {

				dwSrcY = I_dwDCimgY+h;
				dwY = dwDCH-h-1-I_dwDCPutY-dwHHidden;
				if(dwY>=dwDCH) {
					continue ;
				}					

				pbImgRow = _epdfbdc_get_dcimg_ptr(pEPD_dcimg,I_dwDCimgX,dwSrcY);

				//DBG_MSG("y=%d->",I_dwDCimgH-h-1);
				for(w=0;w<I_dwDCimgW;w+=1) {
					dwSrcX = I_dwDCimgX+w;
					dwX = dwDCW-w-1-I_dwDCPutX-dwWHidden;
				//DBG_MSG("y=%d->",dwImgH-h-1);
					if(dwX>=dwDCW) {
						continue ;
					}
					
					#if (GETIMG_PIXEL_METHOD==1)//[
					dwPixelVal = _epdfbdc_get_img_pixelvalue(pEPD_dc,pEPD_img,w,h,dwImgWidthBytes);
					#else //][
					dwPixelVal = _epdfbdc_get_img_pixelvalue_from_ptr(pEPD_dcimg,&pbImgRow,dwSrcX);
					#endif //]
					if(dwEPD_dc_flags&EPDFB_DC_FLAG_SKIPLEFTPIXEL && 0==w) {
						// skip left pixel output.
					}
					else if(dwEPD_dc_flags&EPDFB_DC_FLAG_SKIPRIGHTPIXEL && I_dwDCimgW==w+1) {
						// skip right pixel output.
					}
					else {
						_epdfbdc_set_pixel(pEPD_dc,dwX,dwY,dwPixelVal,bImgPixelBits);
					}
				}
			}
		}		
		break;
	case EPDFB_R_270:GALLEN_DBGLOCAL_RUNLOG(5);
		// down->up,left->right .
		{
			unsigned long dwPixelVal;
			
			pEPD_dc->dwDirtyOffsetStart = (dwDCH-I_dwDCimgX-I_dwDCimgW)*dwDCWidthBytes;
			pEPD_dc->dwDirtyOffsetEnd = (dwDCH-I_dwDCimgX)*dwDCWidthBytes;
			
			//dwImgPixelIdx = 0;
			for(h=0;h<I_dwDCimgH;h+=1) {

				dwSrcY = I_dwDCimgY+h;
				dwX = h+I_dwDCPutY;
				if(dwX>=dwDCW) {
					continue ;
				}

				pbImgRow = _epdfbdc_get_dcimg_ptr(pEPD_dcimg,I_dwDCimgX,dwSrcY);

				for(w=0;w<I_dwDCimgW;w+=1) {

					dwSrcX = I_dwDCimgX+w;
					dwY = dwDCH-w-1-I_dwDCPutX-dwHHidden;
					if(dwY>=dwDCH) {
						continue ;
					}					
					
					#if (GETIMG_PIXEL_METHOD==1) //[
					dwPixelVal = _epdfbdc_get_img_pixelvalue(pEPD_dc,pEPD_img,w,h,dwImgWidthBytes);
					#else //][
					dwPixelVal = _epdfbdc_get_img_pixelvalue_from_ptr(pEPD_dcimg,&pbImgRow,dwSrcX);
					#endif //]

					if(dwEPD_dc_flags&EPDFB_DC_FLAG_SKIPLEFTPIXEL && 0==w) {
						// skip left pixel output.
					}
					else if(dwEPD_dc_flags&EPDFB_DC_FLAG_SKIPRIGHTPIXEL && I_dwDCimgW==w+1) {
						// skip right pixel output.
					}
					else {
						_epdfbdc_set_pixel(pEPD_dc,dwX,dwY,dwPixelVal,bImgPixelBits);
					}
				}
			}
		}
		break;
	}

//printk ("[%s-%d] %d\n",__func__,__LINE__,jiffies-tick);	
	
	GALLEN_DBGLOCAL_END();
	return tRet;
}

EPDFB_DC_RET epdfbdc_put_fbimg(EPDFB_DC *pEPD_dc,
	EPDFB_IMG *pEPD_img,EPDFB_ROTATE_T tRotateDegree)
{
	EPDFB_DC *ptEPD_dcimg ;
	EPDFB_DC_RET tRet;
	
	ptEPD_dcimg = epdfbdc_create_ex(pEPD_img->dwW,pEPD_img->dwH,\
		pEPD_img->bPixelBits,pEPD_img->pbImgBuf,pEPD_dc->dwFlags);
		
	tRet = epdfbdc_put_dcimg(pEPD_dc,ptEPD_dcimg,tRotateDegree,
		0,0,pEPD_img->dwW,pEPD_img->dwH,pEPD_img->dwX,pEPD_img->dwY);

	epdfbdc_delete(ptEPD_dcimg);
	
	return tRet;
}


EPDFB_DC_RET epdfbdc_get_rotate_active(EPDFB_DC *I_pEPD_dc,
	unsigned long *IO_pdwX,unsigned long *IO_pdwY,
	unsigned long *IO_pdwW,unsigned long *IO_pdwH,
	EPDFB_ROTATE_T I_tRotate)
{
	EPDFB_DC_RET tRet=EPDFB_DC_SUCCESS;
	unsigned long dwX,dwY,dwH,dwW;
	unsigned long dwDCW,dwDCH;
	unsigned dwWHidden,dwHHidden;
	
	if(!CHK_EPDFB_DC(I_pEPD_dc)) {
		ERR_MSG("%s(%d): object handle error !\n",__FUNCTION__,__LINE__);
		return EPDFB_DC_OBJECTERR;
	}
	
	if(!(IO_pdwX&&IO_pdwY&&IO_pdwW&&IO_pdwH)) {
		return EPDFB_DC_PARAMERR;
	}
	
	dwWHidden = I_pEPD_dc->dwFBWExtra;
	dwHHidden = I_pEPD_dc->dwFBHExtra;
	dwDCH=I_pEPD_dc->dwHeight + dwHHidden;
	dwDCW=I_pEPD_dc->dwWidth + dwWHidden;
	
	switch(I_tRotate)
	{
	default :
	case EPDFB_R_0:
		dwX = (*IO_pdwX);
		dwY = (*IO_pdwY);
		dwW = (*IO_pdwW);
		dwH = (*IO_pdwH);
		return tRet;
	case EPDFB_R_90:
		dwX = dwDCW-(*IO_pdwY)-(*IO_pdwH)-dwWHidden;
		dwY = (*IO_pdwX);
		dwW = (*IO_pdwH);
		dwH = (*IO_pdwW);
		break;
	case EPDFB_R_180:
		dwX = dwDCW-(*IO_pdwX)-(*IO_pdwW)-dwWHidden;
		dwY = dwDCH-(*IO_pdwY)-(*IO_pdwH)-dwHHidden;
		dwW = (*IO_pdwW);
		dwH = (*IO_pdwH);
		break;
	case EPDFB_R_270:
		dwX = (*IO_pdwY);
		dwY = dwDCH-(*IO_pdwW)-(*IO_pdwX)-dwHHidden;
		dwW = (*IO_pdwH);
		dwH = (*IO_pdwW);
		break;
	}
	*IO_pdwX = dwX;
	*IO_pdwY = dwY;
	*IO_pdwW = dwW;
	*IO_pdwH = dwH;
	return tRet;
}


EPDFB_DC_RET epdfbdc_set_pixel(EPDFB_DC *I_pEPD_dc,\
	unsigned long I_dwX,unsigned long I_dwY,unsigned long I_dwPVal)
{
	EPDFB_DC_RET tRet=EPDFB_DC_SUCCESS;
	
	if(!CHK_EPDFB_DC(I_pEPD_dc)) {
		ERR_MSG("%s(%d): object handle error !\n",__FUNCTION__,__LINE__);
		return EPDFB_DC_OBJECTERR;
	}
	
	_epdfbdc_set_pixel(I_pEPD_dc,I_dwX,I_dwY,I_dwPVal,4);
	
	return tRet;
}

EPDFB_DC_RET epdfbdc_dcbuf_to_RGB565(EPDFB_DC *I_pEPD_dc,\
	unsigned char *O_pbRGB565Buf,unsigned long I_dwRGB565BufSize)
{
	EPDFB_DC_RET tRet=EPDFB_DC_SUCCESS;
	
	if(!CHK_EPDFB_DC(I_pEPD_dc)) {
		return EPDFB_DC_OBJECTERR;
	}
	
	_fb_Gray4toRGB565(O_pbRGB565Buf,I_dwRGB565BufSize,\
		I_pEPD_dc->pbDCbuf,I_pEPD_dc->dwDCSize,I_pEPD_dc->dwFlags&EPDFB_DC_FLAG_REVERSEINPDATA);
	
	return tRet;
}

EPDFB_DC_RET epdfbdc_get_dirty_region(EPDFB_DC *I_pEPD_dc,\
	unsigned long *O_pdwDirtyOffsetStart,unsigned long *O_pdwDirtyOffsetEnd)
{
	EPDFB_DC_RET tRet=EPDFB_DC_SUCCESS;
	
	if(!CHK_EPDFB_DC(I_pEPD_dc)) {
		return EPDFB_DC_OBJECTERR;
	}
	
	if(O_pdwDirtyOffsetStart) {
		*O_pdwDirtyOffsetStart = I_pEPD_dc->dwDirtyOffsetStart ;
	}
	I_pEPD_dc->dwDirtyOffsetStart = 0;
	
	if(O_pdwDirtyOffsetEnd) {
		*O_pdwDirtyOffsetEnd = I_pEPD_dc->dwDirtyOffsetEnd ;
	}
	I_pEPD_dc->dwDirtyOffsetEnd = I_pEPD_dc->dwDCSize;
	
	return tRet;
}


EPDFB_DC_RET epdfbdc_rotate(EPDFB_DC *I_pEPD_dc,EPDFB_ROTATE_T I_tRotate)
{
	EPDFB_DC_RET tRet = EPDFB_DC_SUCCESS;
	unsigned char *pbWorkBuf;
	EPDFB_IMG tEPD_img;
	
	if(!CHK_EPDFB_DC(I_pEPD_dc)) {
		ERR_MSG("%s(%d): object handle error !\n",__FUNCTION__,__LINE__);
		return EPDFB_DC_OBJECTERR;
	}
	
	if(I_tRotate == EPDFB_R_0) {
		// nothing have to do .
		return EPDFB_DC_SUCCESS;
	}
	
	pbWorkBuf = my_malloc(I_pEPD_dc->dwDCSize);
	if(pbWorkBuf) {
		
		tEPD_img.dwX = 0;
		tEPD_img.dwY = 0;
		tEPD_img.dwW = I_pEPD_dc->dwWidth;
		tEPD_img.dwH = I_pEPD_dc->dwHeight;
		
		tEPD_img.bPixelBits = I_pEPD_dc->bPixelBits;
		tEPD_img.pbImgBuf = pbWorkBuf;
		
		my_memcpy(pbWorkBuf,I_pEPD_dc->pbDCbuf,I_pEPD_dc->dwDCSize);
		
		tRet = epdfbdc_put_fbimg(I_pEPD_dc,&tEPD_img,I_tRotate);
		my_free(pbWorkBuf);pbWorkBuf=0;
	}
	else {
		ERR_MSG("%s(%d): memory not enough !\n",__FUNCTION__,__LINE__);
		tRet = EPDFB_DC_MEMMALLOCFAIL;
	}
	
	return tRet;
}

EPDFB_DC_RET epdfbdc_set_host_dataswap(EPDFB_DC *I_pEPD_dc,int iIsSet)
{
	EPDFB_DC_RET tRet = EPDFB_DC_SUCCESS;

	if(!CHK_EPDFB_DC(I_pEPD_dc)) {
		ERR_MSG("%s(%d): object handle error !\n",__FUNCTION__,__LINE__);
		return EPDFB_DC_OBJECTERR;
	}
	
	if(iIsSet) {
		I_pEPD_dc->dwFlags |= EPDFB_DC_FLAG_REVERSEINPDATA;
	}
	else {
		I_pEPD_dc->dwFlags &= ~EPDFB_DC_FLAG_REVERSEINPDATA;
	}

	return tRet;
}

EPDFB_DC_RET epdfbdc_set_drive_dataswap(EPDFB_DC *I_pEPD_dc,int iIsSet)
{
	EPDFB_DC_RET tRet = EPDFB_DC_SUCCESS;

	if(!CHK_EPDFB_DC(I_pEPD_dc)) {
		ERR_MSG("%s(%d): object handle error !\n",__FUNCTION__,__LINE__);
		return EPDFB_DC_OBJECTERR;
	}
	
	if(iIsSet) {
		I_pEPD_dc->dwFlags |= EPDFB_DC_FLAG_SKIPLEFTPIXEL;
	}
	else {
		I_pEPD_dc->dwFlags &= ~EPDFB_DC_FLAG_SKIPLEFTPIXEL;
	}

	return tRet;
}

EPDFB_DC_RET epdfbdc_set_skip_pixel(EPDFB_DC *I_pEPD_dc,int iIsSet,int iIsSkipRight)
{
	EPDFB_DC_RET tRet = EPDFB_DC_SUCCESS;

	if(!CHK_EPDFB_DC(I_pEPD_dc)) {
		ERR_MSG("%s(%d): object handle error !\n",__FUNCTION__,__LINE__);
		return EPDFB_DC_OBJECTERR;
	}
	
	if(iIsSkipRight) {
		if(iIsSet) {
			I_pEPD_dc->dwFlags |= EPDFB_DC_FLAG_SKIPRIGHTPIXEL;
		}
		else {
			I_pEPD_dc->dwFlags &= ~EPDFB_DC_FLAG_SKIPRIGHTPIXEL;
		}
	}
	else {
		if(iIsSet) {
			I_pEPD_dc->dwFlags |= EPDFB_DC_FLAG_SKIPLEFTPIXEL;
		}
		else {
			I_pEPD_dc->dwFlags &= ~EPDFB_DC_FLAG_SKIPLEFTPIXEL;
		}
	}

	return tRet;
}

