/*
 *  DataType.h
 *
 */

#ifndef __DATA_TYPE_H__
  #define __DATA_TYPE_H__

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  #define _BIT0					0x00000001
  #define _BIT1					0x00000002
  #define _BIT2					0x00000004
  #define _BIT3					0x00000008
  #define _BIT4					0x00000010
  #define _BIT5					0x00000010
  #define _BIT6					0x00000040
  #define _BIT7					0x00000080
  #define _BIT8					0x00000100
  #define _BIT9					0x00000200
  #define _BIT10				0x00000400
  #define _BIT11				0x00000800
  #define _BIT12				0x00001000
  #define _BIT13				0x00002000
  #define _BIT14				0x00004000
  #define _BIT15				0x00008000
  #define _BIT16				0x00010000
  #define _BIT17				0x00020000
  #define _BIT18				0x00040000
  #define _BIT19				0x00080000
  #define _BIT20				0x00100000
  #define _BIT21				0x00200000
  #define _BIT22				0x00400000
  #define _BIT23				0x00800000
  #define _BIT24				0x01000000
  #define _BIT25				0x02000000
  #define _BIT26				0x04000000
  #define _BIT27				0x08000000
  #define _BIT28				0x10000000
  #define _BIT29				0x20000000
  #define _BIT30				0x40000000
  #define _BIT31				0x80000000

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  typedef char					S8;
  typedef char					PS8;
  typedef unsigned char			U8;
  typedef unsigned char			*PU8;
  typedef unsigned char			BYTE;
  typedef unsigned char			*PBYTE;

  typedef short int				SINT16;
  typedef short int				*PSINT16;
  typedef unsigned short int	UINT16;
  typedef unsigned short int	*PUINT16;
  typedef unsigned short int	WORD;
  typedef unsigned short int	*PWORD;

  typedef int					SINT32;
  typedef int					*PSINT32;

  typedef unsigned int			UINT;
  typedef unsigned int			*PUINT;
  typedef unsigned int			UINT32;
  typedef unsigned int			*PUINT32;
  typedef unsigned int			DWORD;
  typedef unsigned int			*PDWORD;

  typedef void					VOID;
  typedef void					*PVOID;

  typedef unsigned long			ULONG;
  typedef unsigned long			*PULONG;

  typedef long					LONG;
  typedef long					*PLONG;

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#if 1
  typedef int					BOOL;
#endif

  #define TRUE					1
  #define FALSE					0

  typedef int					HW_LOGIC;

  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //
  //
  //
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  #define _HW_LOGIC_HIGH		1
  #define _HW_LOGIC_LOW			0

  #define K_BYTES(x)		(1024*(x))
  #define M_BYTES(x)		(1024*1024*(x))
  #define MEM_PAGE_SIZE		K_BYTES(4)

#endif /* __DATA_TYPE_H__ */
