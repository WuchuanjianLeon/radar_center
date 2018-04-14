#ifndef __STM32_COMBIT_H__
#define __STM32_COMBIT_H__

#include "stm32f10x.h"


typedef union
{
	struct
	{
		unsigned bit0    :1;
		unsigned bit1    :1;
		unsigned bit2    :1;
		unsigned bit3    :1;
		unsigned bit4    :1;
		unsigned bit5    :1;
		unsigned bit6    :1;
		unsigned bit7    :1;
	} ONEBITS;
	unsigned char BYTE;
} Stm32Byte;

typedef union
{
	short int value;  //注意:操作系统不一致 会导致value不同   所以需要用sizeof测试一下
	unsigned char bytes[2];
} int16andUint8_t;

typedef union
{
	struct
	{
		char Byte0;
		char Byte1;
		char Byte2;
		char Byte3;
	} Byte4;
	long Stm32Long;
} Stm32Long;



#endif
