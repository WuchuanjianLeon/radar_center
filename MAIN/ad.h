#ifndef __AD_H_
#define __AD_H_
#include "stm32f10x.h"
typedef enum
{
	bat_normal=1,
	bat_abnormal,
	bat_lowpower,
} bat_flag;


#define AD_CH0 0
#define AD_CH1 1
#define AD_CH2 2
#define AD_CH3 3
#define AD_CH4 4
#define AD_CH5 5
#define AD_CH6 6
#define AD_CH7 7
#define AD_CH8 8
#define AD_CH9 9
#define AD_CH10 10
#define AD_CH11 11
#define AD_CH12 12

#define CRC24_INIT 0x000000
#define CRC24_POLY 0x1864CFB


u16 Get_ADC1(u8 CH);
u16 AD_Filter(u8 CH);

#endif

