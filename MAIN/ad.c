#include "ad.h"
 
/********************************************************************
***函数名称: Get_ADC1(u8 CH)
***函数说明:获取ADC1的AD采样值
***输入参数:通道
***输出参数:获取的值
***
********************************************************************/
u16 Get_ADC1(u8 CH)
{
	ADC_RegularChannelConfig(ADC1,CH, 1, ADC_SampleTime_13Cycles5);//设置指定的通道以及转换周期
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);//开启转换
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));//等待转换结束
	ADC_SoftwareStartConvCmd(ADC1,DISABLE);//关闭转换
	return ADC_GetConversionValue(ADC1);//返回获取的AD寄存器值
}
/********************************************************************
***函数名称: AD_max(u8 a[])
***函数说明:获取ADC1的AD采样值并进行中位值平均滤波
***输入参数:通道
***输出参数:获取的值
***
********************************************************************/
u16 AD_max(u16 a[2])
{
  u16 max_data=0;
  u8 i;
  for(i=0;i<2;i++)
  	{
  	  if(max_data<=a[i]) max_data=a[i];
  	}
  return (max_data);
}


/********************************************************************
***函数名称: AD_Filter(u8 CH)
***函数说明:获取ADC1的AD采样值并进行中位值平均滤波
***输入参数:通道
***输出参数:获取的值
***
********************************************************************/
u16 AD_Filter(u8 CH)
{
	static u16 AD_Buffer4[2]={0,0};
	static u16 AD_Buffer5[2]={0,0};
	static u16 AD_Buffer14[2]={0,0};
	static u16 AD_Buffer15[2]={0,0};
	u16 And=0;
	static u8 i4=0,i5=0,i14=0,i15=0;
	if(CH==4)
	{AD_Buffer4[i4]=Get_ADC1(CH);i4++;if(i4>=2)i4=0;And=AD_Buffer4[i4];}
	else if(CH==5)
	{AD_Buffer5[i5]=Get_ADC1(CH);i5++;if(i5>=2)i5=0;And=AD_Buffer5[i5];}
	else if(CH==14)
	{AD_Buffer14[i14]=Get_ADC1(CH);i14++;if(i14>=2)i14=0;And=AD_Buffer14[i14];}
	else if(CH==15)
	{AD_Buffer15[i15]=Get_ADC1(CH);i15++;if(i15>=2)i15=0;And=AD_Buffer15[i15];}
	return (And);
}


