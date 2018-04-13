#ifndef __INIT_H_
#define __INIT_H_

#include "stm32f10x.h"
#include "stm32_combit.h"
#include "delay.h"

/*****************AD*****************/
#define AD1_PERPH

/*****************iic*****************/
//#define IIC1_PERPH
//#define IIC_STAR I2C_GenerateSTART(I2C1,ENABLE)//IIC开始信号
//#define IIC_STOP I2C_GenerateSTOP(I2C1,ENABLE)//IIC停止信号
//#define IIC_BUSY I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)//IIC遇忙检测信号
//#define IIC_NAK I2C_GetFlagStatus(I2C1,I2C_FLAG_AF)//应答错误标志
//#define IIC_READ_REG I2C_ReadRegister(I2C1,I2C_Register_CR1)//读寄存器
//#define IIC_SEND_ADD(X) I2C_Send7bitAddress(I2C1,X,I2C_Direction_Transmitter)//写地址数据
//#define IIC_READ I2C_ReceiveData(I2C1)//读数据
//#define IIC_WRIT_DAT(X) I2C_SendData(I2C1,X)//写数据

/*****************SPI*****************/
//#define SPI1_PERPH
#define SPI2_PERPH

/*****************串口1*****************/
#define USART1_PERPH
#define BAND1  (u32)115200
/*****************串口2*****************/
#define USART2_PERPH
#define BAND2  (u32)115200

/*****************串口3*****************/
#define USART3_PERPH
#define BAND3  (u32)115200
/*****************串口4*****************/
#define USART4_PERPH
#define BAND4  (u32)115200
/*****************串口4*****************/
#define USART5_PERPH
#define BAND5  (u32)115200
/*****************CAN1*****************/

//#define CAN_RX0_INT_ENABLE

/***************定时器*****************/
//#define TIME3_PERPH
//#define TIME3_DIV  (7200-1)
//#define TIME3_PRE 100		//周期    TIME_DIV/72000000*TIME_PRE
/***************PWM模式*****************/
//#define TIME4_PWM
#define TIME4_DIV  (72-1)
#define TIME4_PRE 2041		//周期    TIME_DIV/72000000*TIME_PRE
/***************PWM模式*****************/
#define TIME3_PWM
#define TIME3_DIV  (72-1)
#define TIME3_PRE (20000-1)//(40040)		//周期    TIME_DIV/72000000*TIME_PRE

#define pwm_en_set GPIO_SetBits(GPIOA,GPIO_Pin_1)

#define pwm_en_reset GPIO_ResetBits(GPIOA,GPIO_Pin_1)

#define pwr_sw1_set GPIO_SetBits(GPIOC,GPIO_Pin_6)

#define pwr_sw1_reset GPIO_ResetBits(GPIOC,GPIO_Pin_6)

#define pwr_sw2_set GPIO_SetBits(GPIOC,GPIO_Pin_7)

#define pwr_sw2_reset GPIO_ResetBits(GPIOC,GPIO_Pin_7)

#define pwr_sw3_set GPIO_SetBits(GPIOC,GPIO_Pin_8)

#define pwr_sw3_reset GPIO_ResetBits(GPIOC,GPIO_Pin_8)

#define pwr_sw4_set GPIO_SetBits(GPIOC,GPIO_Pin_9)

#define pwr_sw4_reset GPIO_ResetBits(GPIOC,GPIO_Pin_9)

#ifdef SPI1_PERPH
#define MPU_9250_ENABLE GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define MPU_9250_DISENABLE GPIO_SetBits(GPIOA,GPIO_Pin_4)
#endif

#ifdef SPI2_PERPH
#define MPU_9250_ENABLE GPIO_ResetBits(GPIOB,GPIO_Pin_12)
#define MPU_9250_DISENABLE GPIO_SetBits(GPIOB,GPIO_Pin_12)
#endif

#define GPIO_PA0 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)	//
#define GPIO_PA1 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)	//
#define GPIO_PB6 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)	//
#define GPIO_PB7 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)	//
#define GPIO_PB8 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)	//
#define GPIO_PB9 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)	//





#define TIME4_PWM_IDIV  48
#define TIME2_PWM_IDIV  48

//extern u16 pwm4;

void Init_All(void);
void IWDG_Feed(void);

#endif

