#ifndef __INTERRUPT_H_
#define __INTERRUPT_H_
#include "stm32f10x.h"
#include "init.h"

#define USART_BUFF_LEN 512
#define RX_FIFO_NUM 400

#define RX_BUFF_LEN 14

#define FALSE 0
#define TRUE  1

#define RADAR_RAW_LENGTH 10


typedef struct
{
	int usart_len;		//接收长度 
	int rx_len;		//接收长度 
	unsigned char rx_buff[RX_BUFF_LEN];//接收数据缓存
	unsigned char usart_buff[USART_BUFF_LEN];//接收数据缓存
	u16 status;		//if true接收完标志

	u8 roll_cnt;
}usart_message;

extern usart_message USART1_MESSAGE;
extern usart_message USART2_MESSAGE;
extern usart_message USART3_MESSAGE;
extern usart_message UART5_MESSAGE;

extern usart_message USART1_RAW_MESSAGE;
extern usart_message USART2_RAW_MESSAGE;
extern usart_message USART3_RAW_MESSAGE;

extern usart_message Query_Message[2];
extern usart_message Query_Raw_Message[2];
extern u8 Can_Rx_Data[RX_FIFO_NUM][8];
extern u16 Can_In_Count;
extern u16 Can_Out_Count;
extern u8 Can_Rx_Flag ;
extern float Frequency[6];

//#if CAN_RX0_INT_ENABLE	//使能RX0中断
//	extern CanRxMsg RxMessage;
//#endif

void Query_uart(USART_TypeDef* USARTx,unsigned char usart_X);

#endif 
