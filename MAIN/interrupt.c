#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "interrupt.h"
#include "can.h"
#include "commander.h"
#include <math.h>

usart_message USART1_MESSAGE;
usart_message USART2_MESSAGE;
usart_message USART3_MESSAGE;
usart_message UART5_MESSAGE;

usart_message USART1_RAW_MESSAGE;
usart_message USART2_RAW_MESSAGE;
usart_message USART3_RAW_MESSAGE;


usart_message Query_Message[2];//asyrchronous uart


u8 Can_Rx_Data[RX_FIFO_NUM][8];
u16 Can_In_Count = 0;
u16 Can_Out_Count = 0;
u8 Can_Rx_Flag = 0;

u16 IC2Value[6];
float Frequency[6];
float Frequency_pre[6];

u8 IC2flag[6];

#define RADAR1_ID 0xBB
#define RADAR2_ID 0xCC
#define RADAR3_ID 0xDD

#define RADAR4_ID 0xDE
#define RADAR5_ID 0xDF


#define RADAR_SATR 0xAA
#define RADAR_ID_H 0x0C
#define RADAR_ID_L 0x07
#define RADAR_END  0x55
#define RANDR_LENTH 8

#define OA_DATA_START  0xFF
#define OA_DATA_STX    0x00
#define OA_DATA_PWM_ID 0x01
#define OA_DATA_END    0x55
#define OA_DATA_LENTH  2


/*void USART1_IRQHandler(void)
{
	u8 RES;
	static unsigned char statu=0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)//&& !USART1_MESSAGE.status)
	{
		RES =USART_ReceiveData(USART1);
		switch(statu)
		{
			case 0:
				if(RES==RADAR_SATR&&(!USART1_MESSAGE.status))
				{
					memset(USART1_MESSAGE.usart_buff,0,USART_BUFF_LEN);
					USART1_MESSAGE.usart_len=0;
					statu=1;
				}
				else
				{
					statu=0;
				}
				break;
			case 1:
				if(RES==RADAR_SATR)
				{
					statu=2;
				}
				else
				{
					statu=0;
				}
				break;
			case 2:
				if(RES==(RADAR_ID_H))
				{
					statu=3;
				}
				else
				{
					statu=0;
				}
			case 3:
				if(RES==(RADAR_ID_L))
				{
					statu=4;
				}
				else
				{
					statu=0;
				}
				break;
			case 4:
				USART1_MESSAGE.usart_buff[USART1_MESSAGE.usart_len]=RES;
				USART1_MESSAGE.usart_len++;
				if(USART1_MESSAGE.usart_len==RANDR_LENTH)
				{
					statu=5;
				}
				break;
			case 5:
				if(RES==(RADAR_END))
				{
					statu=6;
				}
				else
				{
					statu=0;
				}
				break;
			case 6:
				if(RES==(RADAR_END))
				{
					USART1_MESSAGE.status=TRUE;
					statu=0;
				}
				else
				{
					statu=0;
				}
				break;
			default:
				statu=0;
		}
	}
}*/

void USART1_IRQHandler(void)
{
	u8 RES,i;
	static unsigned char statu=0;
	static unsigned char usart1_i=0;
	//static u16 pre_usart_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET)//&& !USART1_MESSAGE.status)
	{
		RES =USART_ReceiveData(USART1);

		//if(USART1_RAW_MESSAGE.usart_len==USART_BUFF_LEN)
		//{
		//	USART1_RAW_MESSAGE.usart_len=0;
		//pre_usart_len=0;
		//}

		switch(statu)
		{
		case 0:
			if(RES==RADAR_SATR)
			{
				statu=1;
				USART1_RAW_MESSAGE.rx_len=0;
				USART1_RAW_MESSAGE.rx_buff[USART1_RAW_MESSAGE.rx_len] = RADAR3_ID;
				USART1_RAW_MESSAGE.rx_len++;
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
				usart1_i=0;
			}
			else
			{
				statu=0;
			}
			break;
		case 1:
			if(RES==RADAR_SATR)
			{
				statu=2;
				USART1_RAW_MESSAGE.rx_buff[USART1_RAW_MESSAGE.rx_len] = RADAR_SATR;
				USART1_RAW_MESSAGE.rx_len++;
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART1_RAW_MESSAGE.rx_len=0;
			}
			break;
		case 2:
			USART1_RAW_MESSAGE.rx_buff[USART1_RAW_MESSAGE.rx_len] = RES;
			USART1_RAW_MESSAGE.rx_len++;
			//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
			usart1_i++;
			if(usart1_i==RADAR_RAW_LENGTH)
			{
				statu=3;
			}
			break;
		case 3:
			if(RES==(RADAR_END))
			{
				statu=4;
				USART1_RAW_MESSAGE.rx_buff[USART1_RAW_MESSAGE.rx_len] = RADAR_END;
				USART1_RAW_MESSAGE.rx_len++;
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART1_RAW_MESSAGE.rx_len=0;
			}
			break;
		case 4:
			if(RES==(RADAR_END))
			{
				USART1_RAW_MESSAGE.rx_buff[USART1_RAW_MESSAGE.rx_len] = RADAR_END;
				USART1_RAW_MESSAGE.rx_len=0;
				for(i=0; i<RX_BUFF_LEN; i++)
				{
					USART1_RAW_MESSAGE.usart_buff[USART1_RAW_MESSAGE.usart_len] =
						USART1_RAW_MESSAGE.rx_buff[i];
					USART1_RAW_MESSAGE.usart_len++;
					if(USART1_RAW_MESSAGE.usart_len>=USART_BUFF_LEN)
					{
						USART1_RAW_MESSAGE.usart_len=0;
					}
				}
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
				USART1_RAW_MESSAGE.status++;
				statu=0;
			}
			else
			{
				statu=0;
				USART1_RAW_MESSAGE.rx_len=0;
			}
			break;
		default:
			statu=0;
		}
	}
}

void USART2_IRQHandler(void)
{
	u8 RES,i;
	static unsigned char statu=0;
	static unsigned char usart2_i=0;
	//static u16 pre_usart_len = 0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE)!=RESET)//&& !USART1_MESSAGE.status)
	{
		RES =USART_ReceiveData(USART2);

		//if(USART1_RAW_MESSAGE.usart_len==USART_BUFF_LEN)
		//{
		//	USART1_RAW_MESSAGE.usart_len=0;
		//pre_usart_len=0;
		//}

		switch(statu)
		{
		case 0:
			if(RES==RADAR_SATR)
			{
				statu=1;
				USART2_RAW_MESSAGE.rx_len=0;
				USART2_RAW_MESSAGE.rx_buff[USART2_RAW_MESSAGE.rx_len] = RADAR1_ID;
				USART2_RAW_MESSAGE.rx_len++;
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
				usart2_i=0;
			}
			else
			{
				statu=0;
			}
			break;
		case 1:
			if(RES==RADAR_SATR)
			{
				statu=2;
				USART2_RAW_MESSAGE.rx_buff[USART2_RAW_MESSAGE.rx_len] = RADAR_SATR;
				USART2_RAW_MESSAGE.rx_len++;
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART2_RAW_MESSAGE.rx_len=0;
			}
			break;
		case 2:
			USART2_RAW_MESSAGE.rx_buff[USART2_RAW_MESSAGE.rx_len] = RES;
			USART2_RAW_MESSAGE.rx_len++;
			//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
			usart2_i++;
			if(usart2_i==RADAR_RAW_LENGTH)
			{
				statu=3;
			}
			break;
		case 3:
			if(RES==(RADAR_END))
			{
				statu=4;
				USART2_RAW_MESSAGE.rx_buff[USART2_RAW_MESSAGE.rx_len] = RADAR_END;
				USART2_RAW_MESSAGE.rx_len++;
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART2_RAW_MESSAGE.rx_len=0;
			}
			break;
		case 4:
			if(RES==(RADAR_END))
			{
				USART2_RAW_MESSAGE.rx_buff[USART2_RAW_MESSAGE.rx_len] = RADAR_END;
				if(    USART2_RAW_MESSAGE.rx_buff[2] == 0x0A
							 && USART2_RAW_MESSAGE.rx_buff[3] == 0x06 )
				{
					USART2_RAW_MESSAGE.roll_cnt = ( USART2_RAW_MESSAGE.rx_buff[5] & 0x03 );
				}
				USART2_RAW_MESSAGE.rx_len=0;
				for(i=0; i<RX_BUFF_LEN; i++)
				{
					USART2_RAW_MESSAGE.usart_buff[USART2_RAW_MESSAGE.usart_len] =
						USART2_RAW_MESSAGE.rx_buff[i];
					USART2_RAW_MESSAGE.usart_len++;
					if(USART2_RAW_MESSAGE.usart_len>=USART_BUFF_LEN)
					{
						USART2_RAW_MESSAGE.usart_len=0;
					}
				}
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
				USART2_RAW_MESSAGE.status++;
				statu=0;
			}
			else
			{
				statu=0;
				USART2_RAW_MESSAGE.rx_len=0;
			}
			break;
		default:
			statu=0;
		}
	}
}
#if 0
void USART2_IRQHandler(void)
{
	u8 RES;
	static unsigned char statu=0;
	static unsigned char usart2_i=0;
	static u16 pre_usart_len = 0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE)!=RESET)//&& !USART2_MESSAGE.status)
	{
		RES =USART_ReceiveData(USART2);

		if(USART2_RAW_MESSAGE.usart_len==USART_BUFF_LEN)
		{
			USART2_RAW_MESSAGE.usart_len=0;
			pre_usart_len=0;
		}

		switch(statu)
		{
		case 0:
			if(RES==RADAR_SATR)
			{
				statu=1;
				USART2_RAW_MESSAGE.usart_buff[USART2_RAW_MESSAGE.usart_len] = RADAR1_ID;
				USART2_RAW_MESSAGE.usart_len++;
				pre_usart_len = USART2_RAW_MESSAGE.usart_len;
				usart2_i=0;
			}
			else
			{
				statu=0;
			}
			break;
		case 1:
			if(RES==RADAR_SATR)
			{
				statu=2;
				USART2_RAW_MESSAGE.usart_buff[USART2_RAW_MESSAGE.usart_len] = RADAR_SATR;
				USART2_RAW_MESSAGE.usart_len++;
				pre_usart_len = USART2_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART2_RAW_MESSAGE.usart_len=pre_usart_len;
			}
			break;
		case 2:
			USART2_RAW_MESSAGE.usart_buff[USART2_RAW_MESSAGE.usart_len] = RES;
			USART2_RAW_MESSAGE.usart_len++;
			pre_usart_len = USART2_RAW_MESSAGE.usart_len;
			usart2_i++;
			if(usart2_i==RADAR_RAW_LENGTH)
			{
				statu=3;
			}
			break;
		case 3:
			if(RES==(RADAR_END))
			{
				statu=4;
				USART2_RAW_MESSAGE.usart_buff[USART2_RAW_MESSAGE.usart_len] = RADAR_END;
				USART2_RAW_MESSAGE.usart_len++;
				pre_usart_len = USART2_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART2_RAW_MESSAGE.usart_len=pre_usart_len;
			}
			break;
		case 4:
			if(RES==(RADAR_END))
			{
				USART2_RAW_MESSAGE.usart_buff[USART2_RAW_MESSAGE.usart_len] = RADAR_END;
				USART2_RAW_MESSAGE.usart_len++;
				pre_usart_len = USART2_RAW_MESSAGE.usart_len;
				USART2_RAW_MESSAGE.status++;
				statu=0;
			}
			else
			{
				statu=0;
				USART2_RAW_MESSAGE.usart_len=pre_usart_len;
			}
			break;
		default:
			statu=0;
		}
	}
}
#endif
void USART3_IRQHandler(void)
{
	u8 RES,i;
	static unsigned char statu=0;
	static unsigned char usart3_i=0;
	//static u16 pre_usart_len = 0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE)!=RESET)//&& !USART1_MESSAGE.status)
	{
		RES =USART_ReceiveData(USART3);

		//if(USART1_RAW_MESSAGE.usart_len==USART_BUFF_LEN)
		//{
		//	USART1_RAW_MESSAGE.usart_len=0;
		//pre_usart_len=0;
		//}

		switch(statu)
		{
		case 0:
			if(RES==RADAR_SATR)
			{
				statu=1;
				USART3_RAW_MESSAGE.rx_len=0;
				USART3_RAW_MESSAGE.rx_buff[USART3_RAW_MESSAGE.rx_len] = RADAR2_ID;
				USART3_RAW_MESSAGE.rx_len++;
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
				usart3_i=0;
			}
			else
			{
				statu=0;
			}
			break;
		case 1:
			if(RES==RADAR_SATR)
			{
				statu=2;
				USART3_RAW_MESSAGE.rx_buff[USART3_RAW_MESSAGE.rx_len] = RADAR_SATR;
				USART3_RAW_MESSAGE.rx_len++;
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART3_RAW_MESSAGE.rx_len=0;
			}
			break;
		case 2:
			USART3_RAW_MESSAGE.rx_buff[USART3_RAW_MESSAGE.rx_len] = RES;
			USART3_RAW_MESSAGE.rx_len++;
			//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
			usart3_i++;
			if(usart3_i==RADAR_RAW_LENGTH)
			{
				statu=3;
			}
			break;
		case 3:
			if(RES==(RADAR_END))
			{
				statu=4;
				USART3_RAW_MESSAGE.rx_buff[USART3_RAW_MESSAGE.rx_len] = RADAR_END;
				USART3_RAW_MESSAGE.rx_len++;
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART3_RAW_MESSAGE.rx_len=0;
			}
			break;
		case 4:
			if(RES==(RADAR_END))
			{
				USART3_RAW_MESSAGE.rx_buff[USART3_RAW_MESSAGE.rx_len] = RADAR_END;
				if(    USART3_RAW_MESSAGE.rx_buff[2] == 0x0A
							 && USART3_RAW_MESSAGE.rx_buff[3] == 0x06 )
				{
					USART3_RAW_MESSAGE.roll_cnt = ( USART3_RAW_MESSAGE.rx_buff[5] & 0x03 );
				}
				USART3_RAW_MESSAGE.rx_len=0;
				for(i=0; i<RX_BUFF_LEN; i++)
				{
					USART3_RAW_MESSAGE.usart_buff[USART3_RAW_MESSAGE.usart_len] =
						USART3_RAW_MESSAGE.rx_buff[i];
					USART3_RAW_MESSAGE.usart_len++;
					if(USART3_RAW_MESSAGE.usart_len>=USART_BUFF_LEN)
					{
						USART3_RAW_MESSAGE.usart_len=0;
					}
				}
				//pre_usart_len = USART1_RAW_MESSAGE.usart_len;
				USART3_RAW_MESSAGE.status++;
				statu=0;
			}
			else
			{
				statu=0;
				USART3_RAW_MESSAGE.rx_len=0;
			}
			break;
		default:
			statu=0;
		}
	}
}

#if 0
void USART3_IRQHandler(void)
{
	u8 RES;
	static unsigned char statu=0;
	static unsigned char usart3_i=0;
	static u16 pre_usart_len = 0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE)!=RESET)//&& !USART3_MESSAGE.status)
	{
		RES =USART_ReceiveData(USART3);

		if(USART3_RAW_MESSAGE.usart_len==USART_BUFF_LEN)
		{
			USART3_RAW_MESSAGE.usart_len=0;
			pre_usart_len=0;
		}

		switch(statu)
		{
		case 0:
			if(RES==RADAR_SATR)
			{
				statu=1;
				USART3_RAW_MESSAGE.usart_buff[USART3_RAW_MESSAGE.usart_len] = RADAR2_ID;
				USART3_RAW_MESSAGE.usart_len++;
				pre_usart_len=USART3_RAW_MESSAGE.usart_len;
				usart3_i=0;
			}
			else
			{
				statu=0;
			}
			break;
		case 1:
			if(RES==RADAR_SATR)
			{
				statu=2;
				USART3_RAW_MESSAGE.usart_buff[USART3_RAW_MESSAGE.usart_len] = RADAR_SATR;
				USART3_RAW_MESSAGE.usart_len++;
				pre_usart_len=USART3_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART3_RAW_MESSAGE.usart_len=pre_usart_len;
			}
			break;
		case 2:
			USART3_RAW_MESSAGE.usart_buff[USART3_RAW_MESSAGE.usart_len] = RES;
			USART3_RAW_MESSAGE.usart_len++;
			pre_usart_len=USART3_RAW_MESSAGE.usart_len;
			usart3_i++;
			if(usart3_i==RADAR_RAW_LENGTH)
			{
				statu=3;
			}
			break;
		case 3:
			if(RES==(RADAR_END))
			{
				statu=4;
				USART3_RAW_MESSAGE.usart_buff[USART3_RAW_MESSAGE.usart_len] = RADAR_END;
				USART3_RAW_MESSAGE.usart_len++;
				pre_usart_len=USART3_RAW_MESSAGE.usart_len;
			}
			else
			{
				statu=0;
				USART3_RAW_MESSAGE.usart_len=pre_usart_len;
			}
			break;
		case 4:
			if(RES==(RADAR_END))
			{
				USART3_RAW_MESSAGE.usart_buff[USART3_RAW_MESSAGE.usart_len] = RADAR_END;
				USART3_RAW_MESSAGE.usart_len++;
				pre_usart_len=USART3_RAW_MESSAGE.usart_len;
				USART3_RAW_MESSAGE.status++;
				statu=0;
			}
			else
			{
				statu=0;
				USART3_RAW_MESSAGE.usart_len=pre_usart_len;
			}
			break;
		default:
			statu=0;
		}
	}
}
#endif

#if 1
void UART4_IRQHandler(void)
{
	u8 RES=0,i;
	u8 radar_id;
	static unsigned char statu[2] = {0,0};
	static unsigned char usartx_i[2] = {0,0};
	//static u16 pre_usart_len[2] ={0,0};
	u8 usart_X=0;
	if(usart_X==0) radar_id=RADAR4_ID;
	else radar_id=RADAR5_ID;
	if(USART_GetITStatus(UART4, USART_IT_RXNE)!=RESET)//&&!Query_Message[usart_X].status)
	{
		RES =USART_ReceiveData(UART4);
		//if(Query_Message[usart_X].usart_len>=USART_BUFF_LEN)
		//{
		//	Query_Message[usart_X].usart_len=0;
		//	pre_usart_len[usart_X]=0;
		//}

		switch(statu[usart_X])
		{
		case 0:
			if(RES==RADAR_SATR)
			{
				statu[usart_X]=1;
				Query_Message[usart_X].rx_len=0;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = radar_id;
				Query_Message[usart_X].rx_len++;
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
				usartx_i[usart_X]=0;

			}
			else
			{
				statu[usart_X]=0;
				//Query_Message[usart_X].rx_len=0;
			}
			break;
		case 1:
			if(RES==RADAR_SATR)
			{
				statu[usart_X]=2;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RADAR_SATR;
				Query_Message[usart_X].rx_len++;
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
			}
			else
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_len=0;
			}
			break;
		case 2:
			Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RES;
			Query_Message[usart_X].rx_len++;
			//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
			usartx_i[usart_X]++;
			if(usartx_i[usart_X]==RADAR_RAW_LENGTH)
			{
				statu[usart_X]=3;
			}
			break;
		case 3:
			if(RES==(RADAR_END))
			{
				statu[usart_X]=4;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RADAR_END;
				Query_Message[usart_X].rx_len++;
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
			}
			else
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_len=0;
				//Query_Message[usart_X].usart_len=pre_usart_len[usart_X];
			}

			break;
		case 4:
			if(RES==(RADAR_END))
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RADAR_END;
				Query_Message[usart_X].rx_len=0;
				for(i=0; i<RX_BUFF_LEN; i++)
				{
					Query_Message[usart_X].usart_buff[Query_Message[usart_X].usart_len] =
						Query_Message[usart_X].rx_buff[i];
					Query_Message[usart_X].usart_len++;
					if(Query_Message[usart_X].usart_len>=USART_BUFF_LEN)
					{
						Query_Message[usart_X].usart_len=0;
						//pre_usart_len[usart_X]=0;
					}
				}
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
				Query_Message[usart_X].status++;
			}
			else
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_len=0;
			}

			break;
		default:
			statu[usart_X]=0;
		}
	}
}
#endif


#if 0
void UART5_IRQHandler(void)
{
	u8 RES=0,i;
	u8 radar_id;
	static unsigned char statu[2] = {0,0};
	static unsigned char usartx_i[2] = {0,0};
	//static u16 pre_usart_len[2] ={0,0};
	u8 usart_X=1;
	if(usart_X==0) radar_id=RADAR4_ID;
	else radar_id=RADAR5_ID;
	if(USART_GetITStatus(UART5, USART_IT_RXNE)!=RESET)//&&!Query_Message[usart_X].status)
	{
		RES =USART_ReceiveData(UART5);
		//if(Query_Message[usart_X].usart_len>=USART_BUFF_LEN)
		//{
		//	Query_Message[usart_X].usart_len=0;
		//	pre_usart_len[usart_X]=0;
		//}

		switch(statu[usart_X])
		{
		case 0:
			if(RES==RADAR_SATR)
			{
				statu[usart_X]=1;
				Query_Message[usart_X].rx_len=0;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = radar_id;
				Query_Message[usart_X].rx_len++;
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
				usartx_i[usart_X]=0;

			}
			else
			{
				statu[usart_X]=0;
				//Query_Message[usart_X].rx_len=0;
			}
			break;
		case 1:
			if(RES==RADAR_SATR)
			{
				statu[usart_X]=2;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RADAR_SATR;
				Query_Message[usart_X].rx_len++;
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
			}
			else
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_len=0;
			}
			break;
		case 2:
			Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RES;
			Query_Message[usart_X].rx_len++;
			//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
			usartx_i[usart_X]++;
			if(usartx_i[usart_X]==RADAR_RAW_LENGTH)
			{
				statu[usart_X]=3;
			}
			break;
		case 3:
			if(RES==(RADAR_END))
			{
				statu[usart_X]=4;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RADAR_END;
				Query_Message[usart_X].rx_len++;
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
			}
			else
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_len=0;
				//Query_Message[usart_X].usart_len=pre_usart_len[usart_X];
			}

			break;
		case 4:
			if(RES==(RADAR_END))
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RADAR_END;
				Query_Message[usart_X].rx_len=0;
				for(i=0; i<RX_BUFF_LEN; i++)
				{
					Query_Message[usart_X].usart_buff[Query_Message[usart_X].usart_len] =
						Query_Message[usart_X].rx_buff[i];
					Query_Message[usart_X].usart_len++;
					if(Query_Message[usart_X].usart_len>=USART_BUFF_LEN)
					{
						Query_Message[usart_X].usart_len=0;
						//pre_usart_len[usart_X]=0;
					}
				}
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
				Query_Message[usart_X].status++;
			}
			else
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_len=0;
			}

			break;
		default:
			statu[usart_X]=0;
		}
	}
}
#endif

void UART5_IRQHandler(void)
{
	u8 RES;
	static unsigned char statu=0;

	if(USART_GetITStatus(UART5, USART_IT_RXNE)!=RESET)//&& !UART5_MESSAGE.status)
	{
		RES = USART_ReceiveData(UART5);

		switch(statu)
		{
		case 0:
			if(RES==OA_DATA_START&&(!UART5_MESSAGE.status))
			{
				memset(UART5_MESSAGE.usart_buff,0,USART_BUFF_LEN);
				UART5_MESSAGE.usart_len=0;
				statu=1;
			}
			else
			{
				statu=0;
			}
			break;
		case 1:
			if(RES==OA_DATA_STX)
			{
				statu=2;
			}
			else
			{
				statu=0;
			}
			break;
		case 2:
			if(RES==(OA_DATA_PWM_ID))
			{
				statu=3;
			}
			else
			{
				statu=0;
			}
			break;
		case 3:
			UART5_MESSAGE.usart_buff[UART5_MESSAGE.usart_len]=RES;
			UART5_MESSAGE.usart_len++;
			if(UART5_MESSAGE.usart_len==OA_DATA_LENTH)
			{
				statu=4;
			}
			break;
		case 4:
			if(RES==(OA_DATA_END))
			{
				statu=5;
			}
			else
			{
				statu=0;
			}
			break;
		case 5:
			if(RES==(OA_DATA_END))
			{
				UART5_MESSAGE.status=TRUE;
				statu=0;
			}
			else
			{
				statu=0;
			}
			break;
		default:
			statu=0;
		}
	}
}


#if 0
void Query_uart(USART_TypeDef* USARTx,unsigned char usart_X)
{
	u8 RES=0,i;
	u8 radar_id;
	static unsigned char statu[2] = {0,0};
	static unsigned char usartx_i[2] = {0,0};
	//static u16 pre_usart_len[2] ={0,0};

	if(usart_X==0) radar_id=RADAR4_ID;
	else radar_id=RADAR5_ID;
	if(USART_GetITStatus(USARTx, USART_IT_RXNE)!=RESET)//&&!Query_Message[usart_X].status)
	{
		RES =USART_ReceiveData(USARTx);
		//if(Query_Message[usart_X].usart_len>=USART_BUFF_LEN)
		//{
		//	Query_Message[usart_X].usart_len=0;
		//	pre_usart_len[usart_X]=0;
		//}

		switch(statu[usart_X])
		{
		case 0:
			if(RES==RADAR_SATR)
			{
				statu[usart_X]=1;
				Query_Message[usart_X].rx_len=0;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = radar_id;
				Query_Message[usart_X].rx_len++;
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
				usartx_i[usart_X]=0;

			}
			else
			{
				statu[usart_X]=0;
				//Query_Message[usart_X].rx_len=0;
			}
			break;
		case 1:
			if(RES==RADAR_SATR)
			{
				statu[usart_X]=2;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RADAR_SATR;
				Query_Message[usart_X].rx_len++;
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
			}
			else
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_len=0;
			}
			break;
		case 2:
			Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RES;
			Query_Message[usart_X].rx_len++;
			//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
			usartx_i[usart_X]++;
			if(usartx_i[usart_X]==RADAR_RAW_LENGTH)
			{
				statu[usart_X]=3;
			}
			break;
		case 3:
			if(RES==(RADAR_END))
			{
				statu[usart_X]=4;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RADAR_END;
				Query_Message[usart_X].rx_len++;
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
			}
			else
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_len=0;
				//Query_Message[usart_X].usart_len=pre_usart_len[usart_X];
			}

			break;
		case 4:
			if(RES==(RADAR_END))
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_buff[Query_Message[usart_X].rx_len] = RADAR_END;
				Query_Message[usart_X].rx_len=0;
				for(i=0; i<RX_BUFF_LEN; i++)
				{
					Query_Message[usart_X].usart_buff[Query_Message[usart_X].usart_len] =
						Query_Message[usart_X].rx_buff[i];
					Query_Message[usart_X].usart_len++;
					if(Query_Message[usart_X].usart_len>=USART_BUFF_LEN)
					{
						Query_Message[usart_X].usart_len=0;
						//pre_usart_len[usart_X]=0;
					}
				}
				//pre_usart_len[usart_X]=Query_Message[usart_X].usart_len;
				Query_Message[usart_X].status++;
			}
			else
			{
				statu[usart_X]=0;
				Query_Message[usart_X].rx_len=0;
			}

			break;
		default:
			statu[usart_X]=0;
		}
	}
}
#endif


//#if CAN_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	u8 i=0;
	CanRxMsg RxMessage;
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0); //清除FIFO0消息挂号中断标志位
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);//将FIFO0中接收数据信息存入消息结构体中

	//	for(i=0;i<8;i++)
	// {
	//    	  USART_SendData(UART4,RxMessage.Data[i]);
	// }
	if ((RxMessage.IDE == CAN_ID_STD)&&(RxMessage.StdId==AIRC_CAN_ID)) //如果消息标识符的类型为标准帧模式
	{
		// use_art=USE_USART4;
		//  printf("%f\n",((float)RxMessage.StdId));/*输出浮点数*/
		for(i=0; i<8; i++)
		{
			Can_Rx_Data[Can_In_Count][i]=RxMessage.Data[i];
		}
		Can_In_Count++;
		if(Can_In_Count >= RX_FIFO_NUM)
		{
			Can_In_Count = 0;
		}
		CAN_FIFORelease(CAN1,CAN_FIFO0);//释放FIFO0
	}
	CAN_FIFORelease(CAN1,CAN_FIFO0);

}

void TIM2_IRQHandler(void)
{
	u16 test;
	int32_t fre=0;
	if(TIM_GetITStatus(TIM2,TIM_IT_Update))
	{

		IC2flag[4]++;
		IC2flag[5]++;
	}

	if(IC2flag[4]>25)
	{
		Frequency[4]=0;
		IC2flag[4]=0;
		IC2Value[4]=0;
	}
	if(IC2flag[5]>25)
	{
		Frequency[5]=0;
		IC2flag[5]=0;
		IC2Value[5]=0;
	}

	if(TIM_GetITStatus(TIM2,TIM_IT_CC1)&& GPIO_PA0==1)
	{

		//IC2Value[2]= TIM_GetCapture3(TIM3);                         //读取IC2捕获寄存器的值，即为PWM周期的计数值
		test=TIM_GetCapture1(TIM2);
		if (IC2Value[4] != 0&& GPIO_PA0==1)
		{
			fre=test+65536*IC2flag[4]-IC2Value[4];
			if(fre>=600&&IC2flag[4]==0)
				Frequency[4]= (72000000.0/TIME2_PWM_IDIV) /(1.0f*fre)  ;

			if(fabs(Frequency_pre[4]-2*Frequency[4])<=30)
			{
				Frequency[4]=2*Frequency[4];
			}
			Frequency_pre[4]=Frequency[4];
			// Frequency[4]= (72000000.0/TIME2_PWM_IDIV) / (test+65536*IC2flag[4]-IC2Value[4]);                                      //计算PWM频率。
		}
		else
		{
			Frequency[4]= 0.0;
		}
		IC2Value[4]= test;
		IC2flag[4]=0;
	}
	if(TIM_GetITStatus(TIM2,TIM_IT_CC2)&& GPIO_PA1==1)
	{

		//IC2Value[3]= TIM_GetCapture4(TIM3);                         //读取IC2捕获寄存器的值，即为PWM周期的计数值
		test=TIM_GetCapture2(TIM2);
		if (IC2Value[5] != 0&& GPIO_PA1==1)
		{
			fre=test+65536*IC2flag[5]-IC2Value[5];
			if(fre>=600&&IC2flag[5]==0)
				Frequency[5]= (72000000.0/TIME2_PWM_IDIV) /(1.0f*fre)  ;

			if(fabs(Frequency_pre[5]-2*Frequency[5])<=30)
			{
				Frequency[5]=2*Frequency[5];
			}
			Frequency_pre[5]=Frequency[5];
			//Frequency[5]= (72000000.0/TIME2_PWM_IDIV) / (test+65536*IC2flag[5]-IC2Value[5]);                                 //计算PWM频率。
		}
		else
		{
			Frequency[5]= 0.0;
		}
		IC2Value[5]= test;
		IC2flag[5]=0;
	}

//TIM_ClearITPendingBit(TIM3,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);			 //清楚TIM的中断待处理位
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2);			 //清楚TIM的中断待处理位

}

void TIM4_IRQHandler(void)
{
	u16 test;
	int32_t fre=0;
	if(TIM_GetITStatus(TIM4,TIM_IT_Update))
	{

		IC2flag[0]++;
		IC2flag[1]++;
		IC2flag[2]++;
		IC2flag[3]++;
	}

	if(IC2flag[0]>25)
	{
		Frequency[0]=0;
		IC2flag[0]=0;
		IC2Value[0]=0;
	}
	if(IC2flag[1]>25)
	{
		Frequency[1]=0;
		IC2flag[1]=0;
		IC2Value[1]=0;
	}
	if(IC2flag[2]>25)
	{
		Frequency[2]=0;
		IC2flag[2]=0;
		IC2Value[2]=0;
	}
	if(IC2flag[3]>25)
	{
		Frequency[3]=0;
		IC2flag[3]=0;
		IC2Value[3]=0;
	}

	if(TIM_GetITStatus(TIM4,TIM_IT_CC1) && GPIO_PB6==1)
	{

		// IC2Value[0]= TIM_GetCapture1(TIM3);                         //读取IC2捕获寄存器的值，即为PWM周期的计数值
		test=TIM_GetCapture1(TIM4);
		if (IC2Value[0] != 0&& GPIO_PB6==1)
		{
			fre=test+65536*IC2flag[0]-IC2Value[0];
			if(fre>=600&&IC2flag[0]==0)
				Frequency[0]= (72000000.0/TIME4_PWM_IDIV) /(1.0f*fre)  ;

			if(fabs(Frequency_pre[0]-2*Frequency[0])<=30)
			{
				Frequency[0]=2*Frequency[0];
			}
			Frequency_pre[0]=Frequency[0];
		}
		else
		{
			Frequency[0]= 0.0;
		}
		IC2Value[0]= test;
		IC2flag[0]=0;
	}
	if(TIM_GetITStatus(TIM4,TIM_IT_CC2)&& GPIO_PB7==1)
	{

		// IC2Value[1]= TIM_GetCapture2(TIM3);                         //读取IC2捕获寄存器的值，即为PWM周期的计数值
		//printf("IC2Value=%d\n",IC2Value[1]);
		test=TIM_GetCapture2(TIM4);
		if (IC2Value[1] != 0&& GPIO_PB7==1)
		{
			fre=test+65536*IC2flag[1]-IC2Value[1];
			if(fre>=600&&IC2flag[1]==0)
				Frequency[1]= (72000000.0/TIME4_PWM_IDIV) /(1.0f*fre)  ;  //计算PWM频率。

			if(fabs(Frequency_pre[1]-2*Frequency[1])<=30)
			{
				Frequency[1]=2*Frequency[1];
			}
			Frequency_pre[1]=Frequency[1];
		}
		else
		{
			Frequency[1]= 0.0;
		}
		IC2Value[1]= test;
		IC2flag[1]=0;
	}
	if(TIM_GetITStatus(TIM4,TIM_IT_CC3)&& GPIO_PB8==1)
	{

		//IC2Value[2]= TIM_GetCapture3(TIM3);                         //读取IC2捕获寄存器的值，即为PWM周期的计数值
		test=TIM_GetCapture3(TIM4);
		if (IC2Value[2] != 0&& GPIO_PB8==1)
		{
			fre=test+65536*IC2flag[2]-IC2Value[2];
			if(fre>=600&&IC2flag[2]==0)
				Frequency[2]= (72000000.0/TIME4_PWM_IDIV) /(1.0f*fre)  ;  //计算PWM频率。

			if(fabs(Frequency_pre[2]-2*Frequency[2])<=30)
			{
				Frequency[2]=2*Frequency[2];
			}
			Frequency_pre[2]=Frequency[2];
		}
		else
		{
			Frequency[2]= 0.0;
		}
		IC2Value[2]= test;
		IC2flag[2]=0;
	}
	if(TIM_GetITStatus(TIM4,TIM_IT_CC4)&& GPIO_PB9==1)
	{

		//IC2Value[3]= TIM_GetCapture4(TIM3);                         //读取IC2捕获寄存器的值，即为PWM周期的计数值
		test=TIM_GetCapture4(TIM4);
		if (IC2Value[3] != 0&& GPIO_PB9==1)
		{
			fre=test+65536*IC2flag[3]-IC2Value[3];
			if(fre>=600&&IC2flag[3]==0)
				Frequency[3]= (72000000.0/TIME4_PWM_IDIV) /(1.0f*fre) ; //计算PWM频率。

			if(fabs(Frequency_pre[3]-2*Frequency[3])<=30)
			{
				Frequency[3]=2*Frequency[3];
			}
			Frequency_pre[3]=Frequency[3];
		}
		else
		{
			Frequency[3]= 0.0;
		}
		IC2Value[3]= test;
		IC2flag[3]=0;
	}

//TIM_ClearITPendingBit(TIM3,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);			 //清楚TIM的中断待处理位
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);			 //清楚TIM的中断待处理位

}



