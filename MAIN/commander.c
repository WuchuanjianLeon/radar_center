#include "commander.h"
#include "interrupt.h"
#include "flash_rw.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "can.h"
#include "mpu_data.h"
#include <math.h>
#include "Acc_Calibration.h"
#include "imu_srckf.h"

#define ST_THRE_MAX 2400 //最大舵机控制角度PWM
#define ST_THRE_MIN 600  //最小舵机控制角度PWM
#define MSG_RE_LEN 8     //CAN接收数据单包长度
#define ST_CURR_MAX 2850 //0.7A舵机电流保护阈值
#define STE_FLAG_MAX 1000 //正常运行时舵机电流连续大于阈值的计数阈值
#define STE_FLAG_MAXD 100 //正常运行时舵机电流连续小于阈值的计数阈值
#define STE_NORMAL 11 //表示舵机运行正常

#define STE_D1 390 //舵机校准差值
#define STE_D2 390 //舵机校准差值
#define STE_D3 390 //舵机校准差值
#define STE_D4 390 //舵机校准差值

#define STE_TS 40 //舵机校准时电流连续大于阈值的计数阈值



unsigned char use_art=0;//串口发送端口选择变量
//unsigned char flagpwm=0;//
u16 duty_cycle1=1500,duty_cycle2=1500,duty_cycle3=1500,duty_cycle4=1500;//舵机控制PWM变量
int16_t dt_can1=0,dt_can2=0,dt_can3=0,dt_can4=0;//CAN接收舵机控制PWM差值变量
u16 duty_cymed1=1500,duty_cymed2=1500,duty_cymed3=1500,duty_cymed4=1500;//舵机校准后的中位控制PWM变量
u16 duty_cycle_flash[4]= {0,0,0,0}; //舵机校准后待保存的PWM变量
u8  duty_ste_flag[4]= {0,0,0,0}; //舵机硬件连接是否正常标准变量

uint32_t duty_flash_write[4]= {0,0,0,0}; //舵机PWM保存到FLASH的储存变量
unsigned flash_write_flag[4]= {FALSE,FALSE,FALSE,FALSE}; //是否需要重新写入FLASH标志位

u16 ste_flag1=0,ste_flag2=0,ste_flag3=0,ste_flag4=0;//舵机电流大于阈值的计数变量
u8 ste_status1=STE_NORMAL,ste_status2=STE_NORMAL,ste_status3=STE_NORMAL,ste_status4=STE_NORMAL;//舵机运行状态变量
u8 steering_current_counter[4]= {0,0,0,0}; //舵机电流小于阈值的计数变量
float steering_current_data_f[4]= {0,0,0,0}; //舵机电流检测数据浮点型(单位A)
u16 steering_current_data[4]= {0,0,0,0}; //舵机电流检测数据

u8 data_tx_buf[300];//CAN发送数据缓冲区
u8 data_tx_len=0;//CAN发送数据长度
u8 sw[4]= {0,0,0,0}; //是否进行舵机校准标准变量
static int8_t duty_step1=6,duty_step2=6,duty_step3=6,duty_step4=6;//重复校准延时计数
static u8 flagtest1=0,flagtest2=0,flagtest3=0,flagtest4=0,goodsum=0,badsum=0;//程序调试使用

unsigned chflag1=FALSE,chflag2=FALSE,chflag3=FALSE,chflag4=FALSE;//舵机电流检测有效标准变量
static u16 ste_flagt[4]= {0,0,0,0}; //舵机校准时电流连续大于阈值的计数变量


float prm[6];
u8 sum=0;


CAN_SEND_DATA can_send_oa_data;
CAN_SEND_DATA can_send_tf_data;

CAN_REC_DATA can_rec_oa_data;
CAN_REC_DATA can_rec_tf_data;

//CanRxMsg RxMessage;

RADAR_OA_DATA radar1_oa_data;
RADAR_OA_DATA radar2_oa_data;
RADAR_TF_DATA radar1_tf_data;
RADAR_TF_DATA radar2_tf_data;

OA_DATA oa_data;

#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
	int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
int _sys_exit(int x)
{
	x = x;
	return x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
	if(use_art==USE_USART1)
	{
		while((USART1->SR&0X40)==0) {} //循环发送,直到发送完毕
		USART1->DR = (u8) ch;
		return ch;
	}
	else if(use_art==USE_USART2)
	{
		while((USART2->SR&0X40)==0) {} //循环发送,直到发送完毕
		USART2->DR = (u8) ch;
		return ch;
	}
	else if(use_art==USE_USART3)
	{
		while((USART3->SR&0X40)==0) {} //循环发送,直到发送完毕
		USART3->DR = (u8) ch;
		return ch;
	}
	else if(use_art==USE_USART4)
	{
		while((UART4->SR&0X40)==0) {} //循环发送,直到发送完毕
		UART4->DR = (u8) ch;
		return ch;
	}
	else if(use_art==USE_USART5)
	{
		while((UART5->SR&0X40)==0) {} //循环发送,直到发送完毕
		UART5->DR = (u8) ch;
		return ch;
	}
	else
	{
		return 0;
	}
}
#endif
void Current_AD_Read(void)//舵机正常运行时的电流ADC采样函数
{
	if(!sw[0])
	{
		steering_current_data[0]=AD_Filter(4);
		steering_current_data_f[0]=steering_current_data[0]/4096.0;
	}
	if(!sw[1])
	{
		steering_current_data[1]=AD_Filter(5);
		steering_current_data_f[1]=steering_current_data[1]/4096.0;
	}
	if(!sw[2])
	{
		steering_current_data[2]=AD_Filter(14);
		steering_current_data_f[2]=steering_current_data[2]/4096.0;
	}
	if(!sw[3])
	{
		steering_current_data[3]=AD_Filter(15);
		steering_current_data_f[3]=steering_current_data[3]/4096.0;
	}

}

void Current_AD_Read1(void)//舵机校准时电流ADC采样函数
{
	if(sw[0])
	{
		steering_current_data[0]=AD_Filter(4);
		steering_current_data_f[0]=steering_current_data[0]/4096.0;
	}
	if(sw[1])
	{
		steering_current_data[1]=AD_Filter(5);
		steering_current_data_f[1]=steering_current_data[1]/4096.0;
	}
	if(sw[2])
	{
		steering_current_data[2]=AD_Filter(14);
		steering_current_data_f[2]=steering_current_data[2]/4096.0;
	}
	if(sw[3])
	{
		steering_current_data[3]=AD_Filter(15);
		steering_current_data_f[3]=steering_current_data[3]/4096.0;
	}

}

void STE_Check1(void)//舵机校准时电流检测判断函数
{
	if(sw[0]&&chflag1)
	{
		if(steering_current_data[0]>ST_CURR_MAX)
			ste_flagt[0]++;
		if(ste_flagt[0]>STE_TS)
		{
			ste_status1=STE_ABNORMAL;
		}
	}
	if(sw[1]&&chflag2)
	{
		if(steering_current_data[1]>ST_CURR_MAX)
			ste_flagt[1]++;
		if(ste_flagt[1]>STE_TS)
		{
			ste_status2=STE_ABNORMAL;
		}
	}
	if(sw[2]&&chflag3)
	{
		if(steering_current_data[2]>ST_CURR_MAX)
			ste_flagt[2]++;
		if(ste_flagt[2]>STE_TS)
		{
			ste_status3=STE_ABNORMAL;
		}
	}
	if(sw[3]&&chflag4)
	{
		if(steering_current_data[3]>ST_CURR_MAX)
			ste_flagt[3]++;
		if(ste_flagt[3]>STE_TS)
		{
			ste_status4=STE_ABNORMAL;
		}
	}
}

void STE_Check(void)//舵机正常运行时电流检测判断函数
{
	if(!sw[0])
	{
		if(steering_current_data[0]>ST_CURR_MAX)
		{
			ste_flag1+=10;
			if(ste_flag1>=STE_FLAG_MAX)
			{
				ste_flag1=STE_FLAG_MAX;
				ste_status1=STE_ABNORMAL;
				pwr_sw4_reset;
				steering_current_counter[0]=0;
			}
		}
		else
		{
			if(ste_flag1>20)ste_flag1=ste_flag1-20;
			if(ste_flag1<=0)
			{
				ste_flag1=0;
				/*steering_current_counter[0]++;
				if(steering_current_counter[0]>=STE_FLAG_MAXD)
				 {
				    steering_current_counter[0]=STE_FLAG_MAXD;
				    ste_status1=STE_NORMAL;
				 pwr_sw4_set;
				 }*/
			}
		}
	}
	if(!sw[1])
	{
		if(steering_current_data[1]>ST_CURR_MAX)
		{
			ste_flag2+=10;
			if(ste_flag2>=STE_FLAG_MAX)
			{
				ste_flag2=STE_FLAG_MAX;
				ste_status2=STE_ABNORMAL;
				pwr_sw3_reset;
				steering_current_counter[1]=0;
			}
		}
		else
		{
			if(ste_flag2>20)ste_flag2=ste_flag2-20;
			if(ste_flag2<=0)
			{
				ste_flag2=0;
				/*steering_current_counter[1]++;
				if(steering_current_counter[1]>=STE_FLAG_MAXD)
				 {
				    steering_current_counter[1]=STE_FLAG_MAXD;
				    ste_status2=STE_NORMAL;
				 pwr_sw3_set;
				 }*/
			}
		}
	}
	if(!sw[2])
	{
		if(steering_current_data[2]>ST_CURR_MAX)
		{
			ste_flag3+=50;
			if(ste_flag3>=STE_FLAG_MAX)
			{
				ste_flag3=STE_FLAG_MAX;
				ste_status3=STE_ABNORMAL;
				pwr_sw2_reset;
				steering_current_counter[2]=0;
			}
		}
		else
		{
			if(ste_flag3>0)ste_flag3--;
			if(ste_flag3<=0)
			{
				ste_flag3=0;
				steering_current_counter[2]++;
				if(steering_current_counter[2]>=STE_FLAG_MAXD)
				{
					steering_current_counter[2]=STE_FLAG_MAXD;
					ste_status3=STE_NORMAL;
					pwr_sw2_set;
				}
			}
		}
	}
	if(!sw[3])
	{
		if(steering_current_data[3]>ST_CURR_MAX)
		{
			ste_flag4+=50;
			if(ste_flag4>=STE_FLAG_MAX)
			{
				ste_flag4=STE_FLAG_MAX;
				ste_status4=STE_ABNORMAL;
				pwr_sw1_reset;
				steering_current_counter[3]=0;
			}
		}
		else
		{
			if(ste_flag4>0)ste_flag4--;
			if(ste_flag4<=0)
			{
				ste_flag4=0;
				steering_current_counter[3]++;
				if(steering_current_counter[3]>=STE_FLAG_MAXD)
				{
					steering_current_counter[3]=STE_FLAG_MAXD;
					ste_status4=STE_NORMAL;
					pwr_sw1_set;
				}
			}
		}
	}

}
void Usart_Message_read(void)//串口数据获取及保存函数
{
	if(USART2_MESSAGE.status)//避障雷达1数据获取及保存
	{

		radar1_oa_data.data_buff[0]=USART2_MESSAGE.usart_buff[2];
		radar1_oa_data.data_buff[1]=USART2_MESSAGE.usart_buff[3];
		radar1_oa_data.data_buff[2]=USART2_MESSAGE.usart_buff[4];
		radar1_oa_data.dis=(USART2_MESSAGE.usart_buff[2]*256+USART2_MESSAGE.usart_buff[3])*0.01;
		radar1_oa_data.angle=((USART2_MESSAGE.usart_buff[4]&0x7F)*2)-90;
		USART2_MESSAGE.status = FALSE;
		//use_art=USE_USART4;
		//printf("%f\n",radar1_oa_data.dis);/*输出浮点数f=3.141593*/
		//printf(",%f\n",radar1_oa_data.angle);/*输出浮点数f=3.141593*/
	}
	else
	{
		radar1_oa_data.data_buff[0]=0;
		radar1_oa_data.data_buff[1]=0;
		radar1_oa_data.data_buff[2]=0;
		radar1_oa_data.dis=0;
		radar1_oa_data.angle=0;
	}

	if(USART3_MESSAGE.status)//避障雷达2数据获取及保存
	{

		radar2_oa_data.data_buff[0]=USART3_MESSAGE.usart_buff[2];
		radar2_oa_data.data_buff[1]=USART3_MESSAGE.usart_buff[3];
		radar2_oa_data.data_buff[2]=USART3_MESSAGE.usart_buff[4];
		radar2_oa_data.dis=(USART3_MESSAGE.usart_buff[2]*256+USART3_MESSAGE.usart_buff[3])*0.01;
		radar2_oa_data.angle=((USART3_MESSAGE.usart_buff[4]&0x7F)*2)-90;
		USART3_MESSAGE.status = FALSE;
	}
	else
	{
		radar2_oa_data.data_buff[0]=0;
		radar2_oa_data.data_buff[1]=0;
		radar2_oa_data.data_buff[2]=0;
		radar2_oa_data.dis=0;
		radar2_oa_data.angle=0;
	}

	if(Query_Message[0].status)//地形跟踪雷达1数据获取及保存
	{

		radar1_tf_data.data_buff[0]=Query_Message[0].usart_buff[2];
		radar1_tf_data.data_buff[1]=Query_Message[0].usart_buff[3];
		radar1_tf_data.dis=(Query_Message[0].usart_buff[2]*256+Query_Message[0].usart_buff[3])*0.01;
		Query_Message[0].status = FALSE;
		//use_art=USE_USART4;
		//printf("%f\n",radar1_tf_data.dis);/*输出浮点数f=3.141593*/
	}
	else
	{
		radar1_tf_data.data_buff[0]=0;
		radar1_tf_data.data_buff[1]=0;
		radar1_tf_data.dis=0;
	}

	if(Query_Message[1].status)//地形跟踪雷达2数据获取及保存
	{

		radar2_tf_data.data_buff[0]=Query_Message[1].usart_buff[2];
		radar2_tf_data.data_buff[1]=Query_Message[1].usart_buff[3];
		radar2_tf_data.dis=(Query_Message[1].usart_buff[2]*256+Query_Message[1].usart_buff[3])*0.01;
		Query_Message[1].status = FALSE;
	}
	else
	{
		radar2_tf_data.data_buff[0]=0;
		radar2_tf_data.data_buff[1]=0;
		radar2_tf_data.dis=0;
	}
}

void Usart_Raw_Message_send(void)//串口数据获取及发送函数
{
	static u16 u1_i=0, u2_i=0, u3_i=0, u4_i=0, u5_i=0;
	u8 i,j,k;
	u16 temp1_i, temp2_i, temp3_i,temp4_i,temp5_i;

	if(USART2_RAW_MESSAGE.status>0)//雷达1数据获取及保存
	{
		for(i=0; i<RADAR_RAW_LENGTH+4; i++)
		{
			if(u2_i+i>=USART_BUFF_LEN)
			{
				temp2_i = u2_i + i - USART_BUFF_LEN;
			}
			else
			{
				temp2_i = u2_i + i;
			}

			while((USART2->SR&0X40)==0) {} //循环发送,直到发送完毕
			USART_SendData(USART2,USART2_RAW_MESSAGE.usart_buff[temp2_i]);
		}

		u2_i = temp2_i+1;
		USART2_RAW_MESSAGE.status--;
	}

	if(USART3_RAW_MESSAGE.status>0)//雷达1数据获取及保存
	{
		for(j=0; j<RADAR_RAW_LENGTH+4; j++)
		{
			if(u3_i+j>=USART_BUFF_LEN)
			{
				temp3_i = u3_i + j - USART_BUFF_LEN;
			}
			else
			{
				temp3_i = u3_i + j;
			}

			while((USART2->SR&0X40)==0) {} //循环发送,直到发送完毕
			USART_SendData(USART2,USART3_RAW_MESSAGE.usart_buff[temp3_i]);
		}

		u3_i = temp3_i+1;
		USART3_RAW_MESSAGE.status--;
	}

	if(USART1_RAW_MESSAGE.status>0)//雷达3数据获取及保存
	{
		for(k=0; k<RADAR_RAW_LENGTH+4; k++)
		{
			if(u1_i+k>=USART_BUFF_LEN)
			{
				temp1_i = u1_i + k - USART_BUFF_LEN;
			}
			else
			{
				temp1_i = u1_i + k;
			}

			while((USART2->SR&0X40)==0) {} //循环发送,直到发送完毕
			USART_SendData(USART2,USART1_RAW_MESSAGE.usart_buff[temp1_i]);
		}

		u1_i = temp1_i+1;
		USART1_RAW_MESSAGE.status--;
	}
	if(Query_Message[0].status>0)//雷达4数据获取及保存
	{
		for(k=0; k<RADAR_RAW_LENGTH+4; k++)
		{
			if(u4_i+k>=USART_BUFF_LEN)
			{
				temp4_i = u4_i + k - USART_BUFF_LEN;
			}
			else
			{
				temp4_i = u4_i + k;
			}

			while((USART2->SR&0X40)==0) {} //循环发送,直到发送完毕
			USART_SendData(USART2,Query_Message[0].usart_buff[temp4_i]);
		}

		u4_i = temp4_i+1;
		Query_Message[0].status--;
	}
	if(Query_Message[1].status>0)//雷达5数据获取及保存
	{
		for(k=0; k<RADAR_RAW_LENGTH+4; k++)
		{
			if(u5_i+k>=USART_BUFF_LEN)
			{
				temp5_i = u5_i + k - USART_BUFF_LEN;
			}
			else
			{
				temp5_i = u5_i + k;
			}

			//while((USART2->SR&0X40)==0){}//循环发送,直到发送完毕
			//USART_SendData(USART2,Query_Message[1].usart_buff[temp5_i]);
			while((USART2->SR&0X40)==0) {} //循环发送,直到发送完毕
			USART_SendData(USART2,Query_Message[1].usart_buff[temp5_i]);
		}

		u5_i = temp5_i+1;
		Query_Message[1].status--;
	}

}

void Can_Data_Load(void)//CAN发送数据缓存函数
{
	data_tx_buf[0]=radar1_oa_data.data_buff[0];
	data_tx_buf[1]=radar1_oa_data.data_buff[1];
	data_tx_buf[2]=radar1_oa_data.data_buff[2];

	data_tx_buf[3]=radar2_oa_data.data_buff[0];
	data_tx_buf[4]=radar2_oa_data.data_buff[1];
	data_tx_buf[5]=radar2_oa_data.data_buff[2];

	data_tx_buf[6]=radar1_tf_data.data_buff[0];
	data_tx_buf[7]=radar1_tf_data.data_buff[1];

	data_tx_buf[8]=radar2_tf_data.data_buff[0];
	data_tx_buf[9]=radar2_tf_data.data_buff[1];
	data_tx_len+=10;
}
void STE_Ct(void)//舵机校准PWM控制函数
{
	if(sw[0]&&ste_status1==STE_NORMAL&&flagtest1==0)
	{
		if(duty_cycle4>1510||duty_cycle4<1490)
		{
			if(!chflag1)
			{
				ste_flagt[0]=0;
			}
			chflag1=TRUE;

		}
		duty_cycle4=(u16)(((int16_t)duty_cycle4)+(int16_t)duty_step4);
		//if(duty_cycle4>1900) {pwr_sw4_reset;}
		if(duty_cycle4>=ST_THRE_MAX||duty_cycle4<=ST_THRE_MIN)
		{
			chflag1=FALSE;
			duty_cycle4=1500;
			duty_step4=-duty_step4;
		}

		if(ste_flagt[0]==0&&chflag1)
		{
			duty_ste_flag[0]++;
		}
		else
		{
			duty_ste_flag[0]=0;
		}
		if(duty_ste_flag[0]>5) sw[0]=0;
		//printf("duty_ste_flag[0]=%f\n",((float)duty_ste_flag[0]));/*输出浮点数*/
		printf("ste_flagt[0]=%f\n",((float)ste_flagt[0]));/*输出浮点数*/
		ste_flagt[0]=0;
		TIM_SetCompare4(TIM3,(ST_D_MAX-duty_cycle4));
		printf("duty_cycle4=%f\n",((float)duty_cycle4));/*输出浮点数*/

	}
	if(sw[1]&&ste_status2==STE_NORMAL&&flagtest2==0)
	{
		if(duty_cycle3>1510||duty_cycle3<1490)
		{
			if(!chflag2)
			{
				ste_flagt[1]=0;
			}
			chflag2=TRUE;

		}
		duty_cycle3=(u16)(((int16_t)duty_cycle3)+(int16_t)duty_step3);
		//if(duty_cycle3>1900) {pwr_sw3_reset;}
		if(duty_cycle3>=ST_THRE_MAX||duty_cycle3<=ST_THRE_MIN)
		{
			chflag2=FALSE;
			duty_cycle3=1500;
			duty_step3=-duty_step3;
		}

		if(ste_flagt[1]==0&&chflag2)
		{
			duty_ste_flag[1]++;
		}
		else
		{
			duty_ste_flag[1]=0;
		}
		if(duty_ste_flag[1]>5) sw[1]=0;
		//printf("duty_ste_flag[1]=%f\n",((float)duty_ste_flag[1]));/*输出浮点数*/
		printf("ste_flagt[1]=%f\n",((float)ste_flagt[1]));/*输出浮点数*/
		ste_flagt[1]=0;
		TIM_SetCompare3(TIM3,(ST_D_MAX-duty_cycle3));
		printf("duty_cycle3=%f\n",((float)duty_cycle3));/*输出浮点数*/

	}
	if(sw[2]&&ste_status3==STE_NORMAL&&flagtest3==0)
	{
		if(duty_cycle2>1510||duty_cycle2<1490)
		{
			if(!chflag3)
			{
				ste_flagt[2]=0;
			}
			chflag3=TRUE;

		}
		duty_cycle2=(u16)(((int16_t)duty_cycle2)+(int16_t)duty_step2);
		//if(duty_cycle2>1900) {pwr_sw2_reset;}
		if(duty_cycle2>=ST_THRE_MAX||duty_cycle2<=ST_THRE_MIN)
		{
			chflag3=FALSE;
			duty_cycle2=1500;
			duty_step2=-duty_step2;
		}

		if(ste_flagt[2]==0&&chflag3)
		{
			duty_ste_flag[2]++;
		}
		else
		{
			duty_ste_flag[2]=0;
		}
		if(duty_ste_flag[2]>5) sw[2]=0;

		printf("ste_flagt[2]=%f\n",((float)ste_flagt[2]));/*输出浮点数*/
		ste_flagt[2]=0;
		TIM_SetCompare2(TIM3,(ST_D_MAX-duty_cycle2));
		printf("duty_cycle2=%f\n",((float)duty_cycle2));/*输出浮点数*/

	}
	if(sw[3]&&ste_status4==STE_NORMAL&&flagtest4==0)
	{
		if(duty_cycle1>1510||duty_cycle1<1490)
		{
			if(!chflag4)
			{
				ste_flagt[3]=0;
			}
			chflag4=TRUE;

		}
		duty_cycle1=(u16)(((int16_t)duty_cycle1)+(int16_t)duty_step1);
		//if(duty_cycle1>1900) {pwr_sw1_reset;}
		if(duty_cycle1>=ST_THRE_MAX||duty_cycle1<=ST_THRE_MIN)
		{
			chflag4=FALSE;
			duty_cycle1=1500;
			duty_step1=-duty_step1;
		}

		if(ste_flagt[3]==0&&chflag4)
		{
			duty_ste_flag[3]++;
		}
		else
		{
			duty_ste_flag[3]=0;
		}
		if(duty_ste_flag[3]>5) sw[3]=0;

		printf("ste_flagt[3]=%f\n",((float)ste_flagt[3]));/*输出浮点数*/
		ste_flagt[3]=0;
		TIM_SetCompare1(TIM3,(ST_D_MAX-duty_cycle1));
		printf("duty_cycle1=%f\n",((float)duty_cycle1));/*输出浮点数*/

	}


}
void STE_Dt(void)//舵机校准数据处理函数
{
	if(sw[0]&&ste_status1==STE_ABNORMAL)
	{
		flagtest1++;
		if(flagtest1==1)
		{
			chflag1=FALSE;
			//printf("flagtest1=%f\n",((float)flagtest1));/*输出浮点数*/
			printf("duty_cycle4=%f\n",((float)(duty_cycle4-STE_D4)));/*输出浮点数*/
			printf("ste_flagt[0]=%f\n",((float)ste_flagt[0]));/*输出浮点数*/
			ste_flagt[0]=0;
			if(abs(duty_cycle4-1992)<=12) goodsum++;
			else badsum++;
			printf("goodsum=%f\n",((float)goodsum));/*输出浮点数*/
			printf("badsum=%f\n",((float)badsum));/*输出浮点数*/
		}
		duty_cycle_flash[3]=duty_cycle4-STE_D4;//最终校准结果，需保存到flash中
		duty_cymed4=duty_cycle_flash[3];
		duty_flash_write[3]=(u32)(duty_cycle_flash[3]<<16)+(165<<8)+165;
		//WriteFlashOneWord(0,duty_flash_write[3]); //写入数据
		flash_write_flag[3]=TRUE;
		sw[0]=0;
		ste_status1=STE_NORMAL;
		TIM_SetCompare4(TIM3,(ST_D_MAX-(duty_cycle4-STE_D4)));//最终校准结果，需保存到flash中
		duty_cycle4=duty_cycle4-STE_D4;
	}
	if(sw[1]&&ste_status2==STE_ABNORMAL)
	{
		flagtest2++;
		if(flagtest2==1)
		{
			chflag2=FALSE;
			//printf("flagtest2=%f\n",((float)flagtest2));/*输出浮点数*/
			printf("duty_cycle3=%f\n",((float)(duty_cycle3-STE_D3)));/*输出浮点数*/
			printf("ste_flagt[1]=%f\n",((float)ste_flagt[1]));/*输出浮点数*/
			ste_flagt[1]=0;
		}
		duty_cycle_flash[2]=duty_cycle3-STE_D3;
		duty_cymed3=duty_cycle_flash[2];
		duty_flash_write[2]=(u32)(duty_cycle_flash[2]<<16)+(165<<8)+165;
		//WriteFlashOneWord(4,duty_flash_write[2]); //写入数据
		flash_write_flag[2]=TRUE;
		sw[1]=0;
		ste_status2=STE_NORMAL;
		TIM_SetCompare3(TIM3,(ST_D_MAX-(duty_cycle3-STE_D3)));//最终校准结果，需保存到flash中
		duty_cycle3=duty_cycle3-STE_D3;
	}
	if(sw[2]&&ste_status3==STE_ABNORMAL)
	{
		flagtest3++;
		if(flagtest3==1)
		{
			chflag3=FALSE;
			//printf("flagtest3=%f\n",((float)flagtest3));/*输出浮点数*/
			printf("duty_cycle2=%f\n",((float)(duty_cycle2-STE_D2)));/*输出浮点数*/
			printf("ste_flagt[2]=%f\n",((float)ste_flagt[2]));/*输出浮点数*/
			ste_flagt[2]=0;
		}
		duty_cycle_flash[1]=duty_cycle2-STE_D2;
		duty_cymed2=duty_cycle_flash[1];
		duty_flash_write[1]=(u32)(duty_cycle_flash[1]<<16)+(165<<8)+165;
		//WriteFlashOneWord(8,duty_flash_write[1]); //写入数据
		flash_write_flag[1]=TRUE;
		sw[2]=0;
		ste_status3=STE_NORMAL;
		TIM_SetCompare2(TIM3,(ST_D_MAX-(duty_cycle2-STE_D2)));//最终校准结果，需保存到flash中
		duty_cycle2=duty_cycle2-STE_D2;
	}
	if(sw[3]&&ste_status4==STE_ABNORMAL)
	{
		flagtest4++;
		if(flagtest4==1)
		{
			chflag4=FALSE;
			//printf("flagtest4=%f\n",((float)flagtest4));/*输出浮点数*/
			printf("duty_cycle1=%f\n",((float)(duty_cycle1-STE_D1)));/*输出浮点数*/
			printf("ste_flagt[3]=%f\n",((float)ste_flagt[3]));/*输出浮点数*/
			ste_flagt[3]=0;
		}
		duty_cycle_flash[0]=duty_cycle1-STE_D1;
		duty_cymed1=duty_cycle_flash[0];
		duty_flash_write[0]=(u32)(duty_cycle_flash[0]<<16)+(165<<8)+165;
		//WriteFlashOneWord(12,duty_flash_write[0]); //写入数据
		flash_write_flag[0]=TRUE;
		sw[3]=0;
		ste_status4=STE_NORMAL;
		TIM_SetCompare1(TIM3,(ST_D_MAX-(duty_cycle1-STE_D1)));//最终校准结果，需保存到flash中
		duty_cycle1=duty_cycle1-STE_D1;
	}
	if(flash_write_flag[0]||flash_write_flag[1]||flash_write_flag[2]||flash_write_flag[3])
	{
		//if(0==WriteFlashOneWord(4,duty_flash_write)) //写入数据
		//	{
		flash_write_flag[0]=FALSE;
		flash_write_flag[1]=FALSE;
		flash_write_flag[2]=FALSE;
		flash_write_flag[3]=FALSE;
		//	}
	}
}
void STE_Rt(void)//重复校准延时函数
{
	if(sw[0]&&flagtest1>6)
	{
		flagtest1=0;
		duty_cycle4=1500;
		ste_status1=STE_NORMAL;
	}
	if(sw[1]&&flagtest2>6)
	{
		flagtest2=0;
		duty_cycle3=1500;
		ste_status2=STE_NORMAL;
	}
	if(sw[2]&&flagtest3>6)
	{
		flagtest3=0;
		duty_cycle2=1500;
		ste_status3=STE_NORMAL;
	}
	if(sw[3]&&flagtest4>6)
	{
		flagtest4=0;
		duty_cycle1=1500;
		ste_status4=STE_NORMAL;
	}
}

void STE_Control1(void)//舵机校准时PWM控制函数
{
	static u8 tm1=0;

	if(TIME_FLAG.time_sub.flag_1hz)
	{
		TIME_FLAG.time_sub.flag_1hz=FALSE;
		tm1++;
		if(tm1>1)
		{
			tm1=0;
			STE_Ct();
		}
		STE_Dt();
		STE_Rt();
	}
}

void STE_Control(void)//舵机正常运行时PWM控制函数
{
	if(!sw[3])
	{
		if(duty_cycle1>ST_THRE_MAX) duty_cycle1=ST_THRE_MAX;
		else if(duty_cycle1<ST_THRE_MIN) duty_cycle1=ST_THRE_MIN;
		if(ste_status1==STE_ABNORMAL) duty_cycle1=1500;
		TIM_SetCompare1(TIM3,(ST_D_MAX-duty_cycle1));
	}
	if(!sw[2])
	{
		if(duty_cycle2>ST_THRE_MAX) duty_cycle2=ST_THRE_MAX;
		else if(duty_cycle2<ST_THRE_MIN) duty_cycle2=ST_THRE_MIN;
		if(ste_status2==STE_ABNORMAL) duty_cycle2=1500;
		TIM_SetCompare2(TIM3,(ST_D_MAX-duty_cycle2));
	}
	if(!sw[1])
	{
		if(duty_cycle3>ST_THRE_MAX) duty_cycle3=ST_THRE_MAX;
		else if(duty_cycle3<ST_THRE_MIN) duty_cycle3=ST_THRE_MIN;
		if(ste_status3==STE_ABNORMAL) duty_cycle3=1500;
		TIM_SetCompare3(TIM3,(ST_D_MAX-duty_cycle3));
	}
	if(!sw[0])
	{
		if(duty_cycle4>ST_THRE_MAX) duty_cycle4=ST_THRE_MAX;
		else if(duty_cycle4<ST_THRE_MIN) duty_cycle4=ST_THRE_MIN;
		if(ste_status4==STE_ABNORMAL) duty_cycle4=1500;
		TIM_SetCompare4(TIM3,(ST_D_MAX-duty_cycle4));
	}

}

void Get_Oa_Data(void)//uart5接收pwm数据
{
	if( UART5_MESSAGE.status )//雷达1数据获取及保存
	{
		oa_data.pwm = (u16)( (UART5_MESSAGE.usart_buff[0]<<8) + UART5_MESSAGE.usart_buff[1] );
		UART5_MESSAGE.status = FALSE;
	}
}

void Data_Parser(void)//CAN接收数据解析函数
{
	unsigned char i;
	if(Can_Msg_Receive.Over == 1)
	{
		Can_Msg_Receive.Over = 0;
		can_rec_oa_data.len=((u16)(buf[1]<<8))+buf[0]+1-9;
		if(can_rec_oa_data.len==MSG_RE_LEN)
		{

			for(i=0; i<can_rec_oa_data.len; i++)
			{
				can_rec_oa_data.data_buff[i]=buf[i+8];
			}

			dt_can1=(int16_t)((can_rec_oa_data.data_buff[0]<<8)+can_rec_oa_data.data_buff[1]);

			dt_can2=(int16_t)((can_rec_oa_data.data_buff[2]<<8)+can_rec_oa_data.data_buff[3]);

			dt_can3=(int16_t)((can_rec_oa_data.data_buff[4]<<8)+can_rec_oa_data.data_buff[5]);

			dt_can4=(int16_t)((can_rec_oa_data.data_buff[6]<<8)+can_rec_oa_data.data_buff[7]);

		}
	}

	if(!sw[3])
	{
		if((((int16_t)duty_cymed1)+dt_can1)>0)
			duty_cycle1=(u16)(((int16_t)duty_cymed1)+dt_can1);
	}

	if(!sw[2])
	{
		if((((int16_t)duty_cymed2)+dt_can2)>0)
			duty_cycle2=(u16)(((int16_t)duty_cymed2)+dt_can2);
	}
	if(!sw[1])
	{
		if((((int16_t)duty_cymed3)+dt_can3)>0)
			duty_cycle3=(u16)(((int16_t)duty_cymed3)+dt_can3);
	}
	if(!sw[0])
	{
		if((((int16_t)duty_cymed4)+dt_can4)>0)
			duty_cycle4=(u16)(((int16_t)duty_cymed4)+dt_can4);
	}

}
void send_fre(u8 number)
{
	u8 fre_H=0,fre_L=0,i;
	float fre_f;
	u16 fre_i;

	// send_data(number);
	// sum+=number;

	fre_f=Frequency[number-1]/20*60+0.4;
	for(i=2; i<=10; i++)
	{
		if(fabs(prm[number-1]-i*fre_f)<=i*15)
		{
			fre_f=i*fre_f;
			break;
		}
	}
	prm[number-1]=fre_f;
	fre_i=(u16)fre_f;

	fre_L=(u8)(fre_i&0x00ff);
	send_data_USART2(fre_L);
	sum+=fre_L;

	fre_H=(u8)((fre_i>>8)&0x00ff);
	send_data_USART2(fre_H);
	sum+=fre_H;
	/*
	     if(number==1)
	printf("RPM1=%d\n",fre_i);
	else if(number==2)
	printf("RPM2=%d\n",fre_i);
	else if(number==3)
	printf("RPM3=%d\n",fre_i);
	else if(number==4)
	printf("RPM4=%d\n",fre_i);
	else if(number==5)
	printf("RPM5=%d\n",fre_i);
	else if(number==6)
	printf("RPM6=%d\n",fre_i);
	   */




}
void flash_read(uint32_t SReadAddress,int32_t ReadNum)//FLASH数据读取函数
{
	uint8_t Temp_Data[30] = {0,0,0,0,0,
													 0,0,0,0,0,
													 0,0,0,0,0,
													 0,0,0,0,0,
													 0,0,0,0,0,
													 0,0,0,0,0
													};
	//int32_t ReadNum = 20;

	//sw[0]=1;sw[1]=1;sw[2]=1;sw[3]=1;
	if(SReadAddress==0)
	{
		ReadNum =(int32_t) ReadFlashNBtye(SReadAddress, Temp_Data,ReadNum); //读取数据,先读低位后读高位
		gyro_scale_zero.flag=(Temp_Data[1]<<8)+Temp_Data[0];
		if((gyro_scale_zero.flag&0x00ff)==GRO_FLAG_FLASH)
		{
			gyro_scale_zero.x_offset=((Temp_Data[7]<<24)+(Temp_Data[6]<<16)+(Temp_Data[5]<<8)+Temp_Data[4])/32768.0;
			gyro_scale_zero.y_offset=((Temp_Data[11]<<24)+(Temp_Data[10]<<16)+(Temp_Data[9]<<8)+Temp_Data[8])/32768.0;
			gyro_scale_zero.z_offset=((Temp_Data[15]<<24)+(Temp_Data[14]<<16)+(Temp_Data[13]<<8)+Temp_Data[12])/32768.0;
		}
	}
	if(SReadAddress==16)
	{
		ReadNum =(int32_t) ReadFlashNBtye(SReadAddress, Temp_Data,ReadNum); //读取数据,先读低位后读高位
		acc_scale_zero.flag=(Temp_Data[1]<<8)+Temp_Data[0];
		if((acc_scale_zero.flag&0x00ff)==ACC_FLAG_FLASH)
		{
			acc_scale_zero.ax_gain=((Temp_Data[7]<<24)+(Temp_Data[6]<<16)+(Temp_Data[5]<<8)+Temp_Data[4])/32768.0;
			acc_scale_zero.ay_gain=((Temp_Data[11]<<24)+(Temp_Data[10]<<16)+(Temp_Data[9]<<8)+Temp_Data[8])/32768.0;
			acc_scale_zero.az_gain=((Temp_Data[15]<<24)+(Temp_Data[14]<<16)+(Temp_Data[13]<<8)+Temp_Data[12])/32768.0;
			acc_scale_zero.ox_offset=((Temp_Data[19]<<24)+(Temp_Data[18]<<16)+(Temp_Data[17]<<8)+Temp_Data[16])/32768.0;
			acc_scale_zero.oy_offset=((Temp_Data[23]<<24)+(Temp_Data[22]<<16)+(Temp_Data[21]<<8)+Temp_Data[20])/32768.0;
			acc_scale_zero.oz_offset=((Temp_Data[27]<<24)+(Temp_Data[26]<<16)+(Temp_Data[25]<<8)+Temp_Data[24])/32768.0;
			if(fabs(acc_scale_zero.ax_gain-1.0)>0.1 ||
					fabs(acc_scale_zero.ay_gain-1.0)>0.1 ||
					fabs(acc_scale_zero.az_gain-1.0)>0.1)
			{
				acc_scale_zero.ax_gain=1.0;
				acc_scale_zero.ay_gain=1.0;
				acc_scale_zero.az_gain=1.0;
				acc_scale_zero.ox_offset=0.0;
				acc_scale_zero.oy_offset=0.0;
				acc_scale_zero.oz_offset=0.0;
			}
			else if(fabs(acc_scale_zero.ox_offset)>0.5*9.81 ||
							fabs(acc_scale_zero.oy_offset)>0.5*9.81 ||
							fabs(acc_scale_zero.oz_offset)>0.5*9.81)
			{
				acc_scale_zero.ox_offset=0.0;
				acc_scale_zero.oy_offset=0.0;
				acc_scale_zero.oz_offset=0.0;
				acc_scale_zero.ax_gain=1.0;
				acc_scale_zero.ay_gain=1.0;
				acc_scale_zero.az_gain=1.0;

			}
		}
	}

#if 0
	if(Temp_Data[0]!=165&&Temp_Data[1]!=165)
	{
		sw[3]=1;
	}
	else
	{
		duty_cymed1=(u16)(Temp_Data[3]<<8)+(Temp_Data[2]);
		printf("duty_cymed1=%f\n",((float)duty_cymed1));/*输出浮点数*/
		duty_flash_write[0]=(u32)(duty_cymed1<<16)+(165<<8)+165;
	}
	if(Temp_Data[4]!=165&&Temp_Data[5]!=165)
	{
		sw[2]=1;
	}
	else
	{
		duty_cymed2=(u16)(Temp_Data[7]<<8)+(Temp_Data[6]);
		printf("duty_cymed2=%f\n",((float)duty_cymed2));/*输出浮点数*/
		duty_flash_write[1]=(u32)(duty_cymed2<<16)+(165<<8)+165;
	}

	if(Temp_Data[8]!=165&&Temp_Data[9]!=165)
	{
		sw[1]=1;
	}
	else
	{
		duty_cymed3=(u16)(Temp_Data[11]<<8)+(Temp_Data[10]);
		printf("duty_cymed3=%f\n",((float)duty_cymed3));/*输出浮点数*/
		duty_flash_write[2]=(u32)(duty_cymed3<<16)+(165<<8)+165;
	}

	if(Temp_Data[12]!=165&&Temp_Data[13]!=165)
	{
		sw[0]=1;
	}
	else
	{
		duty_cymed4=(u16)(Temp_Data[15]<<8)+(Temp_Data[14]);
		printf("duty_cymed4=%f\n",((float)duty_cymed4));/*输出浮点数*/
		duty_flash_write[3]=(u32)(duty_cymed4<<16)+(165<<8)+165;
	}
	TIM_SetCompare1(TIM3,(ST_D_MAX-duty_cymed1));
	TIM_SetCompare2(TIM3,(ST_D_MAX-duty_cymed2));
	TIM_SetCompare3(TIM3,(ST_D_MAX-duty_cymed3));
	TIM_SetCompare4(TIM3,(ST_D_MAX-duty_cymed4));
#endif
}

void Run_command(void)//循环运行所有功能函数
{
	//static float rt = 0;
	static float timepre=0;
//  static uint32_t timecount=0;
	static u8 flat=0;
	u8 i;
	u32 flashdata[7]= {0,0,0,0,0,0,0};
	if(TIME_FLAG.time_sub.flag_50hz)
	{
		TIME_FLAG.time_sub.flag_50hz=FALSE;

		Current_AD_Read();
		STE_Check();
		Radar_Center_Data_Send();

		if((gyro_scale_zero.flag&0x00ff)!=GRO_FLAG_FLASH)

		{
			if(mpu_value_f.state==1&&Gyro_Calibration()==0)
			{
				gyro_scale_zero.flag=GRO_FLAG_FLASH;
				// flashdata[0]=(0<<16)+gyro_scale_zero.flag;
				// flashdata[1]=((int32_t)(gyro_scale_zero.x_offset*32768));
				// flashdata[2]=((int32_t)(gyro_scale_zero.y_offset*32768));
				// flashdata[3]=((int32_t)(gyro_scale_zero.z_offset*32768));
				// WriteFlashOneWord(0,3,flashdata);
				// gyro_scale_zero.flag=0;
				// flash_read(0,16);
			}
		}
	}
	if(TIME_FLAG.time_sub.flag_500hz)
	{
		TIME_FLAG.time_sub.flag_500hz=FALSE;

		if((mpu_value_f.state==1)&&((gyro_scale_zero.flag&0x00ff)==GRO_FLAG_FLASH))//&&((acc_scale_zero.flag&0x00ff)==ACC_FLAG_FLASH))
		{
			Ckf_main();
			Rool  = fRPY[0];
			Pitch = fRPY[1]; //(float)oa_data.pwm;//(float)USART2_RAW_MESSAGE.roll_cnt;//steering_current_data_f[1];//;
			Yaw   = fRPY[2]; //(float)USART3_RAW_MESSAGE.roll_cnt;//0;//
		}
		if((mpu_value_f.state==1)&&
				((acc_scale_zero.flag&0x00ff)!=ACC_FLAG_FLASH) &&
				(acc_calc()==7))
		{

			acc_scale_zero.ax_gain=(float)m_result[0];
			acc_scale_zero.ay_gain=(float)m_result[1];
			acc_scale_zero.az_gain=(float)m_result[2];

			acc_scale_zero.ox_offset=(float)m_result[3];
			acc_scale_zero.oy_offset=(float)m_result[4];
			acc_scale_zero.oz_offset=(float)m_result[5];

			acc_scale_zero.flag=ACC_FLAG_FLASH;
			flashdata[0]=(0<<16)+acc_scale_zero.flag;
			flashdata[1]=((int32_t)(acc_scale_zero.ax_gain*32768));
			flashdata[2]=((int32_t)(acc_scale_zero.ay_gain*32768));
			flashdata[3]=((int32_t)(acc_scale_zero.az_gain*32768));
			flashdata[4]=((int32_t)(acc_scale_zero.ox_offset*32768));
			flashdata[5]=((int32_t)(acc_scale_zero.oy_offset*32768));
			flashdata[6]=((int32_t)(acc_scale_zero.oz_offset*32768));

			WriteFlashOneWord(4,9,flashdata);

			acc_scale_zero.flag=0;
			flash_read(16,28);

		}

	}

	if(1==0&&TIME_FLAG.time_sub.flag_200_1hz)
	{
		TIME_FLAG.time_sub.flag_200_1hz=FALSE;
		if(flat>=1)
		{
			halfT=0.5*((float) millis()*1.0/2000.0-timepre);
			sampleFreq=2/halfT;
		}
		timepre=(float) millis()*1.0/2000.0;

		if(flat==0)
		{
			// timecount=millis();
			flat=1;
		}

		//if((gyro_scale_zero.flag&0x00ff)==GRO_FLAG_FLASH)
		//	{
		//mpu_9250_handing();
		// acc_calc();
		//	}
		// use_art=USE_USART4;
		// printf("Pitch=%f\n",Pitch);/*输出浮点数*/
		// printf("Rool=%f\n",Rool);/*输出浮点数*/
		//  printf("Yaw=%f\n",Yaw);/*输出浮点数*/
	}
	if(1==0&&TIME_FLAG.time_sub.flag_100hz)
	{
		TIME_FLAG.time_sub.flag_100hz=FALSE;
		//use_art=USE_USART2;
		//Frequency[0]=504.32;
		sum=0;
		send_data_USART2(0xCD);
		sum+=0xCD;
		send_data_USART2(0xCD);
		sum+=0xCD;
		for(i=1; i<7; i++)
		{
			send_fre(i);
		}
		send_data_USART2(sum);

	}
	//Query_uart(UART4,0);//地形跟踪雷达1数据获取及保存
	//Query_uart(UART5,1);//地形跟踪雷达2数据获取及保存
	Usart_Raw_Message_send();

	if(TIME_FLAG.time_sub.flag_20hz)
	{
		TIME_FLAG.time_sub.flag_20hz=FALSE;

		Get_Oa_Data();

		//rt = rt + 1.0/20.0;
		//if(rt>1000000)
		// rt = 0;
		//oa_data.pwm = 1800+500*sin(2*PI*1.0f*rt);

		TIM_SetCompare4(TIM3,(ST_D_MAX-oa_data.pwm));//pwm1
		TIM_SetCompare3(TIM3,(ST_D_MAX-oa_data.pwm));//pwm2

		printf("pwm=%d\n",oa_data.pwm);/*输出浮点数*/
	}



#if  0
	if(TIME_FLAG.time_sub.flag_50hz)
	{
		TIME_FLAG.time_sub.flag_50hz=FALSE;

		if(Can_Msg_Send.Start ==0&&Can_Msg_Send.Over==0)
		{
			Can_Msg_Send.Start = 0;
			Can_Msg_Send.Over = 1;
			data_tx_len=0;
			Usart_Message_read();
			Can_Data_Load();
			data_tx_buf[10]=ste_status1;
			data_tx_buf[11]=ste_status2;
			data_tx_buf[12]=ste_status3;
			data_tx_buf[13]=ste_status4;
			data_tx_len+=4;
			data_tx_buf[14]=(u8)(steering_current_data[0]>>8);
			data_tx_buf[15]=steering_current_data[0];

			data_tx_buf[16]=(u8)(steering_current_data[1]>>8);
			data_tx_buf[17]=steering_current_data[1];

			data_tx_buf[18]=(u8)(steering_current_data[2]>>8);
			data_tx_buf[19]=steering_current_data[2];

			data_tx_buf[20]=(u8)(steering_current_data[3]>>8);
			data_tx_buf[21]=steering_current_data[3];

			data_tx_len+=8;
		}

	}
	if(TIME_FLAG.time_sub.flag_200hz)
	{
		TIME_FLAG.time_sub.flag_200hz=FALSE;
		Can_Tx_Frame_Loop(1,2,data_tx_len,data_tx_buf);
	}
	Can_Rx_Read_Data();
	Data_Parser();
	STE_Control();
	STE_Control1();
	if(TIME_FLAG.time_sub.flag_2000hz)
	{
		TIME_FLAG.time_sub.flag_2000hz=FALSE;
		Current_AD_Read1();
	}
	if(TIME_FLAG.time_sub.flag_100hz)
	{
		TIME_FLAG.time_sub.flag_100hz=FALSE;
		Current_AD_Read();
		STE_Check();
		STE_Check1();
	}
#endif


}


