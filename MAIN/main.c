#include "stm32f10x.h"
#include "init.h"
#include "mpu_data.h"
#include "delay.h"
#include "Acc_Calibration.h"

#include "commander.h"
#include "flash_rw.h"
#include <stdio.h>
#include <string.h>
//#include <math.h>

//unsigned char aa;


int main(void)
{
	cycleCounterInit();
	SystemInit();
	Init_All();
	TIME_FLAG.time_all=0x00;
	USART1_MESSAGE.status=0;
	USART2_MESSAGE.status=0;
	USART3_MESSAGE.status=0;

	Query_Message[0].status=0;
	Query_Message[1].status=0;

	Query_Message[0].usart_len=0;
	Query_Message[1].usart_len=0;

	USART1_RAW_MESSAGE.status=0;
	USART2_RAW_MESSAGE.status=0;
	USART3_RAW_MESSAGE.status=0;

	USART1_RAW_MESSAGE.usart_len=0;
	USART2_RAW_MESSAGE.usart_len=0;
	USART3_RAW_MESSAGE.usart_len=0;

	gyro_scale_zero.flag=0;
	acc_scale_zero.flag=0;
	sw[0]=0;
	sw[1]=0;
	sw[2]=0;
	sw[3]=0;
	use_art=USE_USART3;
	//flash_read(0,16);
	flash_read(16,28);
	dis_play(0);


	int_second_order_low_pass(&gro_fiter_x,gro_tau,1.0/sampleFreq,0);
	int_second_order_low_pass(&gro_fiter_y,gro_tau,1.0/sampleFreq,0);
	int_second_order_low_pass(&gro_fiter_z,gro_tau,1.0/sampleFreq,0);
	ResetMatrix();
	while(1)
	{
		Run_command();
		IWDG_Feed();
	}
}


