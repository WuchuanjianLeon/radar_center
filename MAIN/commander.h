#ifndef __COMMANDER_H_
#define __COMMANDER_H_

#include "ad.h"
#include "delay.h"
#include "init.h"
#include "interrupt.h"

#define ST_D_MAX (20000-1) //40000
#define GRO_FLAG_FLASH 0x0044
#define ACC_FLAG_FLASH 0x0077
#define STE_ABNORMAL 22 //表示舵机异常

typedef enum
{
	USE_USART1=0,
	USE_USART2,
	USE_USART3,
	USE_USART4,
	USE_USART5,
}USE_USART_SELECTION;
typedef struct
{   
	u32 stid;
	u32 etid;
    u8 len;
	unsigned char data_buff[8];//can总线待发原始数据包缓存
}CAN_SEND_DATA;
typedef struct
{   
	u32 stid;
	u32 etid;
    u16 len;
	u8 status;
	unsigned char data_buff[8];//can总线待发原始数据包缓存
}CAN_REC_DATA;



typedef struct
{
	unsigned char data_buff[3];//雷达原始数据缓存
	float dis;		//if true接收完标志
	float angle;
}RADAR_OA_DATA;

typedef struct
{
	unsigned char data_buff[2];//雷达原始数据缓存
	float dis;		//if true接收完标志
}RADAR_TF_DATA; //Terrain following

typedef struct
{
	u16 pwm;
	float theta;
}OA_DATA;


extern CAN_SEND_DATA can_send_oa_data;

extern CAN_REC_DATA can_rec_oa_data;
extern CAN_REC_DATA can_rec_tf_data;


//extern CanRxMsg RxMessage;

extern CAN_SEND_DATA can_send_tf_data;


extern RADAR_OA_DATA radar1_oa_data;

extern RADAR_OA_DATA radar2_oa_data;

extern RADAR_TF_DATA radar1_tf_data;

extern RADAR_TF_DATA radar2_tf_data;

extern OA_DATA oa_data;

extern unsigned char use_art;
extern u8 sw[4];
extern u16 duty_cymed1,duty_cymed2,duty_cymed3,duty_cymed4;
extern uint32_t duty_flash_write[4];
extern u8 ste_status1,ste_status2,ste_status3,ste_status4;//舵机运行状态变量
extern float steering_current_data_f[4];//舵机电流检测数据浮点型(单位A)


void Run_command(void);

void flash_read(uint32_t SReadAddress,int32_t ReadNum);//FLASH数据读取函数


#endif

