#ifndef __MPU_DATA_H_
#define __MPU_DATA_H_
#include "stm32f10x.h"
#include "spi.h"
#ifndef PI
#define PI					3.14159265358979f
#endif

//互补滤波
//时间常数   t=a/(1-a)*dt    a=t/(t+dt)         t 截至频率  dt计算间隔时间
#define        kfa   0.95
#define        kfan  (1.0-kfa)

#define        kfm   0.9
#define        kfmn  (1.0-kfm)

#define        kfg   0.08
#define        kfgn  (1.0-kfg)

#define Kp 2.0f                        // 比例增益支配率收敛到加速度计/磁强计
#define Ki 0.05f                // 积分增益支配率的陀螺仪偏见的衔接

#define ACC_FC 25.0
#define GRO_FC 25.0

typedef struct 
{
	float x_offset;		//x偏移量
	float y_offset;		//y偏移量
	float z_offset;		//z偏移量
	u16   flag;		//
}gro_calibration;

typedef struct 
{
	float ax_gain;		//x缩放量
	float ay_gain;		//y缩放量
	float az_gain;		//z缩放量
	float ox_offset;		//x偏移量
	float oy_offset;		//y偏移量
	float oz_offset;		//z偏移量
	u16   flag;		//
}acc_calibration;

typedef struct 
{
	float o[2];		//
	float   coef;		//
}SencondOrderLowPass;

extern float halfT;               // 采样周期的一半
extern float sampleFreq;
extern float Yaw,Pitch,Rool;  //偏航角，俯仰角，翻滚角
extern u8 whoami;
extern gro_calibration gyro_scale_zero;
extern acc_calibration acc_scale_zero;

extern SencondOrderLowPass acc_fiter_x;
extern SencondOrderLowPass acc_fiter_y;
extern SencondOrderLowPass acc_fiter_z;
extern SencondOrderLowPass gro_fiter_x;
extern SencondOrderLowPass gro_fiter_y;
extern SencondOrderLowPass gro_fiter_z;

extern float acc_tau;
extern float gro_tau;

void mpu_9250_handing(void);
void mpu_mag_read(void);
u8 Gyro_Calibration(void);
void mpu_9250_read(void);
void Radar_Center_Data_Send(void);

void int_second_order_low_pass(SencondOrderLowPass* fiter,float tau,float sample_time,float value);
float update_second_order_low_pass(SencondOrderLowPass* fiter,float value);


#endif


