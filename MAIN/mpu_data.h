#ifndef __MPU_DATA_H_
#define __MPU_DATA_H_
#include "stm32f10x.h"
#include "spi.h"
#ifndef PI
#define PI					3.14159265358979f
#endif

//�����˲�
//ʱ�䳣��   t=a/(1-a)*dt    a=t/(t+dt)         t ����Ƶ��  dt������ʱ��
#define        kfa   0.95
#define        kfan  (1.0-kfa)

#define        kfm   0.9
#define        kfmn  (1.0-kfm)

#define        kfg   0.08
#define        kfgn  (1.0-kfg)

#define Kp 2.0f                        // ��������֧�������������ٶȼ�/��ǿ��
#define Ki 0.05f                // ��������֧���ʵ�������ƫ�����ν�

#define ACC_FC 25.0
#define GRO_FC 25.0

typedef struct 
{
	float x_offset;		//xƫ����
	float y_offset;		//yƫ����
	float z_offset;		//zƫ����
	u16   flag;		//
}gro_calibration;

typedef struct 
{
	float ax_gain;		//x������
	float ay_gain;		//y������
	float az_gain;		//z������
	float ox_offset;		//xƫ����
	float oy_offset;		//yƫ����
	float oz_offset;		//zƫ����
	u16   flag;		//
}acc_calibration;

typedef struct 
{
	float o[2];		//
	float   coef;		//
}SencondOrderLowPass;

extern float halfT;               // �������ڵ�һ��
extern float sampleFreq;
extern float Yaw,Pitch,Rool;  //ƫ���ǣ������ǣ�������
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


