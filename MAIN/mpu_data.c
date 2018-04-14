#include "spi.h"
#include "mpu_data.h"
#include <math.h>
#include "delay.h"
#include "commander.h"

/********************************************************************
***函数名称:MPU_9250DATA()
***函数说明:data 数据四元数解算
***输入参数:原始数据
***输出参数:姿态角
***
********************************************************************/
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
float gx=0,gy=0,gz=0,ax0=0,ay0=0,az0=0,mx0=0,my0=0,mz0=0;          //全局变量
float Yaw,Pitch,Rool;  //偏航角，俯仰角，翻滚角
float integralFBx=0.0,integralFBy=0.0,integralFBz=0.0;
float exInt = 0, eyInt = 0, ezInt = 0;        // 按比例缩小积分误差
float halfT=0.5/500.0f ;               // 采样周期的一半
float sampleFreq=500.0f;
gro_calibration gyro_scale_zero;
gro_calibration gyro_scale_zero_0;
acc_calibration acc_scale_zero;

u8 whoami=0;
float acc_tau=1.0/(2.0*3.1415926*ACC_FC);
float gro_tau=1.0/(2.0*3.1415926*GRO_FC);

SencondOrderLowPass acc_fiter_x;
SencondOrderLowPass acc_fiter_y;
SencondOrderLowPass acc_fiter_z;

SencondOrderLowPass gro_fiter_x;
SencondOrderLowPass gro_fiter_y;
SencondOrderLowPass gro_fiter_z;

void int_second_order_low_pass(SencondOrderLowPass* fiter,float tau,float sample_time,float value)
{
	fiter->coef = 1.0/tau*sample_time;
	if(fiter->coef > 1.0)fiter->coef = 0;
	fiter->o[0] = value;

}
float update_second_order_low_pass(SencondOrderLowPass* fiter,float value)
{
	fiter->o[0]=fiter->o[0]+(value-fiter->o[0])*fiter->coef;
	fiter->o[1]=fiter->o[1]+(fiter->o[0]-fiter->o[1])*fiter->coef;
	return  fiter->o[1];

}
void Get_Gyro_Value(float *gyro_x,float *gyro_y,float *gyro_z)
{
	gyro_scale_zero_0.x_offset+=(*gyro_x);
	gyro_scale_zero_0.y_offset+=(*gyro_y);
	gyro_scale_zero_0.z_offset+=(*gyro_z);
}
u8 Gyro_Calibration(void)
{
	const u16 calibration_count=100;
	float xdiff=0;	//差值
	float ydiff=0;	//差值
	float zdiff=0;	//差值
	const float max_off=1.0*PI/180.0;	//最大偏差值
	static u16 calibration_counter=0;


	gyro_scale_zero.x_offset=0;
	gyro_scale_zero.y_offset=0;
	gyro_scale_zero.z_offset=0;



	if(calibration_counter<calibration_count)
	{
		//获取陀螺仪的值
		READ_MPU9250_GYRO();
		//delay(2);
		if(fabs(mpu_value_f.Gyro[0])<(1.0*PI/180.0)&&
				fabs(mpu_value_f.Gyro[1])<(1.0*PI/180.0)&&
				fabs(mpu_value_f.Gyro[2])<(1.0*PI/180.0))
		{
			Get_Gyro_Value(&mpu_value_f.Gyro[0],&mpu_value_f.Gyro[1],&mpu_value_f.Gyro[2]);
			calibration_counter++;
		}
		return 2;
	}
	READ_MPU9250_GYRO();
	gyro_scale_zero_0.x_offset/=calibration_counter;
	gyro_scale_zero_0.y_offset/=calibration_counter;
	gyro_scale_zero_0.z_offset/=calibration_counter;

	//获取陀螺仪的值

	xdiff=mpu_value_f.Gyro[0]-gyro_scale_zero_0.x_offset;
	ydiff=mpu_value_f.Gyro[1]-gyro_scale_zero_0.y_offset;
	zdiff=mpu_value_f.Gyro[2]-gyro_scale_zero_0.z_offset;

	if((fabs(xdiff)>max_off)||
			(fabs(ydiff)>max_off)||
			(fabs(zdiff)>max_off))
	{
		gyro_scale_zero_0.x_offset=0;
		gyro_scale_zero_0.y_offset=0;
		gyro_scale_zero_0.z_offset=0;
		calibration_counter=0;
		return 1;	//失败

	}
	gyro_scale_zero.x_offset=gyro_scale_zero_0.x_offset;
	gyro_scale_zero.y_offset=gyro_scale_zero_0.y_offset;
	gyro_scale_zero.z_offset=gyro_scale_zero_0.z_offset;
	return 0;	//成功

}
void mpu_mag_read(void)
{
	READ_MPU9250_MAG();
}

void mpu_9250_read(void)
{
	//  static u8 count=0;
	// whoami=0;
	// whoami=MPU9250_Read_Reg(WHO_AM_I);

	READ_MPU9250_ACCEL();
	READ_MPU9250_GYRO();
	//if(count>=5)
	//{
	// READ_MPU9250_MAG();
	// count=0;
	//}
	//count++;
}
void IMUupdate1(float gxi, float gyi, float gzi, float axi, float ayi, float azi)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	//增加互补滤波
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
//        float q0q3 = q0*q3;
	float q1q1 = q1*q1;
//        float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	float ax=0,ay=0,az=0;
	float qa,qb,qc;
	ax0=update_second_order_low_pass(&acc_fiter_x,axi);
	ay0=update_second_order_low_pass(&acc_fiter_y,ayi);
	az0=update_second_order_low_pass(&acc_fiter_z,azi);
	//ax0=ax0*kfa+kfan*axi;
	// ay0=ay0*kfa+kfan*ayi;
	//az0=az0*kfa+kfan*azi;
	gx=gxi;//update_second_order_low_pass(&gro_fiter_x,gxi);
	gy=gyi;//update_second_order_low_pass(&gro_fiter_y,gyi);
	gz=gzi;//update_second_order_low_pass(&gro_fiter_z,gzi);

	// gx=gx*kfg+kfgn*gxi;
	//  gy=gy*kfg+kfgn*gyi;
	//  gz=gz*kfg+kfgn*gzi;

	// 测量正常化
	norm = sqrt(ax0*ax0 + ay0*ay0 + az0*az0);
	ax = ax0 / norm;
	ay = ay0 / norm;
	az = az0 / norm;

	// 估计方向的重力
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 -q1q1-q2q2 + q3q3;

	// 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	// 积分误差比例积分增益
	exInt = exInt + ex*Ki/sampleFreq;
	eyInt = eyInt + ey*Ki/sampleFreq;
	ezInt = ezInt + ez*Ki/sampleFreq;

	// 调整后的陀螺仪测量
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	gx*=halfT;
	gy*=halfT;
	gz*=halfT;
	// 整合四元数率和正常化
	qa=q0;
	qb=q1;
	qc=q2;
	q0 = q0 + (-qb*gx - qc*gy - q3*gz);
	q1 = q1 + (qa*gx + qc*gz - q3*gy);
	q2 = q2 + (qa*gy - qb*gz + q3*gx);
	q3 = q3 + (qa*gz + qb*gy - qc*gx);

	// 正常化四元
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
	if(Pitch>=180)Pitch=180;
	else if(Pitch<=-180)Pitch=-180;
	Pitch=-Pitch;
	Rool   = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
	//if((-2 * q1 * q1 - 2 * q2* q2 + 1)<0 &&Rool>0) Rool=Rool-180;
	//else if((-2 * q1 * q1 - 2 * q2* q2 + 1)<0 &&Rool<0) Rool=Rool+180;
	if(Rool>=180)Rool=180;
	else if(Rool<=-180)Rool=-180;
	//Rool=180-Rool;

	// Yaw  =atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3* q3 + 1)* 57.3; // Yaw
}
void IMUupdate(float gxi, float gyi, float gzi, float axi, float ayi, float azi,float mxi, float myi, float mzi)
{
	float norm;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	//增加互补滤波
	float q0q0=q0*q0;
	float q0q1=q0*q1;
	float q0q2=q0*q2;
	float q0q3=q0*q3;
	float q1q1=q1*q1;
	float q1q2=q1*q2;
	float q1q3=q1*q3;
	float q2q2=q2*q2;
	float q2q3=q2*q3;
	float q3q3=q3*q3;
	float ax=0,ay=0,az=0,mx=0,my=0,mz=0;

	if((mxi==0.0f)&&(myi==0.0f)&&(mzi==0.0f))
	{
		IMUupdate1(gxi, gyi, gzi, axi, ayi, azi);
		return;
	}

	mx0=mx0*kfm+kfmn*mxi;
	my0=my0*kfm+kfmn*myi;
	mz0=mz0*kfm+kfmn*mzi;
	norm = 1.0f/sqrtf(mx0 * mx0 + my0 * my0 + mz0 * mz0);
	mx = mx0*norm;
	my =my0*norm;
	mz =mz0*norm;


	if(!((axi==0.0f)&&(ayi==0.0f)&&(azi==0.0f)))
	{
		//   ax0=ax0*kfa+kfan*axi;
		//    ay0=ay0*kfa+kfan*ayi;
		//    az0=az0*kfa+kfan*azi;
		ax0=update_second_order_low_pass(&acc_fiter_x,axi);
		ay0=update_second_order_low_pass(&acc_fiter_y,ayi);
		az0=update_second_order_low_pass(&acc_fiter_z,azi);

		// gx=gx*kfg+kfgn*gxi;
		// gy=gy*kfg+kfgn*gyi;
		// gz=gz*kfg+kfgn*gzi;
		gx=update_second_order_low_pass(&gro_fiter_x,gxi);
		gy=update_second_order_low_pass(&gro_fiter_y,gyi);
		gz=update_second_order_low_pass(&gro_fiter_z,gzi);

		norm =1.0f/sqrtf(ax0*ax0 + ay0*ay0 + az0*az0);
		ax = ax0*norm;
		ay = ay0*norm;
		az = az0*norm;

		hx=2.0f * (mx * (0.5f -q2q2 - q3q3) + my * (q1q2-q0q3) + mz * (q1q3 + q0q2));
		hy=2.0f * (mx * (q1q2 + q0q3) + my * (0.5f-q1q1-q3q3) + mz * (q2q3-q0q1));
		bx=sqrt(hx * hx + hy * hy);
		bz=2.0f * (mx * (q1q3-q0q2) + my * (q2q3 + q0q1) + mz * (0.5f -q1q1-q2q2));

		halfvx=q1q3-q0q2;
		halfvy=q0q1+q2q3;
		halfvz=q0q0-0.5f+q3q3;
		halfwx=bx*(0.5f-q2q2-q3q3)+bz*(q1q3-q0q2);
		halfwy=bx*(q1q2-q0q3)+bz*(q0q1+q2q3);
		halfwz=bx*(q0q2 + q1q3)+bz*(0.5f-q1q1-q2q2);


		halfex=(ay*halfvz-az*halfvy)+(my*halfwz-mz*halfwy);
		halfey=(az*halfvx-ax*halfvz)+(mz*halfwx-mx*halfwz);
		halfez=(ax*halfvy-ay*halfvx)+(mx*halfwy-my*halfwx);

		if(Ki > 0.0f)
		{
			integralFBx += Ki * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
			integralFBy += Ki * halfey * (1.0f / sampleFreq);
			integralFBz += Ki * halfez * (1.0f / sampleFreq);
			gx += integralFBx; // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
		// Apply proportional feedback
		gx += Kp * halfex;
		gy += Kp * halfey;
		gz += Kp * halfez;
	}
	gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0+=(-qb*gx-qc*gy-q3*gz);
	q1+=(qa*gx+qc*gz-q3*gy);
	q2+=(qa*gy-qb*gz+q3*gx);
	q3+=(qa*gz+qb*gy-qc*gx);

	norm =1.0f/sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;
	Pitch  = asin(-2*q1*q3+2*q0*q2)*57.3; // pitch
	Rool   = atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.3; // rollv
	Yaw    = atan2(2*q1*q2+2*q0*q3,-2*q2*q2-2*q3*q3+1)*57.3; // Yaw
}

void Radar_Center_Data_Send(void)
{
	static u8 BUF[8]= {0,0,0,0,0,0,0,0},sum,i;
	//static float roll[6];
	static u8 bi=0;
#if 1
	static u16 ai=0;
	ai++;
	if(ai>50)
	{
		ai=0;
		ahrs_send(0x53,BUF);
	}
#endif
	//roll[j]=Rool;
	//j++;
	//if(j>=5)j=0;
	//roll[5]=(roll[0]+roll[1]+roll[2]+roll[3]+roll[4])/5.0;

	//pitch[k]=Pitch;
	//k++;
	//if(k>=5)k=0;
	//pitch[5]=(pitch[0]+pitch[1]+pitch[2]+pitch[3]+pitch[4])/5.0;

	//bi++;
	//if(bi>9)
	//{
	BUF[0]=(u8)(((int16_t)(Rool/180.0*32768))&0xff);//低位
	BUF[1]=(u8)(((int16_t)(Rool/180.0*32768))>>8);//高位

	BUF[2]=(u8)(((int16_t)(steering_current_data_f[0]/10.0*32768))&0xff);
	BUF[3]=(u8)(((int16_t)(steering_current_data_f[0]/10.0*32768))>>8);

	BUF[4]=(u8)(((int16_t)(steering_current_data_f[1]/10.0*32768))&0xff);
	BUF[5]=(u8)(((int16_t)(steering_current_data_f[1]/10.0*32768))>>8);

	if(ste_status1==STE_ABNORMAL)
		BUF[6]|=0x01;
	if(ste_status2==STE_ABNORMAL)
		BUF[6]|=0x02;
	if(ste_status3==STE_ABNORMAL)
		BUF[6]|=0x04;
	if(ste_status4==STE_ABNORMAL)
		BUF[6]|=0x08;

	bi=0;
	sum=0;
	send_data_USART2(0xEE);
	sum+=0xEE;
	send_data_USART2(0xEE);
	sum+=0xEE;

	for(i=0; i<8; i++)
	{
		send_data_USART2(BUF[i]);
		sum+=BUF[i];
	}
	send_data_USART2(sum);
	//}

}
void mpu_9250_handing(void)
{

	mpu_9250_read();
	IMUupdate(mpu_value_f.Gyro[0],mpu_value_f.Gyro[1],mpu_value_f.Gyro[2],
						mpu_value_f.Accel[0],mpu_value_f.Accel[1],mpu_value_f.Accel[2],
						mpu_value_f.Mag[1],mpu_value_f.Mag[0],-mpu_value_f.Mag[2]);

}

