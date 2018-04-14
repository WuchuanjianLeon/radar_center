#include "imu_srckf.h"
#include "spi.h"
#include "mpu_data.h"
#include <math.h>
#include "delay.h"
//#include "SRCKF.h"
#include "Matrix.h"
#include "Quaternion.h"

//#define USE_SRCKF
#define USE_6AXIS_EKF
//#define USE_9AXIS_EKF

#ifdef USE_SRCKF
#include "SRCKF.h"
#elif defined USE_6AXIS_EKF
#include "miniIMU.h"
#elif defined USE_9AXIS_EKF
#include "miniAHRS.h"
#endif
float fRealGyro[3] = {0}, fRealAccel[3] = {0};
#if !defined USE_6AXIS_EKF && !defined USE_6AXIS_FP_EKF
float fRealMag[3] = {0}, fRealQ[4] = {0};
#endif
long lQuat[4] = {0};

float fRPY[3] = {0};
float fQ[4] = {0};


#ifdef USE_SRCKF
SRCKF_Filter srckf;
#endif
unsigned long ulNowTime = 0;
unsigned long ulLastTime = 0;
float fDeltaTime = 0.0f;
uint32_t u32KFState = 0;

void Ckf_main(void)
{
	if(!u32KFState)
	{
#ifdef USE_SRCKF
		//Create a new EKF object;
		SRCKF_New(&srckf);
#endif

	}
	Get_Ms(&ulNowTime);
	mpu_9250_read();
	fRealGyro[0] = -mpu_value_f.Gyro[1]; //DEGTORAD(s16Gyro[0]) * 0.06097560975609756097560975609756f;
	fRealGyro[1] = -mpu_value_f.Gyro[0];//DEGTORAD(s16Gyro[1]) * 0.06097560975609756097560975609756f;
	fRealGyro[2] = -mpu_value_f.Gyro[2];//DEGTORAD(s16Gyro[2]) * 0.06097560975609756097560975609756f;

	fRealGyro[0] = update_second_order_low_pass(&gro_fiter_x,mpu_value_f.Gyro[0]);
	fRealGyro[1] = update_second_order_low_pass(&gro_fiter_y,mpu_value_f.Gyro[1]);
	fRealGyro[2] = update_second_order_low_pass(&gro_fiter_z,mpu_value_f.Gyro[2]);

	//if(fabs(fRealGyro[0])<10*3.14159/180) fRealGyro[0]=0;
	//if(fabs(fRealGyro[1])<10*3.14159/180) fRealGyro[1]=0;
	//if(fabs(fRealGyro[2])<10*3.14159/180) fRealGyro[2]=0;
	if(!u32KFState)
	{
		int_second_order_low_pass(&acc_fiter_x,acc_tau,1.0/sampleFreq,mpu_value_f.Accel[0]);
		int_second_order_low_pass(&acc_fiter_y,acc_tau,1.0/sampleFreq,mpu_value_f.Accel[1]);
		int_second_order_low_pass(&acc_fiter_z,acc_tau,1.0/sampleFreq,mpu_value_f.Accel[2]);
	}
	fRealAccel[0] = -mpu_value_f.Accel[1];//update_second_order_low_pass(&acc_fiter_x,mpu_value_f.Accel[0]);//s16Accel[0] / 2048.0f;
	fRealAccel[1] = -mpu_value_f.Accel[0];//update_second_order_low_pass(&acc_fiter_y,mpu_value_f.Accel[1]);//s16Accel[1] / 2048.0f;
	fRealAccel[2] = -mpu_value_f.Accel[2];//update_second_order_low_pass(&acc_fiter_z,mpu_value_f.Accel[2]);//s16Accel[2] / 2048.0f;

	fRealAccel[0] = update_second_order_low_pass(&acc_fiter_x,mpu_value_f.Accel[0]);//s16Accel[0] / 2048.0f;
	fRealAccel[1] = update_second_order_low_pass(&acc_fiter_y,mpu_value_f.Accel[1]);//s16Accel[1] / 2048.0f;
	fRealAccel[2] = update_second_order_low_pass(&acc_fiter_z,mpu_value_f.Accel[2]);//s16Accel[2] / 2048.0f;

#if !defined USE_6AXIS_EKF && !defined USE_6AXIS_FP_EKF
	if(!s32Result)
	{
		fRealMag[0] = mpu_value_f.Mag[0];
		fRealMag[1] = mpu_value_f.Mag[1];
		fRealMag[2] = mpu_value_f.Mag[2];
	}
	//Quaternion_From6AxisData(fRealQ, fRealAccel, fRealMag);
#endif
	if(!u32KFState)
	{
#ifdef  USE_SRCKF
		SRCKF_Init(&srckf, fRealAccel, fRealMag);
#elif defined USE_6AXIS_EKF
		EKF_IMUInit(fRealAccel, fRealGyro);
#elif defined USE_9AXIS_EKF
		EKF_AHRSInit(fRealAccel, fRealMag);

#endif
		ulLastTime = ulNowTime;
		u32KFState = 1;
	}
	else
	{
		fDeltaTime =0.001f * (float)(ulNowTime - ulLastTime);
		//fDeltaTime=0.002;
		//if(fDeltaTime>0.02) fDeltaTime=0.005;
#ifdef USE_SRCKF
		SRCKF_Update(&srckf, fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#elif defined USE_6AXIS_EKF
		EKF_IMUUpdate(fRealGyro, fRealAccel, fDeltaTime);
#elif defined USE_9AXIS_EKF
		EKF_AHRSUpdate(fRealGyro, fRealAccel,fRealMag, fDeltaTime);

#endif
	}

#ifdef USE_SRCKF
	SRCKF_GetAngle(&srckf, fRPY);
	SRCKF_GetQ(&srckf, fQ);
#elif defined USE_6AXIS_EKF
	EKF_IMUGetAngle(fRPY);
	EKF_IMUGetQ(fQ);
#elif defined USE_9AXIS_EKF
	EKF_AHRSGetAngle(fRPY);
	EKF_AHRSGetQ(fQ);
#endif
//	fRPY[0]=fRPY[0]+fRealGyro[0]*fDeltaTime*180/PI;
//	fRPY[1]=fRPY[1]+fRealGyro[1]*fDeltaTime*180/PI;

	fRPY[2]=fDeltaTime*1000.0;
	lQuat[0] = (long)(fQ[0] * 2147483648.0f);
	lQuat[1] = (long)(fQ[1] * 2147483648.0f);
	lQuat[2] = (long)(fQ[2] * 2147483648.0f);
	lQuat[3] = (long)(fQ[3] * 2147483648.0f);
	ulLastTime = ulNowTime;

}




