#include "spi.h"
#include "delay.h"
#include "init.h"
#include "commander.h"
#include "mpu_data.h"
#include "Acc_Calibration.h"
#include <stdio.h>
#include <string.h>

MPU_value mpu_value;          //9轴数据
MPU_value_f mpu_value_f;
u8 sumj=0,regpre=0;       //接收数据缓存区
/***************************************************************/
/***************************************************************/
//SPIx
//TxData:发送一个字节
//返回值:data
/***************************************************************/
static u8 SPI_ReadWriteByte(u8 TxData)
{
	u8 retry=0;
#ifdef SPI1_PERPH
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //等待SPI发送标志位空
	{
		retry++;
		if(retry>200)return 0;
	}
	SPI_I2S_SendData(SPI1, TxData); //发送数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //等待SPI接收标志位空
	{
		retry++;
		if(retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI1); //接收数据
#endif

#ifdef SPI2_PERPH
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //等待SPI发送标志位空
	{
		retry++;
		if(retry>200)return 0;
	}
	SPI_I2S_SendData(SPI2, TxData); //发送数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //等待SPI接收标志位空
	{
		retry++;
		if(retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI2); //接收数据
#endif
}
/***************************************************************/
//SPI发送
//reg: addr
//value:数据
/***************************************************************/
u8 MPU9250_Write_Reg(u8 reg,u8 value)
{
	u8 status;
	MPU_9250_ENABLE;   //	MPU9250_CS=0;  //片选MPU9250
	status=SPI_ReadWriteByte(reg); //发送reg地址
	SPI_ReadWriteByte(value);//发送数据
	MPU_9250_DISENABLE;//	MPU9250_CS=1;  //失能MPU9250
	return(status);//
}
//---------------------------------------------------------------//
//SPI读取
//reg: addr
u8 MPU9250_Read_Reg(u8 reg)
{
	u8 reg_val;
	MPU_9250_ENABLE;//	MPU9250_CS=0;  //片选MPU9250
	SPI_ReadWriteByte(reg|0x80); //reg地址+读命令
	reg_val=SPI_ReadWriteByte(0xff);//任意数据
	MPU_9250_DISENABLE;//	MPU9250_CS=1;  //失能MPU9250
	return(reg_val);
}
/***************************************************************/
// MPU内部i2c 写入
//I2C_SLVx_ADDR:  MPU9250_AK8963_ADDR
//I2C_SLVx_REG:   reg
//I2C_SLVx_Data out:  value
/***************************************************************/
static void i2c_Mag_write(u8 reg,u8 value)
{
	//u16 j=5000;
	MPU9250_Write_Reg(I2C_SLV0_ADDR ,MPU9250_AK8963_ADDR);//设置磁力计地址,mode: write
	MPU9250_Write_Reg(I2C_SLV0_REG ,reg);//set reg addr
	MPU9250_Write_Reg(I2C_SLV0_DO ,value);//send value
	delay(20);
	//while(j--);//此处因为MPU内部I2C读取速度较慢，延时等待内部写完毕
}
/***************************************************************/
// MPU内部i2c 读取
//I2C_SLVx_ADDR:  MPU9250_AK8963_ADDR
//I2C_SLVx_REG:   reg
//return value:   EXT_SENS_DATA_00 register value
/***************************************************************/
static u8 i2c_Mag_read(u8 reg)
{
	u8 j=0;
	MPU9250_Write_Reg(I2C_SLV0_ADDR ,MPU9250_AK8963_ADDR|0x80); //设置磁力计地址，mode：read
	MPU9250_Write_Reg(I2C_SLV0_REG ,reg);// set reg addr
	MPU9250_Write_Reg(I2C_SLV0_DO ,0xff);//read
	delayMicroseconds(500);
	j=MPU9250_Read_Reg(EXT_SENS_DATA_00);
	return j;
}

//****************初始化MPU9250，根据需要请参考pdf进行修改************************
void Init_MPU9250(void)
{
	u8 who;
	who=MPU9250_Read_Reg(WHO_AM_I);
	//use_art=USE_USART2;
	//printf("who=%d\n",who);/*输出浮点数*/
	if(who==0x71)
	{
		mpu_value_f.state=1;
		MPU9250_Write_Reg(PWR_MGMT_1, 0x00);	//解除休眠状态
		delay(10);
		if(MPU9250_Read_Reg(PWR_MGMT_1)!=0x00) mpu_value_f.state=0;

		MPU9250_Write_Reg(CONFIG, 0x03);      //低通滤波频率，典型值：0x03(1000Hz)此寄存器内决定Internal_Sample_Rate==8K
		delay(10);
		if(MPU9250_Read_Reg(CONFIG)!=0x03) mpu_value_f.state=0;

		MPU9250_Write_Reg(INT_ENABLE,0x00);
		delay(10);
		if(MPU9250_Read_Reg(INT_ENABLE)!=0x00) mpu_value_f.state=0;

		MPU9250_Write_Reg(MOT_DETECT_CTRL,0x00);
		delay(10);
		if(MPU9250_Read_Reg(MOT_DETECT_CTRL)!=0x00) mpu_value_f.state=0;

		MPU9250_Write_Reg(PWR_MGMT_2,0x00);
		delay(10);
		if(MPU9250_Read_Reg(PWR_MGMT_2)!=0x00) mpu_value_f.state=0;
		/**********************Init SLV0 i2c**********************************/
		//Use SPI-bus read slave0
		MPU9250_Write_Reg(INT_PIN_CFG ,0x30);// INT Pin / Bypass Enable Configuration
		delay(10);
		if(MPU9250_Read_Reg(INT_PIN_CFG)!=0x30) mpu_value_f.state=0;

		MPU9250_Write_Reg(I2C_MST_CTRL,0x4d);//I2C MAster mode and Speed 400 kHz
		delay(10);
		if(MPU9250_Read_Reg(I2C_MST_CTRL)!=0x4d) mpu_value_f.state=0;

		MPU9250_Write_Reg(USER_CTRL ,0x20); // I2C_MST _EN
		delay(10);
		if(MPU9250_Read_Reg(USER_CTRL)!=0x20) mpu_value_f.state=0;

		MPU9250_Write_Reg(I2C_MST_DELAY_CTRL ,0x01);//延时使能I2C_SLV0 _DLY_ enable
		delay(10);
		if(MPU9250_Read_Reg(I2C_MST_DELAY_CTRL)!=0x01) mpu_value_f.state=0;

		MPU9250_Write_Reg(I2C_SLV0_CTRL ,0x81); //enable IIC	and EXT_SENS_DATA==1 Byte
		delay(10);
		if(MPU9250_Read_Reg(I2C_SLV0_CTRL)!=0x81) mpu_value_f.state=0;
		/*******************Init GYRO and ACCEL******************************/
		MPU9250_Write_Reg(SMPLRT_DIV, 0x09);  //陀螺仪采样率，典型值：0x09(100Hz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
		delay(10);
		if(MPU9250_Read_Reg(SMPLRT_DIV)!=0x09) mpu_value_f.state=0;

		MPU9250_Write_Reg(GYRO_CONFIG, 0x18); //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
		delay(10);
		if(MPU9250_Read_Reg(GYRO_CONFIG)!=0x18) mpu_value_f.state=0;

		MPU9250_Write_Reg(ACCEL_CONFIG, 0x18);//加速计自检、测量范围及高通滤波频率，典型值：0x18(不自检，16G)
		delay(10);
		if(MPU9250_Read_Reg(ACCEL_CONFIG)!=0x18) mpu_value_f.state=0;

		MPU9250_Write_Reg(ACCEL_CONFIG_2, 0x0B);//加速计高通滤波频率 典型值 ：0x08  （1.13kHz）
		delay(10);
		if(MPU9250_Read_Reg(ACCEL_CONFIG_2)!=0x0B) mpu_value_f.state=0;


		/**********************Init MAG **********************************/
		delay(1000);
		i2c_Mag_write(AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
		delay(1000);
		i2c_Mag_write(AK8963_CNTL1_REG,0x12); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output

	}
	else
	{
		mpu_value_f.state=0;
	}


}
void Init_MPU9250_Moving_Mode(void)
{
	MPU9250_Write_Reg(INT_ENABLE,0x40);
	MPU9250_Write_Reg(MOT_DETECT_CTRL,0xc0);
	MPU9250_Write_Reg(LP_ACCEL_ODR,0x03);
	MPU9250_Write_Reg(WOM_THR,0x7f);//中间值0.5g，在1~255之间（0~1020mg）
	MPU9250_Write_Reg(PWR_MGMT_2,0x07);
	MPU9250_Write_Reg(PWR_MGMT_1, 0x20);	//加速度低功耗模式
}

//************************加速度读取**************************/
void send_data(u8 data)
{
	while((UART5->SR&0X40)==0) {}
	USART_SendData(UART5,data);
}
void send_data_USART2(u8 data)
{
	while((USART2->SR&0X40)==0) {}
	USART_SendData(USART2,data);
}

void ahrs_send(u8 address,u8 BUF[8] )
{
	u8 suma=0;
	send_data(0x55);
	suma=0x55;
	send_data(address);
	suma+=address;
	send_data((BUF[0]));
	suma+=(BUF[0]);
	send_data((BUF[1]));
	suma+=(BUF[1]);
	send_data((BUF[2]));
	suma+=(BUF[2]);
	send_data((BUF[3]));
	suma+=(BUF[3]);
	send_data((BUF[4]));
	suma+=(BUF[4]);
	send_data((BUF[5]));
	suma+=(BUF[5]);
	send_data((BUF[6]));
	suma+=(BUF[6]);
	send_data((BUF[7]));
	suma+=(BUF[7]);
	send_data(suma);

}
void READ_MPU9250_ACCEL(void)//
{
	static u8 BUF[8];

#if 0
	static u8 ai=0;
	ai++;
	if(ai>10)
	{
		ai=0;
		ahrs_send(0x51,BUF);
	}
#endif

	//static u16 count=0;
	BUF[0]=MPU9250_Read_Reg(ACCEL_XOUT_L);
	BUF[1]=MPU9250_Read_Reg(ACCEL_XOUT_H);
	mpu_value.Accel[0]=((BUF[1]<<8))|BUF[0];
	mpu_value_f.Accel[0]=9.81*(mpu_value.Accel[0])/2048.0;//±16g
	if((acc_scale_zero.flag&0x00ff)==ACC_FLAG_FLASH)
		mpu_value_f.Accel[0]=(mpu_value_f.Accel[0]+acc_scale_zero.ox_offset)*acc_scale_zero.ax_gain;
	//mpu_value.Accel[0]/=2048.0; 						   //读取计算X轴数据
	BUF[2]=MPU9250_Read_Reg(ACCEL_YOUT_L);
	BUF[3]=MPU9250_Read_Reg(ACCEL_YOUT_H);
	mpu_value.Accel[1]=((BUF[3]<<8))|BUF[2];
	mpu_value_f.Accel[1]=9.81*(mpu_value.Accel[1])/2048.0;
	if((acc_scale_zero.flag&0x00ff)==ACC_FLAG_FLASH)
		mpu_value_f.Accel[1]=(mpu_value_f.Accel[1]+acc_scale_zero.oy_offset)*acc_scale_zero.ay_gain;
	//mpu_value.Accel[1]/=2048.0; 						   //读取计算Y轴数据
	BUF[4]=MPU9250_Read_Reg(ACCEL_ZOUT_L);
	BUF[5]=MPU9250_Read_Reg(ACCEL_ZOUT_H);
	mpu_value.Accel[2]=(((BUF[5]<<8))|BUF[4]);
	mpu_value_f.Accel[2]=9.81*(mpu_value.Accel[2])/2048.0;
	if((acc_scale_zero.flag&0x00ff)==ACC_FLAG_FLASH)
		mpu_value_f.Accel[2]=(mpu_value_f.Accel[2]+acc_scale_zero.oz_offset)*acc_scale_zero.az_gain;

}
/**********************陀螺仪读取*****************************/
void READ_MPU9250_GYRO(void)
{
	static u8 BUF[8];
#if 0
	static u16 ai=0;
	ai++;
	if(ai>10)
	{
		ai=0;
		ahrs_send(0x52,BUF);
	}
#endif
	BUF[0]=MPU9250_Read_Reg(GYRO_XOUT_L);
	BUF[1]=MPU9250_Read_Reg(GYRO_XOUT_H);
	mpu_value.Gyro[0]=(BUF[1]<<8)|BUF[0];
	mpu_value_f.Gyro[0]=PI/180.0*(mpu_value.Gyro[0])/16.4-gyro_scale_zero.x_offset;  //±2000°/s
	//mpu_value.Gyro[0]/=16.4; 						   //读取计算X轴数据
	BUF[2]=MPU9250_Read_Reg(GYRO_YOUT_L);
	BUF[3]=MPU9250_Read_Reg(GYRO_YOUT_H);
	mpu_value.Gyro[1]=(BUF[3]<<8)|BUF[2];
	mpu_value_f.Gyro[1]=PI/180.0*(mpu_value.Gyro[1])/16.4-gyro_scale_zero.y_offset;  //±2000°/s
	//mpu_value.Gyro[1]/=16.4; 						   //读取计算Y轴数据
	BUF[4]=MPU9250_Read_Reg(GYRO_ZOUT_L);
	BUF[5]=MPU9250_Read_Reg(GYRO_ZOUT_H);
	mpu_value.Gyro[2]=(BUF[5]<<8)|BUF[4];
	mpu_value_f.Gyro[2]=PI/180.0*(mpu_value.Gyro[2])/16.4-gyro_scale_zero.z_offset;  //±2000°/s
	//mpu_value.Gyro[2]/=16.4; 					       //读取计算Z轴数据

}


/**********************磁力计读取***************************/
//i2c_Mag_read(AK8963_ST2_REG) 此步读取不可省略
//数据读取结束寄存器，reading this register means data reading end
//AK8963_ST2_REG 同时具有数据非正常溢出检测功能
//详情参考 MPU9250 PDF
/**********************************************************/
void READ_MPU9250_MAG(void)
{
	static u8 BUF[8];
	u8 x_axis=0,y_axis=0,z_axis=0;
	u8 st1=0,st2=0;

	st1=i2c_Mag_read(AK8963_ST1_REG);
	st2=i2c_Mag_read(AK8963_ST2_REG);
	if((st1& AK8963_ST1_DOR)||(st2&AK8963_ST2_HOFL))
		return;
	x_axis=i2c_Mag_read(AK8963_ASAX);// X轴灵敏度调整值
	y_axis=i2c_Mag_read(AK8963_ASAY);
	z_axis=i2c_Mag_read(AK8963_ASAZ);
	//读取计算X轴数据
	BUF[0]=i2c_Mag_read(MAG_XOUT_L); //Low data
	BUF[1]=i2c_Mag_read(MAG_XOUT_H); //High data
	BUF[2]=i2c_Mag_read(MAG_YOUT_L); //Low data
	BUF[3]=i2c_Mag_read(MAG_YOUT_H); //High data
	//读取计算Z轴数据
	BUF[4]=i2c_Mag_read(MAG_ZOUT_L); //Low data
	BUF[5]=i2c_Mag_read(MAG_ZOUT_H); //High data
	mpu_value.Mag[0]=((int16_t)(BUF[1]<<8))|BUF[0];		//灵敏度纠正 公式见/RM-MPU-9250A-00 PDF/ 5.13
	mpu_value_f.Mag[0]=(float)mpu_value.Mag[0]*0.15*(((float)(x_axis-128)/256.0)+1);
	mpu_value.Mag[1]=((int16_t)(BUF[3]<<8))|BUF[2];
	mpu_value_f.Mag[1]=(float)mpu_value.Mag[1]*0.15*(((float)(y_axis-128)/256.0)+1);
	mpu_value.Mag[2]=((int16_t)(BUF[5]<<8))|BUF[4];
	mpu_value_f.Mag[2]=(float)mpu_value.Mag[2]*0.15*(((float)(z_axis-128)/256.0)+1);
}



