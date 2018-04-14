#include "init.h"
#include "can.h"
#include "mpu_data.h"

/********************************************************************
***函数名称:AD_Init()
***函数说明:AD初始化
***输入参数:无
***输出参数:
***
********************************************************************/
#define CAN_RX0_INT_ENABLE TRUE
//u16 pwm4=1500;
static void AD_Init()
{
#ifdef AD1_PERPH
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4 | GPIO_Pin_5 ;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4 | GPIO_Pin_5 ;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;//快速交替模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//多通道扫描
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//连续模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;//规则采样通道数目
	ADC_Init(ADC1,&ADC_InitStructure);//初始化AD采样
	ADC_Cmd(ADC1,ENABLE);//使能AD
	ADC_ResetCalibration(ADC1);//重置校准程序
	while(ADC_GetResetCalibrationStatus(ADC1));//等待校准程序获取完成
	ADC_StartCalibration(ADC1);//开始重新校准
	while(ADC_GetCalibrationStatus(ADC1));//等待获取AD校准程序完成
	ADC_SoftwareStartConvCmd(ADC1,DISABLE);//关闭转换
#endif
}
#ifdef IIC1_PERPH
static void IIC_Init()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//iic模式
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//快速模式
	I2C_InitStructure.I2C_OwnAddress1 = 0x0010;//设备地址
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//使能应答位
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//应答7位地址
	I2C_InitStructure.I2C_ClockSpeed = 200000;//设置时钟位200k
	I2C_Init(I2C1,&I2C_InitStructure);//初始化配置
	I2C_Cmd(I2C1,ENABLE);//使能IIC
	I2C_AcknowledgeConfig(I2C1,ENABLE);//使能应答功能

}
#endif

#ifdef SPI1_PERPH
static void SPI1_Init()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//设置为主设备
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//8位数据结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//时钟悬空为高
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS管脚由内部控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;//预分频值为4分频
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据从低位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;//7位循环冗余计算
	SPI_Init(SPI1,&SPI_InitStructure);//初始化配置
	SPI_Cmd(SPI1,ENABLE);//使能SPI1

}
#endif

#ifdef SPI2_PERPH
static void SPI2_Init()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |  GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//设置为主设备
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//8位数据结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//时钟悬空为高
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS管脚由内部控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//预分频值为4分频
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据
	SPI_InitStructure.SPI_CRCPolynomial = 7;//7位循环冗余计算
	SPI_Init(SPI2,&SPI_InitStructure);//初始化配置
	SPI_Cmd(SPI2,ENABLE);//使能SPI1
}
#endif

static void Usart1_Init(u32 Baud)
{
#ifdef USART1_PERPH
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//串口发送引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//串口接收引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//上拉输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;//串口中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//主优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;//从优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate=Baud;//波特率设置
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据为8位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1位停止位
	USART_InitStructure.USART_Parity=USART_Parity_No;//无校验位
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//发送和接收
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流
	USART_Init(USART1,&USART_InitStructure);//初始化串口
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//开串口接收中断
	USART_Cmd(USART1,ENABLE);//使能命令

#endif
}
static void Usart2_Init(u32 Baud)
{
#ifdef USART2_PERPH
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	//GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//PA1引脚
	//GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
	//GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	//GPIO_Init(GPIOA,&GPIO_InitStructure);
	//GPIO_ResetBits(GPIOA,GPIO_Pin_1);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//串口发送引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//串口接收引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//上拉输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;//串口中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//主优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;//从优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate=Baud;//波特率设置
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据为8位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1位停止位
	USART_InitStructure.USART_Parity=USART_Parity_No;//无校验位
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//发送和接收
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流
	USART_Init(USART2,&USART_InitStructure);//初始化串口
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//开串口接收中断
	USART_Cmd(USART2,ENABLE);//使能命令

#endif
}
static void Usart3_Init(u32 Baud)
{
#ifdef USART3_PERPH
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//串口发送引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;//串口接收引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//上拉输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;//串口中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//主优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//从优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate=Baud;//波特率设置
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据为8位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1位停止位
	USART_InitStructure.USART_Parity=USART_Parity_No;//无校验位
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//发送和接收
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流
	USART_Init(USART3,&USART_InitStructure);//初始化串口
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//开串口接收中断
	USART_Cmd(USART3,ENABLE);//使能命令

#endif
}

static void Uart4_Init(u32 Baud)
{
#ifdef USART4_PERPH
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//串口发送引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;//串口接收引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//上拉输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=UART4_IRQn;//串口中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//主优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//从优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能
	NVIC_Init(&NVIC_InitStructure);


	USART_InitStructure.USART_BaudRate=Baud;//波特率设置
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据为8位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1位停止位
	USART_InitStructure.USART_Parity=USART_Parity_No;//无校验位
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//发送和接收
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流
	USART_Init(UART4,&USART_InitStructure);//初始化串口
	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);//开串口接收中断
	USART_Cmd(UART4,ENABLE);//使能命令

#endif
}
static void Uart5_Init(u32 Baud)
{
#ifdef USART5_PERPH
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;//串口发送引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//串口接收引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//上拉输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=UART5_IRQn;//串口中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//主优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//从优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate=Baud;//波特率设置
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据为8位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1位停止位
	USART_InitStructure.USART_Parity=USART_Parity_No;//无校验位
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//发送和接收
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流
	USART_Init(UART5,&USART_InitStructure);//初始化串口
	USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);//开串口接收中断
	USART_Cmd(UART5,ENABLE);//使能命令

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//串口发送引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//管脚速率
	GPIO_Init(GPIOC,&GPIO_InitStructure);

#endif
}
static void Time2_Pwm_in_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);      //时钟配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_0|GPIO_Pin_1;                               //GPIO配置
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel= TIM2_IRQn;                     //NVIC配置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_1;                   //通道选择
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //上升沿触发
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //管脚与寄存器对应关系
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //输入预分频。意思是控制在多少个输入周
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF
	TIM_ICInit(TIM2,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_2;                   //通道选择
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //上升沿触发
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //管脚与寄存器对应关系
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //输入预分频。意思是控制在多少个输入周
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF
	TIM_ICInit(TIM2,&TIM_ICInitStructure);

	TIM_TimeBaseStructure.TIM_Period= 0xFFFF;	  //周期0～FFFF
	TIM_TimeBaseStructure.TIM_Prescaler = TIME2_PWM_IDIV-1;		//时钟分频，分频数为5+1即6分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	//时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;//模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);//基本初始化

	TIM_Cmd(TIM2,ENABLE);                                 //启动TIM2

	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2, ENABLE);     //打开中断
}

static void Time4_Pwm_in_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);      //时钟配置
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_6|GPIO_Pin_7;                               //GPIO配置
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_8|GPIO_Pin_9;                               //GPIO配置
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel= TIM4_IRQn;                     //NVIC配置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_1;                   //通道选择
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //上升沿触发
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //管脚与寄存器对应关系
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //输入预分频。意思是控制在多少个输入周
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_2;                   //通道选择
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //上升沿触发
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //管脚与寄存器对应关系
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //输入预分频。意思是控制在多少个输入周
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_3;                   //通道选择
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //上升沿触发
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //管脚与寄存器对应关系
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //输入预分频。意思是控制在多少个输入周
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_4;                   //通道选择
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //上升沿触发
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //管脚与寄存器对应关系
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //输入预分频。意思是控制在多少个输入周
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //滤波设置，经历几个周期跳变认定波形稳定0x0～0xF
	TIM_ICInit(TIM4,&TIM_ICInitStructure);



	//TIM_PWMIConfig(TIM3,&TIM_ICInitStructure);                 //根据参数配置TIM外设信息

	TIM_TimeBaseStructure.TIM_Period= 0xFFFF;	  //周期0～FFFF
	TIM_TimeBaseStructure.TIM_Prescaler = TIME4_PWM_IDIV-1;		//时钟分频，分频数为5+1即6分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	//时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;//模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);//基本初始化

//   TIM_SelectInputTrigger(TIM3,TIM_TS_TI2FP2);                //选择IC2为始终触发源

//   TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);//TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件

//   TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable); //启动定时器的被动触发

	TIM_Cmd(TIM4,ENABLE);                                 //启动TIM2

	// TIM_ITConfig(TIM3,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);     //打开中断

	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);     //打开中断
}

#ifdef TIME4_PWM
static void Time4_Pwm_Init(u16 Time_Div,u16 Time_Period)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = Time_Div;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//
	TIM_TimeBaseStructure.TIM_Period = Time_Period;//
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x01;//
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //脉冲模式为PWM2模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性为高
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //
	TIM_Cmd(TIM4, ENABLE);

}
#endif

static void Time3_Pwm_Init(u16 Time_Div,u16 Time_Period)
{
#ifdef TIME3_PWM
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;//3/4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;//1/2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = Time_Div;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//
	TIM_TimeBaseStructure.TIM_Period = Time_Period;//
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x01;//
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //脉冲模式为PWM2模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性为高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3 ,ENABLE);
	TIM_Cmd(TIM3, ENABLE);

#endif
}

#ifdef TIME3_PERPH
static void Time3_Init(u16 Time_Div,u16 Time_Period)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = Time_Div;//预分频值
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseStructure.TIM_Period = Time_Period;//自动装载周期
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x01;//重复计数值
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //
	NVIC_Init(&NVIC_InitStructure);  //
	TIM_Cmd(TIM3, ENABLE);//使能time1

}
#endif

void IWDG_Init(u8 prer,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(prer);
	IWDG_SetReload(rlr);
	IWDG_ReloadCounter();
	IWDG_Enable();
}
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}

static void Can1_init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef		GPIO_InitStructure;
	CAN_InitTypeDef 		CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;
#if CAN_RX0_INT_ENABLE //can中断使能
	NVIC_InitTypeDef		NVIC_InitStructure;
#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);//使能PORTA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟
	CAN_DeInit(CAN1);//重置CAN1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO
	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE; 		//非时间触发通信模式
	CAN_InitStructure.CAN_ABOM=DISABLE; 		//软件自动离线管理
	CAN_InitStructure.CAN_AWUM=DISABLE; 		//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=DISABLE;			//禁止报文自动传送
	CAN_InitStructure.CAN_RFLM=DISABLE; 		//报文不锁定,新的覆盖旧的
	CAN_InitStructure.CAN_TXFP=DISABLE; 		//优先级由报文标识符决定
	CAN_InitStructure.CAN_Mode= mode;			//模式设置： mode:0,普通模式;1,回环模式;
	//设置波特率
	CAN_InitStructure.CAN_SJW=tsjw; 			//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位	CAN_SJW_1tq  CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2; 			//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;		//分频系数(Fdiv)为brp+1
	CAN_Init(CAN1, &CAN_InitStructure); 		//初始化CAN1

	CAN_FilterInitStructure.CAN_FilterNumber=0; //过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//32位宽
	CAN_FilterInitStructure.CAN_FilterIdHigh=(AIRC_CAN_ID<<5);	//32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//激活过滤器0

	CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化

#if CAN_RX0_INT_ENABLE
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0消息挂号中断允许.

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	  // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			  // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}
void Init_All()
{
	SysTick_Config(SystemCoreClock / 1000);

	AD_Init();
	Time2_Pwm_in_Init();
	Time4_Pwm_in_Init();
	IWDG_Init(4,1000);

	//delay(1000);
#ifdef SPI1_PERPH
	SPI1_Init();
#endif
#ifdef SPI2_PERPH
	SPI2_Init();
#endif
	delay(2000);
	MPU9250_Read_Reg(WHO_AM_I);
	Init_MPU9250();

	Usart1_Init(BAND1);
	Usart2_Init(BAND2);
	Usart3_Init(BAND3);
	Uart4_Init(BAND4);
	Uart5_Init(BAND5);

	Time3_Pwm_Init(TIME3_DIV,TIME3_PRE);
	//Time3_Init(TIME3_DIV,TIME3_PRE);
	//Time4_Pwm_Init(TIME4_DIV,TIME4_PRE);

//	pwm_en_set;
	pwr_sw1_set;
	pwr_sw2_set;
	pwr_sw3_set;
	pwr_sw4_set;

//	Can1_init(CAN_SJW_1tq,CAN_BS2_1tq,CAN_BS1_6tq,9,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps   36M/((8+9+1)*4)
	//TIM_SetCompare4(TIM2,143);
	//Init_MPU9250();
	//TIM_SetCompare1(TIM3,(1500));
	//TIM_SetCompare2(TIM3,(1500));
	//TIM_SetCompare3(TIM3,(1500));
	//TIM_SetCompare4(TIM3,(1500));
	//delay(1000);

}

