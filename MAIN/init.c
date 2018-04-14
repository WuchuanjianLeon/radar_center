#include "init.h"
#include "can.h"
#include "mpu_data.h"

/********************************************************************
***��������:AD_Init()
***����˵��:AD��ʼ��
***�������:��
***�������:
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

	ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;//���ٽ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ͨ��ɨ��
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//����ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת�����������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;//�������ͨ����Ŀ
	ADC_Init(ADC1,&ADC_InitStructure);//��ʼ��AD����
	ADC_Cmd(ADC1,ENABLE);//ʹ��AD
	ADC_ResetCalibration(ADC1);//����У׼����
	while(ADC_GetResetCalibrationStatus(ADC1));//�ȴ�У׼�����ȡ���
	ADC_StartCalibration(ADC1);//��ʼ����У׼
	while(ADC_GetCalibrationStatus(ADC1));//�ȴ���ȡADУ׼�������
	ADC_SoftwareStartConvCmd(ADC1,DISABLE);//�ر�ת��
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//iicģʽ
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//����ģʽ
	I2C_InitStructure.I2C_OwnAddress1 = 0x0010;//�豸��ַ
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//ʹ��Ӧ��λ
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//Ӧ��7λ��ַ
	I2C_InitStructure.I2C_ClockSpeed = 200000;//����ʱ��λ200k
	I2C_Init(I2C1,&I2C_InitStructure);//��ʼ������
	I2C_Cmd(I2C1,ENABLE);//ʹ��IIC
	I2C_AcknowledgeConfig(I2C1,ENABLE);//ʹ��Ӧ����

}
#endif

#ifdef SPI1_PERPH
static void SPI1_Init()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//ȫ˫��ģʽ
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//����Ϊ���豸
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//8λ���ݽṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//ʱ������Ϊ��
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//�ڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS�ܽ����ڲ�����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;//Ԥ��ƵֵΪ4��Ƶ
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//���ݴӵ�λ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;//7λѭ���������
	SPI_Init(SPI1,&SPI_InitStructure);//��ʼ������
	SPI_Cmd(SPI1,ENABLE);//ʹ��SPI1

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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//ȫ˫��ģʽ
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//����Ϊ���豸
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//8λ���ݽṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//ʱ������Ϊ��
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//�ڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS�ܽ����ڲ�����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//Ԥ��ƵֵΪ4��Ƶ
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//����
	SPI_InitStructure.SPI_CRCPolynomial = 7;//7λѭ���������
	SPI_Init(SPI2,&SPI_InitStructure);//��ʼ������
	SPI_Cmd(SPI2,ENABLE);//ʹ��SPI1
}
#endif

static void Usart1_Init(u32 Baud)
{
#ifdef USART1_PERPH
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//���ڷ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//���ڽ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;//�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//ʹ��
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate=Baud;//����������
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//����Ϊ8λ
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1λֹͣλ
	USART_InitStructure.USART_Parity=USART_Parity_No;//��У��λ
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//���ͺͽ���
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ����
	USART_Init(USART1,&USART_InitStructure);//��ʼ������
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//�����ڽ����ж�
	USART_Cmd(USART1,ENABLE);//ʹ������

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

	//GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//PA1����
	//GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//�������
	//GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	//GPIO_Init(GPIOA,&GPIO_InitStructure);
	//GPIO_ResetBits(GPIOA,GPIO_Pin_1);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//���ڷ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//���ڽ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;//�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//ʹ��
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate=Baud;//����������
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//����Ϊ8λ
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1λֹͣλ
	USART_InitStructure.USART_Parity=USART_Parity_No;//��У��λ
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//���ͺͽ���
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ����
	USART_Init(USART2,&USART_InitStructure);//��ʼ������
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//�����ڽ����ж�
	USART_Cmd(USART2,ENABLE);//ʹ������

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

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//���ڷ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;//���ڽ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;//�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//ʹ��
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate=Baud;//����������
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//����Ϊ8λ
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1λֹͣλ
	USART_InitStructure.USART_Parity=USART_Parity_No;//��У��λ
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//���ͺͽ���
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ����
	USART_Init(USART3,&USART_InitStructure);//��ʼ������
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//�����ڽ����ж�
	USART_Cmd(USART3,ENABLE);//ʹ������

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

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//���ڷ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;//���ڽ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=UART4_IRQn;//�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//ʹ��
	NVIC_Init(&NVIC_InitStructure);


	USART_InitStructure.USART_BaudRate=Baud;//����������
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//����Ϊ8λ
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1λֹͣλ
	USART_InitStructure.USART_Parity=USART_Parity_No;//��У��λ
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//���ͺͽ���
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ����
	USART_Init(UART4,&USART_InitStructure);//��ʼ������
	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);//�����ڽ����ж�
	USART_Cmd(UART4,ENABLE);//ʹ������

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
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;//���ڷ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//���ڽ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel=UART5_IRQn;//�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//ʹ��
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate=Baud;//����������
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//����Ϊ8λ
	USART_InitStructure.USART_StopBits=USART_StopBits_1;// 1λֹͣλ
	USART_InitStructure.USART_Parity=USART_Parity_No;//��У��λ
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//���ͺͽ���
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ����
	USART_Init(UART5,&USART_InitStructure);//��ʼ������
	USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);//�����ڽ����ж�
	USART_Cmd(UART5,ENABLE);//ʹ������

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//���ڷ�������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//�����������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//�ܽ�����
	GPIO_Init(GPIOC,&GPIO_InitStructure);

#endif
}
static void Time2_Pwm_in_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);      //ʱ������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_0|GPIO_Pin_1;                               //GPIO����
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel= TIM2_IRQn;                     //NVIC����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_1;                   //ͨ��ѡ��
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //�����ش���
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //�ܽ���Ĵ�����Ӧ��ϵ
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //����Ԥ��Ƶ����˼�ǿ����ڶ��ٸ�������
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //�˲����ã������������������϶������ȶ�0x0��0xF
	TIM_ICInit(TIM2,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_2;                   //ͨ��ѡ��
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //�����ش���
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //�ܽ���Ĵ�����Ӧ��ϵ
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //����Ԥ��Ƶ����˼�ǿ����ڶ��ٸ�������
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //�˲����ã������������������϶������ȶ�0x0��0xF
	TIM_ICInit(TIM2,&TIM_ICInitStructure);

	TIM_TimeBaseStructure.TIM_Period= 0xFFFF;	  //����0��FFFF
	TIM_TimeBaseStructure.TIM_Prescaler = TIME2_PWM_IDIV-1;		//ʱ�ӷ�Ƶ����Ƶ��Ϊ5+1��6��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	//ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;//ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);//������ʼ��

	TIM_Cmd(TIM2,ENABLE);                                 //����TIM2

	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2, ENABLE);     //���ж�
}

static void Time4_Pwm_in_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);      //ʱ������
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_6|GPIO_Pin_7;                               //GPIO����
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_8|GPIO_Pin_9;                               //GPIO����
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel= TIM4_IRQn;                     //NVIC����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_1;                   //ͨ��ѡ��
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //�����ش���
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //�ܽ���Ĵ�����Ӧ��ϵ
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //����Ԥ��Ƶ����˼�ǿ����ڶ��ٸ�������
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //�˲����ã������������������϶������ȶ�0x0��0xF
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_2;                   //ͨ��ѡ��
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //�����ش���
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //�ܽ���Ĵ�����Ӧ��ϵ
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //����Ԥ��Ƶ����˼�ǿ����ڶ��ٸ�������
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //�˲����ã������������������϶������ȶ�0x0��0xF
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_3;                   //ͨ��ѡ��
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //�����ش���
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //�ܽ���Ĵ�����Ӧ��ϵ
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //����Ԥ��Ƶ����˼�ǿ����ڶ��ٸ�������
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //�˲����ã������������������϶������ȶ�0x0��0xF
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel =TIM_Channel_4;                   //ͨ��ѡ��
	TIM_ICInitStructure.TIM_ICPolarity= TIM_ICPolarity_Rising;       //�����ش���
	TIM_ICInitStructure.TIM_ICSelection= TIM_ICSelection_DirectTI;    //�ܽ���Ĵ�����Ӧ��ϵ
	TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;           //����Ԥ��Ƶ����˼�ǿ����ڶ��ٸ�������
	TIM_ICInitStructure.TIM_ICFilter= 0x0f;                            //�˲����ã������������������϶������ȶ�0x0��0xF
	TIM_ICInit(TIM4,&TIM_ICInitStructure);



	//TIM_PWMIConfig(TIM3,&TIM_ICInitStructure);                 //���ݲ�������TIM������Ϣ

	TIM_TimeBaseStructure.TIM_Period= 0xFFFF;	  //����0��FFFF
	TIM_TimeBaseStructure.TIM_Prescaler = TIME4_PWM_IDIV-1;		//ʱ�ӷ�Ƶ����Ƶ��Ϊ5+1��6��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	//ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;//ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);//������ʼ��

//   TIM_SelectInputTrigger(TIM3,TIM_TS_TI2FP2);                //ѡ��IC2Ϊʼ�մ���Դ

//   TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);//TIM��ģʽ�������źŵ����������³�ʼ���������ʹ����Ĵ����ĸ����¼�

//   TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable); //������ʱ���ı�������

	TIM_Cmd(TIM4,ENABLE);                                 //����TIM2

	// TIM_ITConfig(TIM3,TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);     //���ж�

	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);     //���ж�
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = Time_Div;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//
	TIM_TimeBaseStructure.TIM_Period = Time_Period;//
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x01;//
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //����ģʽΪPWM2ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������Ϊ��
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;//1/2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = Time_Div;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//
	TIM_TimeBaseStructure.TIM_Period = Time_Period;//
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x01;//
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //����ģʽΪPWM2ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������Ϊ��
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

	TIM_TimeBaseStructure.TIM_Prescaler = Time_Div;//Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseStructure.TIM_Period = Time_Period;//�Զ�װ������
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x01;//�ظ�����ֵ
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ��
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //
	NVIC_Init(&NVIC_InitStructure);  //
	TIM_Cmd(TIM3, ENABLE);//ʹ��time1

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
#if CAN_RX0_INT_ENABLE //can�ж�ʹ��
	NVIC_InitTypeDef		NVIC_InitStructure;
#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��
	CAN_DeInit(CAN1);//����CAN1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO
	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE; 		//��ʱ�䴥��ͨ��ģʽ
	CAN_InitStructure.CAN_ABOM=DISABLE; 		//����Զ����߹���
	CAN_InitStructure.CAN_AWUM=DISABLE; 		//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=DISABLE;			//��ֹ�����Զ�����
	CAN_InitStructure.CAN_RFLM=DISABLE; 		//���Ĳ�����,�µĸ��Ǿɵ�
	CAN_InitStructure.CAN_TXFP=DISABLE; 		//���ȼ��ɱ��ı�ʶ������
	CAN_InitStructure.CAN_Mode= mode;			//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ;
	//���ò�����
	CAN_InitStructure.CAN_SJW=tsjw; 			//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ	CAN_SJW_1tq  CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2; 			//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;		//��Ƶϵ��(Fdiv)Ϊbrp+1
	CAN_Init(CAN1, &CAN_InitStructure); 		//��ʼ��CAN1

	CAN_FilterInitStructure.CAN_FilterNumber=0; //������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//����λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//32λ��
	CAN_FilterInitStructure.CAN_FilterIdHigh=(AIRC_CAN_ID<<5);	//32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0

	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��

#if CAN_RX0_INT_ENABLE
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	  // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			  // �����ȼ�Ϊ0
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

//	Can1_init(CAN_SJW_1tq,CAN_BS2_1tq,CAN_BS1_6tq,9,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������500Kbps   36M/((8+9+1)*4)
	//TIM_SetCompare4(TIM2,143);
	//Init_MPU9250();
	//TIM_SetCompare1(TIM3,(1500));
	//TIM_SetCompare2(TIM3,(1500));
	//TIM_SetCompare3(TIM3,(1500));
	//TIM_SetCompare4(TIM3,(1500));
	//delay(1000);

}

