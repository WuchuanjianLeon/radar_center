#ifndef __CAN_H
#define __CAN_H
#include "stm32f10x.h"
#define CAN_MSG_FIX0 0xAA
#define CAN_MSG_FIX1 0x0C
#define MSG_DATA_NUM 300
#define AIRC_CAN_ID 0x000E
#define RADAR_CAN_ID 0x000F

typedef struct
{
	u8 ucFixed0;  //
	u8 ucFixed1;
	u16 ucLength;
	u8 ucFrameSequence;
} ST_Can_MSGHeader_Type;
typedef struct
{
	u8 ucFrameID;
	u8 ucMsgID;
	u8 ucData[MSG_DATA_NUM];
} ST_Can_MSGInfo_Type;

typedef struct
{
	u8 Frame_Size;
	u8 Start;
	u8 Over;
	u8 fr_flag;
	u8 time;
} ST_Can_MSG_STATE_Type;
typedef struct
{
	ST_Can_MSGHeader_Type Can_MsgHeader;
	u16 Can_ID;
	u32 Time_Stamp;
	ST_Can_MSGInfo_Type Can_MsgInfo;
	u8 Can_Checksum;
} ST_Can_MSG_Type;
typedef struct
{
	u8 Size;
	u8 FrameID;
	u8 MsgID;
	u8 Data[MSG_DATA_NUM];
} ST_Can_MSGData_Type;
typedef enum
{
	MSGHeader,
	MSGLoad1,
	MSGLoad2,
	MSGLoad3
} Can_PROCESS_STATUS_Type;


extern ST_Can_MSG_STATE_Type Can_Msg_Send;
extern ST_Can_MSG_Type Can_Msg_Tx;
extern ST_Can_MSGData_Type Can_Msg_Tx_Data;
extern ST_Can_MSG_STATE_Type Can_Msg_Receive;

extern u8 buf[300];

extern u8 Can_Send(u8* msg,u32 stid, u8 len);
void Can_Rx_Read_Data(void);
extern void Can_Tx_Frame_Loop(u8 Data_Type,u8 Msg_ID,u8 Data_Size,u8 *Data);

#endif

