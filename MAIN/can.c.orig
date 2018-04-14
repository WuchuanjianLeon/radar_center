#include <stdio.h> 
#include <string.h>
#include "can.h"
#include "interrupt.h"
#include "commander.h"
ST_Can_MSG_STATE_Type Can_Msg_Send;
ST_Can_MSG_STATE_Type Can_Msg_Receive;

ST_Can_MSG_Type Can_Msg_Tx;
ST_Can_MSG_Type Can_Msg_Rx;

ST_Can_MSGData_Type Can_Msg_Tx_Data;
ST_Can_MSGData_Type Can_Msg_Rx_Data;

u8 Can_Tx_Process_State;
u8 Can_Tx_Data[8];
u8 Can_Rx_Process_State;
u8 buf[300];

/*
@ can总线发送数据 stid为标识符 etid为扩展标识符 msg为数据 len为长度   这个长度不能超过8个字节
*/
u8 Can_Send(u8* msg,u32 stid,u8 len)
{	
	u8 mbox=0;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=stid;			// 标准标识符 
	//TxMessage.ExtId=etid;			// 设置扩展标示符 
	TxMessage.IDE=CAN_Id_Standard; 	// 标准帧
	TxMessage.RTR=CAN_RTR_Data;		// 数据帧
	TxMessage.DLC=len;				// 要发送的数据长度
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];			          
	mbox=CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFF))i++;	//等待发送结束
	if(i>=0XFF)return 1;//超时判断
	return 0;	 
}
void Can_Tx_Load(u8 Data_Type,u8 Msg_ID,u8 Data_Size,u8 *Data)
{
	  Can_Msg_Send.Frame_Size = Data_Size + 14;
	  Can_Tx_Process_State = MSGHeader;
	  Can_Msg_Tx.Can_MsgHeader.ucFixed0 = CAN_MSG_FIX0;
	  Can_Msg_Tx.Can_MsgHeader.ucFixed1 = CAN_MSG_FIX1;
	  Can_Msg_Tx.Can_MsgHeader.ucLength = Data_Size + 9;
	  
	  if(Can_Msg_Tx.Can_MsgHeader.ucFrameSequence == 0xFF)
	    Can_Msg_Tx.Can_MsgHeader.ucFrameSequence = 1;
	  else
		Can_Msg_Tx.Can_MsgHeader.ucFrameSequence++;
	  
	  Can_Msg_Tx.Can_ID = RADAR_CAN_ID;
      Can_Msg_Tx.Can_MsgInfo.ucFrameID = Data_Type;
	  Can_Msg_Tx.Can_MsgInfo.ucMsgID = Msg_ID;
	  memcpy(Can_Msg_Tx.Can_MsgInfo.ucData,Data,Data_Size);
}



void Can_Tx_Pro(void)
{
	u8 i;
	static u8 Tx_Data_Num = 0;
    switch(Can_Tx_Process_State)
		{
			case MSGHeader:
				Can_Tx_Data[0] = CAN_MSG_FIX0;
			    Can_Tx_Data[1] = CAN_MSG_FIX1;
			    Can_Tx_Data[2] = (u8)(Can_Msg_Tx.Can_MsgHeader.ucLength >> 8);
			    Can_Tx_Data[3] = (u8)Can_Msg_Tx.Can_MsgHeader.ucLength;
			    Can_Tx_Data[4] = Can_Msg_Tx.Can_MsgHeader.ucFrameSequence;
			    Can_Tx_Data[5] = (u8)(Can_Msg_Tx.Can_ID >> 8);
			    Can_Tx_Data[6] = (u8)Can_Msg_Tx.Can_ID;
			    Can_Tx_Data[7] = (u8)(Can_Msg_Tx.Time_Stamp);
			    Can_Msg_Tx.Can_Checksum = Can_Tx_Data[2];
			    for(i = 3; i < 8; i++)
			     {
				     Can_Msg_Tx.Can_Checksum ^= Can_Tx_Data[i];
				 }
			    Can_Tx_Process_State = MSGLoad1;
			    break;
			case MSGLoad1:
				Can_Tx_Data[0] = (u8)(Can_Msg_Tx.Time_Stamp >> 8);
			    Can_Tx_Data[1] = (u8)(Can_Msg_Tx.Time_Stamp >> 16);
			    Can_Tx_Data[2] = (u8)(Can_Msg_Tx.Time_Stamp >> 24);
				Can_Tx_Data[3] = Can_Msg_Tx.Can_MsgInfo.ucFrameID;
				Can_Tx_Data[4] = Can_Msg_Tx.Can_MsgInfo.ucMsgID;
			    for(i = 0; i < 5; i++)
			     {
			         Can_Msg_Tx.Can_Checksum ^= Can_Tx_Data[i];
				 }
			    Tx_Data_Num = Can_Msg_Tx.Can_MsgHeader.ucLength - 9;
			    if(Tx_Data_Num < 3)
				 {
				      for(i = 0; i < Tx_Data_Num; i++)
			           {
					       Can_Tx_Data[i + 5] = Can_Msg_Tx.Can_MsgInfo.ucData[i];
						   Can_Msg_Tx.Can_Checksum ^= Can_Tx_Data[i + 5];
					   }
					  Can_Tx_Data[Tx_Data_Num + 5] = Can_Msg_Tx.Can_Checksum;
				      Tx_Data_Num = 0;
				      Can_Tx_Process_State = MSGHeader;
				 }
				else
				 {
					  for(i = 5; i < 8; i++)
		               {
				           Can_Tx_Data[i] = Can_Msg_Tx.Can_MsgInfo.ucData[i - 5];
					       Can_Msg_Tx.Can_Checksum ^= Can_Tx_Data[i];
				       }
					  Tx_Data_Num -= 3;
				      Can_Tx_Process_State = MSGLoad2;
				 }
			    break;
			case MSGLoad2:
				if(Tx_Data_Num > 7)
				 {
					  for(i = 0; i < 8; i++)
					   {
						   Can_Tx_Data[i] = Can_Msg_Tx.Can_MsgInfo.ucData[i + Can_Msg_Tx.Can_MsgHeader.ucLength \
								            - 9 - Tx_Data_Num];
						   Can_Msg_Tx.Can_Checksum ^= Can_Tx_Data[i];
					   }
					  Tx_Data_Num -= 8;
				 }
				else
				 {
				      for(i = 0; i < Tx_Data_Num; i++)
			           {
					       Can_Tx_Data[i] = Can_Msg_Tx.Can_MsgInfo.ucData[i + Can_Msg_Tx.Can_MsgHeader.ucLength \
								            - 9 - Tx_Data_Num];
						   Can_Msg_Tx.Can_Checksum ^= Can_Tx_Data[i];
					   }
					  Can_Tx_Data[Tx_Data_Num] = Can_Msg_Tx.Can_Checksum;
					  Tx_Data_Num = 0;
				 }
			    break;
			default:break;
		}
}
void Can_Tx(void)
{
   if(Can_Msg_Send.Frame_Size)
	{
	    Can_Tx_Pro();
		if(Can_Msg_Send.Frame_Size > 7)
	     {
		     Can_Msg_Send.Frame_Size -= 8;
	         Can_Send(Can_Tx_Data,RADAR_CAN_ID,8);
		 }
		else
		 {  
		     Can_Send(Can_Tx_Data,RADAR_CAN_ID,Can_Msg_Send.Frame_Size);
			 Can_Msg_Send.Start = 0;
             Can_Msg_Send.Over = 0;
			 Can_Msg_Send.Frame_Size = 0;

		 }
	}
   else
	{
	    Can_Msg_Send.Start = 0;
        Can_Msg_Send.Over = 0;
	}
}
void Can_Tx_First_Frame(u8 Data_Type,u8 Msg_ID,u8 Data_Size,u8 *Data)
{
	  Can_Msg_Send.Start = 1;
	  Can_Msg_Send.Over = 0;
      Can_Tx_Load(Data_Type,Msg_ID,Data_Size,Data);
	  Can_Tx();

}
void Can_Tx_Frame_Loop(u8 Data_Type,u8 Msg_ID,u8 Data_Size,u8 *Data)
{
  if(Can_Msg_Send.Over == 1&&Can_Msg_Send.Start ==0)
   {
        Can_Tx_First_Frame(Data_Type,Msg_ID,Data_Size,Data);
   }
  else if(Can_Msg_Send.Start ==1&&Can_Msg_Send.Over == 0)
   {
  	    Can_Tx();
   }
}

//can口接收数据查询
void Can_Rx_Pro(void)
{
	u8 i;
	static u8 Rx_Data_Num;
	static u8 Check_Sum;
	if(Can_In_Count == Can_Out_Count)
	 {
	  	   return;
	 }
    switch(Can_Rx_Process_State)
		{
			  case MSGHeader:
					  Can_Msg_Receive.time = 0;
					  if((Can_Rx_Data[Can_Out_Count][0] == CAN_MSG_FIX0) && (Can_Rx_Data[Can_Out_Count][1] == CAN_MSG_FIX1))
					   {
			              Can_Msg_Rx.Can_MsgHeader.ucLength = ((u16)Can_Rx_Data[Can_Out_Count][2] << 8) + Can_Rx_Data[Can_Out_Count][3];
						  Can_Msg_Rx.Can_MsgHeader.ucFrameSequence = Can_Rx_Data[Can_Out_Count][4];
		 				  Can_Msg_Rx.Can_ID = ((u16)Can_Rx_Data[Can_Out_Count][5] << 8) + Can_Rx_Data[Can_Out_Count][6];
						  Can_Msg_Rx.Time_Stamp = (u32)Can_Rx_Data[Can_Out_Count][7] << 24;
						  Check_Sum = Can_Rx_Data[Can_Out_Count][2];
						  for(i = 0; i < 5; i++)
						   {
							    Check_Sum ^= Can_Rx_Data[Can_Out_Count][i + 3];
						   }
						  Can_Rx_Process_State = MSGLoad1;
					   }
				      break;
			  case MSGLoad1:  
					  Can_Msg_Receive.time = 0;
					  for(i = 0; i < 3; i++)
				       {
					      Can_Msg_Rx.Time_Stamp >>= 8;
					      Can_Msg_Rx.Time_Stamp += ((u32)Can_Rx_Data[Can_Out_Count][i] << 24);
						  Check_Sum ^= Can_Rx_Data[Can_Out_Count][i];
                       }
					  Can_Msg_Rx.Can_MsgInfo.ucFrameID = Can_Rx_Data[Can_Out_Count][3];
					  Can_Msg_Rx.Can_MsgInfo.ucMsgID = Can_Rx_Data[Can_Out_Count][4];
				      Rx_Data_Num = Can_Msg_Rx.Can_MsgHeader.ucLength - 9;
				      Check_Sum ^= Can_Rx_Data[Can_Out_Count][3];
					  Check_Sum ^= Can_Rx_Data[Can_Out_Count][4];
				      if(Rx_Data_Num < 2)
					   {
						  for(i = 0; i < Rx_Data_Num; i++)
						   {
							   Can_Msg_Rx.Can_MsgInfo.ucData[i] = Can_Rx_Data[Can_Out_Count][i + 5];
							   Check_Sum ^= Can_Rx_Data[Can_Out_Count][i + 5];
						   }
						  Can_Msg_Rx.Can_Checksum = Can_Rx_Data[Can_Out_Count][Rx_Data_Num + 5];
						  Can_Rx_Process_State = MSGHeader;
						  Rx_Data_Num = 0;
						  if(Can_Msg_Rx.Can_Checksum == Check_Sum)
						   {
							   Can_Msg_Rx.Can_MsgHeader.ucLength -= 1;
							   memcpy(buf,&Can_Msg_Rx.Can_MsgHeader.ucLength,2);
							   Can_Msg_Rx.Can_MsgHeader.ucLength += 1;
							   memcpy(buf + 2,&Can_Msg_Rx.Time_Stamp,4);
							   memcpy(buf + 6,&Can_Msg_Rx.Can_MsgInfo.ucFrameID,Can_Msg_Rx.Can_MsgHeader.ucLength - 7);
						       //can_msg_fifo_rx_in(buf,Can_Msg_Rx.Can_MsgHeader.ucLength - 1);
							   Can_Msg_Receive.Over = 1;
						       //Led_Blink();//test
						   }
					   }
					  else
					   {
						  for(i = 0; i< 3; i++)
						   {
				               Can_Msg_Rx.Can_MsgInfo.ucData[i] = Can_Rx_Data[Can_Out_Count][i + 5];
							   Check_Sum ^= Can_Rx_Data[Can_Out_Count][i + 5];
						   }
						  Can_Rx_Process_State = MSGLoad2;
						  Rx_Data_Num -= 3;
					   }
				      break;

			  case MSGLoad2:
					  Can_Msg_Receive.time = 0;
					  if(Rx_Data_Num > 7)
					   {
						    for(i = 0; i< 8; i++)
							 {
				                 Can_Msg_Rx.Can_MsgInfo.ucData[i + Can_Msg_Rx.Can_MsgHeader.ucLength - \
									 9 - Rx_Data_Num] = Can_Rx_Data[Can_Out_Count][i];
								 Check_Sum ^= Can_Rx_Data[Can_Out_Count][i];
							 }
							Rx_Data_Num -= 8;
					   }
					  else
					   {
							for(i = 0; i < Rx_Data_Num; i++)
							 {
							     Can_Msg_Rx.Can_MsgInfo.ucData[i + Can_Msg_Rx.Can_MsgHeader.ucLength - \
							          9 - Rx_Data_Num] = Can_Rx_Data[Can_Out_Count][i];
							     Check_Sum ^= Can_Rx_Data[Can_Out_Count][i];
							 } 
							Can_Msg_Rx.Can_Checksum = Can_Rx_Data[Can_Out_Count][Rx_Data_Num];
							Can_Rx_Process_State = MSGHeader;
							Rx_Data_Num = 0;
							if(Can_Msg_Rx.Can_Checksum == Check_Sum)
							 {
							     Can_Msg_Rx.Can_MsgHeader.ucLength -= 1;
							     memcpy(buf,&Can_Msg_Rx.Can_MsgHeader.ucLength,2);
							     Can_Msg_Rx.Can_MsgHeader.ucLength += 1;
							     memcpy(buf + 2,&Can_Msg_Rx.Time_Stamp,4);
							     memcpy(buf + 6,&Can_Msg_Rx.Can_MsgInfo.ucFrameID,Can_Msg_Rx.Can_MsgHeader.ucLength - 7);
							     Can_Msg_Receive.Over = 1;
								 use_art=USE_USART4;
		                         printf("%f\n",((float)Can_Msg_Rx.Can_MsgHeader.ucFrameSequence));/*输出浮点数*/
							     if(Can_Msg_Rx.Can_MsgHeader.ucLength > 100)
								  {
								      Can_Msg_Rx.Can_MsgHeader.ucLength = Can_Msg_Rx.Can_MsgHeader.ucLength;
								  }
							 }
					    }
				       break;
				default:break;
		}
	Can_Out_Count ++;
    if(Can_Out_Count >= RX_FIFO_NUM)
       Can_Out_Count = 0;
}
void Can_Rx_Read_Data(void)
{
   Can_Rx_Pro();
}


