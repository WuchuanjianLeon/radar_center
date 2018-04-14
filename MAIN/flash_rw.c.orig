/**************************************************************
*Function:	STM32F103系列内部Flash读写操作

*Author: ValerianFan

*Date:	 2014/04/09

*E-Mail:	fanwenjingnihao@163.com

*Other:	 该程序不能直接编译运行，只包含了Flash读写操作

****************************************************************/

#include "flash_rw.h"


#include "stm32f10x_flash.h" //flash操作接口文件（在库文件中），必须要包含

#define STARTADDR 0x0803F800//(256-2)*1024 //0x08010000 //STM32F103RB 其他型号基本适用，未测试

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE; //Flash操作状态变量

/****************************************************************

*Name:	 ReadFlashNBtye

*Function:	从内部Flash读取N字节数据

*Input:	 ReadAddress：数据地址（偏移地址）ReadBuf：数据指针	ReadNum：读取字节数

*Output:	 读取的字节数

*Author: ValerianFan

*Date:	 2014/04/09

*E-Mail:	fanwenjingnihao@163.com

*Other:

****************************************************************/

int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum)

{

int DataNum = 0;

ReadAddress = (uint32_t)STARTADDR + ReadAddress;

while(DataNum < ReadNum)

{

*(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;

DataNum++;

}

return DataNum;

}

/****************************************************************

*Name:	 WriteFlashOneWord

*Function:	向内部Flash写入32位数据

*Input:	 WriteAddress：数据地址（偏移地址）WriteData：写入数据

*Output:	NULL

*Author: ValerianFan

*Date:	 2014/04/09

*E-Mail:	fanwenjingnihao@163.com

*Other:

****************************************************************/

u8 WriteFlashOneWord(uint32_t SWriteAddress,uint32_t EWriteAddress,uint32_t *WriteData)

{
uint32_t i=0,j=0; 
int32_t ReadNum=100;
uint8_t  ReadBuf[100];
uint32_t  Read_word_Buf[25];

ReadNum=ReadFlashNBtye(0,ReadBuf,ReadNum);
for(i=0;i<=(ReadNum-4);i+=4)
{
 Read_word_Buf[j]=(ReadBuf[i+3]<<24)+(ReadBuf[i+2]<<16)+(ReadBuf[i+1]<<8)+ReadBuf[i];
 j++;
}
for(j=SWriteAddress;j<=EWriteAddress;j++)
{
 Read_word_Buf[j]=WriteData[j-SWriteAddress];
}

FLASH_UnlockBank1();

FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

FLASHStatus = FLASH_ErasePage(STARTADDR);
for(i=0;i<25;i++)
{
  if(FLASHStatus == FLASH_COMPLETE)

   {

    FLASHStatus = FLASH_ProgramWord(STARTADDR + 4*i, Read_word_Buf[i]); //flash.c 中API函数

    //FLASHStatus = FLASH_ProgramWord(StartAddress+4, 0x56780000); //需要写入更多数据时开启

   //FLASHStatus = FLASH_ProgramWord(StartAddress+8, 0x87650000); //需要写入更多数据时开启

   }
  else
  	{ 
  	 FLASH_LockBank1();
	 return 1;
  	}
}
FLASH_LockBank1();
return 0;


}

