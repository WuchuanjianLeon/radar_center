#ifndef __FLASH_RW_H_
#define __FLASH_RW_H_

#include "stm32f10x.h"

extern int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum);

extern u8 WriteFlashOneWord(uint32_t SWriteAddress,uint32_t EWriteAddress,uint32_t *WriteData);

#endif
