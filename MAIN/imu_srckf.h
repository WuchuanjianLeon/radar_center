#ifndef __IMU_SRCKF_H_
#define __IMU_SRCKF_H_
#include "stm32f10x.h"

extern float fRPY[3];
extern uint32_t u32KFState;

void Ckf_main(void);
#endif
