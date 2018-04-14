#ifndef __ACC_CALIBRATION_H_
#define __ACC_CALIBRATION_H_
#include "stm32f10x.h"

#define MATRIX_SIZE 7
extern double m_result[MATRIX_SIZE];

u8 acc_calc(void);
void ResetMatrix(void);
void dis_play(u8 data);


#endif

