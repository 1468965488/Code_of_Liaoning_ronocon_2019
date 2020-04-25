#ifndef _ENC_H
#define _ENC_H
#include "main.h"

void TIM4_Mode_Config(void);
void Date_encoder_TIM4(void);
void Data_processing_TIM4(int enc);

void TIM5_In_Init(u16 arr,u16 psc);
void TIM5_IRQHandler(void);

void TIM8_Mode_Config(void);
void Date_encoder_TIM8(void);
void Data_processing_TIM8(int enc);



#endif

