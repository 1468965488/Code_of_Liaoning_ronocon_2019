#ifndef ADC_H
#define ADC_H
#include "main.h"
#include "sys.h"
void Adc1_Init(void);
u16 Get_Adc(u8 ch);
float get_adcaverage(u8 ch,u8 times);






#endif


