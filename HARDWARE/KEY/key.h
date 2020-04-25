#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//����������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

/*����ķ�ʽ��ͨ��ֱ�Ӳ����⺯����ʽ��ȡIO*/
#define KEY0 		      GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_6) //PE4
#define KEY1 		      GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_7)	//PE3 
#define KEY2 		      GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_8) //PE2
#define WK_UP 	      GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_9)	//PA0

#define left_diaplasis_sw  GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_12)
#define right_diaplasis_ww GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_13)
#define switch_203  GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_14)
#define switch_204  GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_15)


//#define switch_203 		  	GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11)
//#define switch_204			  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15)


#define basket_1   GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_1)
#define basket_2   GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_2)
#define basket_3   GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_3)
#define basket_4   GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_4)

/*���淽ʽ��ͨ��λ��������ʽ��ȡIO*/
/*
#define KEY0 		PEin(4)   //PE4
#define KEY1 		PEin(3)		//PE3 
#define KEY2 		PEin(2)		//P32
#define WK_UP 	PAin(0)		//PA0
*/


#define KEY0_PRES 	1
#define KEY1_PRES 	2
#define KEY2_PRES	  3
#define WKUP_PRES   4


void KEY_Init(void);	//IO��ʼ��
u8 KEY_Scan(u8);  		//����ɨ�躯��	
void key_scanf(void);

#endif
