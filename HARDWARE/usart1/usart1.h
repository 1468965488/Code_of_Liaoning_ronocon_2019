#ifndef __USART1_H
#define __USART1_H
#include "stdio.h"	
#include "sys.h" 



//////////////////////////////////////////////////////////////////////////////////	 

#define USART1_REC_NUM  			100  	//�����������ֽ��� 200
extern u8 uart_byte_count;          //uart_byte_countҪС��USART_REC_LEN
extern u8 receive_str[USART1_REC_NUM];  


void PosConfig(void);//��λϵͳ��ʼ��

void uart1_init(u32 bound);
void uart3_init(u32 bound);
void uart1SendChars(u8 *str, u16 strlen);
void Update(void);
u8 Err_YorZ(void);


#endif


