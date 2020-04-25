#include "usart1.h"
#include "string.h"
#include "stdlib.h"  
#include "led.h" 
#include "math.h"



u8 receive_str[USART1_REC_NUM];     //接收缓存数组,最大USART_REC_LEN个字节 
u8 uart_byte_count=0;


void uart1_init(u32 bound)
{   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);  //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);          //初始化PA9，PA10
   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);         //开启相关中断
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;      //串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		   //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			   //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	  //根据指定的参数初始化VIC寄存器、
}

//串口1发送一个字符
void uart1SendChar(u8 ch)
{      
	while((USART1->SR&0x40)==0);  
    USART1->DR = (u8) ch;      
}
/****************************************************************************
* 名    称: void uart1SendChars(u8 *str, u16 strlen)
* 功    能：串口1发送一字符串
* 入口参数：*str：发送的字符串
            strlen：字符串长度
* 返回参数：无
* 说    明： 
****************************************************************************/
void uart1SendChars(u8 *str, u16 strlen)
{ 
	  u16 k= 0 ; 
   do { uart1SendChar(*(str + k)); k++; }   //循环发送,直到发送完毕   
    while (k < strlen); 
} 

//串口1中断服务程序





