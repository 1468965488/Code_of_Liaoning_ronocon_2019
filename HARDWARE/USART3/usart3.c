#include "main.h"

#include "dbus.h" 

 /***************************usart1.c*****************************/
 /*-----USART1_RX-----PB7----*/ 
 //for D-BUS
  DBUS dbus;
  unsigned char dbus_buf[DBUS_BUF_SIZE];

 void USART3_Config(void)//改成f4版本的串口，引脚不对
 {
     USART_InitTypeDef usart1;
     GPIO_InitTypeDef  gpio;
     NVIC_InitTypeDef  nvic;
     DMA_InitTypeDef   dma;
     
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB| RCC_AHB1Periph_DMA1,ENABLE);
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
     
     GPIO_PinAFConfig(GPIOB,GPIO_PinSource11 ,GPIO_AF_USART3);
     gpio.GPIO_Pin = GPIO_Pin_11 ;
     gpio.GPIO_Mode = GPIO_Mode_AF;
     gpio.GPIO_OType = GPIO_OType_PP;
     gpio.GPIO_Speed = GPIO_Speed_100MHz;
     gpio.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL
     GPIO_Init(GPIOB,&gpio);
     USART_DeInit(USART3);
     usart1.USART_BaudRate = 100000;   //SBUS 100K baudrate
     usart1.USART_WordLength = USART_WordLength_8b;
     usart1.USART_StopBits = USART_StopBits_1;
     usart1.USART_Parity = USART_Parity_Even;
     usart1.USART_Mode = USART_Mode_Rx;
     usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
     USART_Init(USART3,&usart1);
     USART_Cmd(USART3,ENABLE);
     USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
     nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;
     nvic.NVIC_IRQChannelPreemptionPriority = 0;
     nvic.NVIC_IRQChannelSubPriority = 1;
     nvic.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&nvic);
     	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE){}//等待DMA可配置 

     DMA_DeInit(DMA1_Stream1);//注意是Stream5不是Stream7
     dma.DMA_Channel= DMA_Channel_4;
     dma.DMA_PeripheralBaseAddr = (u32)&(USART3->DR);//
     dma.DMA_Memory0BaseAddr = (u32)dbus_buf;
     dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
     dma.DMA_BufferSize = DBUS_BUF_SIZE;
     dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
     dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
     dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
     dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
     dma.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal DMA_Mode_Circular
     dma.DMA_Priority = DMA_Priority_VeryHigh;
     dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
     dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
     dma.DMA_MemoryBurst = DMA_Mode_Normal;//DMA_Mode_Normal DMA_MemoryBurst_Single
     dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
     DMA_Init(DMA1_Stream1,&dma);

     DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
     DMA_Cmd(DMA1_Stream1,ENABLE);
 }
 void DMA1_Stream1_IRQHandler(void)
 {
     if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1)!=RESET)
     {
         /*******************decode DBUS data*******************/
         DBUS_Dec(&dbus,dbus_buf);
         DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
         DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
     }
 }
     

 int zhengshu[3]={0};
struct _color color;
 void UART5_IRQHandler(void)
{
		static	uint8_t receive[11],ii=0;
	static u8 flag=0;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)//
	{	 
		receive[ii++]=USART_ReceiveData(UART5);
	if(receive[0]==0x5A)	
		flag=1;
	else
		ii=0;
	if(flag==1&&ii>6)
	{
			color.R=receive[4];
			color.G=receive[5];
			color.B=receive[6];
			ii=0;
			flag=0;
	  
	//	data_transform();
	}
  }	
	USART_ClearFlag(UART5,USART_FLAG_RXNE);//	

}

void usart5_config(u32 bound)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	USART_DeInit(UART5);  //复位串口3
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//使能USART3时钟
	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化GPIOB11，和GPIOB10
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化GPIOB11，和GPIOB10

	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOB10复用为USART3	  
	
	USART_InitStructure.USART_BaudRate = bound;//波特率一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART5, &USART_InitStructure); //初始化串口3
	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启中断   当接收到数据时候进入中断 daun
		
	USART_Cmd(UART5, ENABLE);                    //使能串口 
	
 
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

}
	
 
