#include "main.h"

#include "dbus.h" 

 /***************************usart1.c*****************************/
 /*-----USART1_RX-----PB7----*/ 
 //for D-BUS
  DBUS dbus;
  unsigned char dbus_buf[DBUS_BUF_SIZE];

 void USART3_Config(void)//�ĳ�f4�汾�Ĵ��ڣ����Ų���
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
     	while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE){}//�ȴ�DMA������ 

     DMA_DeInit(DMA1_Stream1);//ע����Stream5����Stream7
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
 
	USART_DeInit(UART5);  //��λ����3
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOBʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//ʹ��USART3ʱ��
	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; //GPIOB11��GPIOB10��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��GPIOB11����GPIOB10
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; //GPIOB11��GPIOB10��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��GPIOB11����GPIOB10

	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOB11����ΪUSART3
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOB10����ΪUSART3	  
	
	USART_InitStructure.USART_BaudRate = bound;//������һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(UART5, &USART_InitStructure); //��ʼ������3
	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//�����ж�   �����յ�����ʱ������ж� daun
		
	USART_Cmd(UART5, ENABLE);                    //ʹ�ܴ��� 
	
 
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

}
	
 
