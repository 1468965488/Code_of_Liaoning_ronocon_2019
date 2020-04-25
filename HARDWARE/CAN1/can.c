#include "main.h"

struct __motor_feedback motor_feedback; 
extern struct _motor_feedback Motor_feedback;

extern PID_TypeDef rm3508_pid[3];

int first_205_angle=0;
int first_206_angle=0;

u8 CAN1_Transmit_Success = 0;

long int sum_2006_angle_205=0;
long int sum_2006_angle_206=0;

void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);



    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
	//gpio.GPIO_OType = GPIO_OType_PP;//????
	//	gpio.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	//	gpio.GPIO_PuPd = GPIO_PuPd_UP;//??
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
	
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1  = CAN_BS1_9tq;
    can.CAN_BS2  = CAN_BS2_4tq;
    can.CAN_Prescaler = 3; 					  //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

		can_filter.CAN_FilterNumber=0;
		can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
		can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
		can_filter.CAN_FilterIdHigh=0x0000;
		can_filter.CAN_FilterIdLow=0x0000;
		can_filter.CAN_FilterMaskIdHigh=0x0000;
		can_filter.CAN_FilterMaskIdLow=0x0000;
		can_filter.CAN_FilterFIFOAssignment=0;
		can_filter.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
		{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
		CAN1_Transmit_Success = 1;
}

void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
		{
       CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
       CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
				
			 if(rx_message.StdId == 0x201)
			 {
					motor_feedback.rm3508_201_agree = (short)( (int)rx_message.Data[0] << 8 | rx_message.Data[1]);
					motor_feedback.rm3508_201_speed = (short)( (int)rx_message.Data[2] << 8 | rx_message.Data[3]);
					motor_feedback.rm3508_201_temp  = (short)((int)rx_message.Data[6]);
			 }
			 if(rx_message.StdId == 0x202)
			 {
					motor_feedback.rm3508_202_agree = (short)( (int)rx_message.Data[0] << 8 | rx_message.Data[1]);
					motor_feedback.rm3508_202_speed = (short)( (int)rx_message.Data[2] << 8 | rx_message.Data[3]);
					motor_feedback.rm3508_202_temp  = (short)((int)rx_message.Data[6]);
			 }
			  
			 if(rx_message.StdId == 0x203)
			 {
					Motor_feedback.rm2006_205_agree = (short)( (int)rx_message.Data[0] << 8 | rx_message.Data[1]);
					Motor_feedback.rm2006_205_speed = (short)( (int)rx_message.Data[2] << 8 | rx_message.Data[3]);
			 }
			 	if(rx_message.StdId == 0x204)
			 {
					Motor_feedback.rm2006_206_agree = (short)( (int)rx_message.Data[0] << 8 | rx_message.Data[1]);
					Motor_feedback.rm2006_206_speed = (short)( (int)rx_message.Data[2] << 8 | rx_message.Data[3]);
			 }
			 
 			 if(first_205_angle==0 && Motor_feedback.rm2006_205_agree >1)  first_205_angle=Motor_feedback.rm2006_205_agree;
			 if(first_206_angle==0 && Motor_feedback.rm2006_206_agree >1)  first_206_angle=Motor_feedback.rm2006_206_agree;

			 get_total_angle_();
		}

}

u8 Transmit_Motor_Data() 
{
    CanTxMsg tx_message;
 
		tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

		tx_message.Data[0] = ((short)rm3508_pid[0].output >> 8);
    tx_message.Data[1] =  (short)rm3508_pid[0].output;
	  
    tx_message.Data[2] = ((short)rm3508_pid[1].output >> 8);
		tx_message.Data[3] =  (short)rm3508_pid[1].output ;
	
		tx_message.Data[4] = ((short)motor_pid[4].output >> 8);
		tx_message.Data[5] =  (short)motor_pid[4].output ;

		tx_message.Data[6] = ((short)motor_pid[5].output >> 8);
		tx_message.Data[7] =  (short)motor_pid[5].output ;
	
		CAN1_Transmit_Success = 0;
	
		CAN_Transmit(CAN1,&tx_message);
//		while( !CAN1_Transmit_Success );

		return 0;
}





  /**
  ******************************************************************************
  * @file		void get_total_angle()
  * @brief  ÀÛ¼Ó±àÂëÆ÷Êý¾Ý
  ******************************************************************************
  */
	
	
void get_total_angle_()
{
	int r1,r2,r3,r4;
	int date1,date2;
	static int last_205_angle,current_205_angle,last_206_angle,current_206_angle;
	
	current_205_angle=Motor_feedback.rm2006_205_agree-first_205_angle;
	current_206_angle=Motor_feedback.rm2006_206_agree-first_206_angle;
		
	
	  if(last_205_angle>current_205_angle)
	{
		 r1=current_205_angle+8192-last_205_angle;
		 r2=current_205_angle-last_205_angle;
	}
	else
	{
		 r1=current_205_angle-8192-last_205_angle;
		 r2=current_205_angle-last_205_angle;
	}
	
		if(last_206_angle>current_206_angle)
	{
		 r3=current_206_angle+8192-last_206_angle;
		 r4=current_206_angle-last_206_angle;
		
	}
	else
	{
		 r3=current_206_angle-8192-last_206_angle;
		 r4=current_206_angle-last_206_angle;
		
	}
	

	
	 
	if(abs(r1)>abs(r2))
	{
		date1=r2;
		
	}
	else
	{
		date1=r1;
		
	}
	
	  if(abs(r3)>abs(r4))
	{
		date2=r4;
		
	}
	else
	{
		date2=r3;
		
	}
	

	
	
	sum_2006_angle_205+=date1;
  last_205_angle=current_205_angle;
	
	sum_2006_angle_206+=date2;
  last_206_angle=current_206_angle;


	
}
