#include  "main.h"

unsigned int CAN_Time_Out = 0;

long int sum_2006_angle_201=0;
long int sum_2006_angle_202=0;
long int sum_2006_angle_203=0;
long int sum_2006_angle_204=0;


u8 CAN2_Transmit_Success = 0;

struct _motor_feedback Motor_feedback;
extern struct __motor_feedback motor_feedback; 

int first_201_angle=0;
int first_202_angle=0;

extern PID_TypeDef motor_pid[6];

//void position_init()
//{
//	if(first_201_angle==0)
//	{
//		first_201_angle=
//	}
//}
/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/

/*************************************************************************
                          CAN1_Configuration
描述：初始化CAN1配置为1M波特率
*************************************************************************/
void CAN2_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1|RCC_APB1Periph_CAN2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);
    
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
	
    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps      
    CAN_Init(CAN2, &can);

		can_filter.CAN_FilterNumber = 14;
		can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
		can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
		can_filter.CAN_FilterIdHigh = 0x0000;
		can_filter.CAN_FilterIdLow = 0x0000;
		can_filter.CAN_FilterMaskIdHigh = 0x0000;
		can_filter.CAN_FilterMaskIdLow = 0x0000;
		can_filter.CAN_FilterFIFOAssignment = 0;
		can_filter.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}

unsigned char can_tx_success_flag = 0;

void CAN2_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
		{
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
    }
		CAN2_Transmit_Success = 1;
}
/*************************************************************************
                          CAN1_TX_IRQHandler
描述：CAN1的发送中断函数
*************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
		{
       CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
       CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
				
			 if(rx_message.StdId == 0x201)
			 {
					Motor_feedback.rm2006_201_agree = (short)( (int)rx_message.Data[0] << 8 | rx_message.Data[1]);
					Motor_feedback.rm2006_201_speed = (short)( (int)rx_message.Data[2] << 8 | rx_message.Data[3]);
			 }
			 if(rx_message.StdId == 0x202)
			 {
					Motor_feedback.rm2006_202_agree = (short)( (int)rx_message.Data[0] << 8 | rx_message.Data[1]);
					Motor_feedback.rm2006_202_speed = (short)( (int)rx_message.Data[2] << 8 | rx_message.Data[3]);
			 }
			 if(rx_message.StdId == 0x203)
			 {
					Motor_feedback.rm2006_203_agree = (short)( (int)rx_message.Data[0] << 8 | rx_message.Data[1]);
					Motor_feedback.rm2006_203_speed = (short)( (int)rx_message.Data[2] << 8 | rx_message.Data[3]);
			 }
			 if(rx_message.StdId == 0x204)
			 {
					Motor_feedback.rm2006_204_agree = (short)( (int)rx_message.Data[0] << 8 | rx_message.Data[1]);
					Motor_feedback.rm2006_204_speed = (short)( (int)rx_message.Data[2] << 8 | rx_message.Data[3]);
			 }
			 if(rx_message.StdId == 0x205)
			 {
					motor_feedback.rm3508_203_agree = (short)( (int)rx_message.Data[0] << 8 | rx_message.Data[1]);
					motor_feedback.rm3508_203_speed = (short)( (int)rx_message.Data[2] << 8 | rx_message.Data[3]);
					motor_feedback.rm3508_203_temp  = (short)rx_message.Data[6];
			 }

			   get_total_angle();
			 
			 if(first_201_angle==0 && Motor_feedback.rm2006_201_agree >1000)  first_201_angle=Motor_feedback.rm2006_201_agree;
			 if(first_202_angle==0 && Motor_feedback.rm2006_202_agree >1000)  first_202_angle=Motor_feedback.rm2006_202_agree;


		}	
}

u8 Transmit_Motor_Data_can2() 
{
    CanTxMsg tx_message;
 
		tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

		
		tx_message.Data[0] = ((short)motor_pid[0].output >> 8);
    tx_message.Data[1] = (short)motor_pid[0].output;
	  
    tx_message.Data[2] = ((short)motor_pid[1].output >> 8);
		tx_message.Data[3] = (short)motor_pid[1].output ;
	
		tx_message.Data[4] = ((short)motor_pid[2].output >> 8);
    tx_message.Data[5] = (short)motor_pid[2].output;
	  
    tx_message.Data[6] = ((short)motor_pid[3].output >> 8);
		tx_message.Data[7] = (short)motor_pid[3].output;
	  
	
		CAN2_Transmit_Success = 0;
	
		CAN_Transmit(CAN2,&tx_message);
//		while( !CAN1_Transmit_Success );

		return 0;
}

u8 Transmit_Motor_Data_can_2() 
{
    CanTxMsg tx_message;
 
		tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
		
		tx_message.Data[0] = ((short)rm3508_pid[2].output >> 8);
    tx_message.Data[1] = (short)rm3508_pid[2].output;
	  
		CAN2_Transmit_Success = 0;
	
		CAN_Transmit(CAN2,&tx_message);
//		while( !CAN1_Transmit_Success );

		return 0;
}





  /**
  ******************************************************************************
  * @file		void get_total_angle()
  * @brief  累加编码器数据
  ******************************************************************************
  */
	
	
void get_total_angle()
{
	int r1,r2,r3,r4,r5,r6,r7,r8;
	int date1,date2,date3,date4;
	static int last_201_angle,current_201_angle,last_202_angle,current_202_angle,
             last_203_angle,current_203_angle,last_204_angle,current_204_angle;
	
  current_201_angle=Motor_feedback.rm2006_201_agree-first_201_angle;
	current_202_angle=Motor_feedback.rm2006_202_agree-first_202_angle;
	current_203_angle=Motor_feedback.rm2006_203_agree;
	current_204_angle=Motor_feedback.rm2006_204_agree;
	if(last_201_angle>current_201_angle)
	{
		 r1=current_201_angle+8192-last_201_angle;
		 r2=current_201_angle-last_201_angle;
		
	}
	else
	{
		 r1=current_201_angle-8192-last_201_angle;
		 r2=current_201_angle-last_201_angle;

	}
	
		if(last_202_angle>current_202_angle)
	{
		 r3=current_202_angle+8192-last_202_angle;
		 r4=current_202_angle-last_202_angle;
		
	}
	else
	{
		 r3=current_202_angle-8192-last_202_angle;
		 r4=current_202_angle-last_202_angle;
		
	}
		
	
	  if(last_203_angle>current_203_angle)
	{
		 r5=current_203_angle+8192-last_203_angle;
		 r6=current_203_angle-last_203_angle;
		
	}
	else
	{
		 r5=current_203_angle-8192-last_203_angle;
		 r6=current_203_angle-last_203_angle;
		
	}
	
		if(last_204_angle>current_204_angle)
	{
		 r7=current_204_angle+8192-last_204_angle;
		 r8=current_204_angle-last_204_angle;
		
	}
	else
	{
		 r7=current_204_angle-8192-last_204_angle;
		 r8=current_204_angle-last_204_angle;
		
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
	 
	if(abs(r5)>abs(r6))
	{
		date3=r6;
		
	}
	else
	{
		date3=r5;
		
	}
	
	  if(abs(r7)>abs(r8))
	{
		date4=r8;
		
	}
	else
	{
		date4=r7;
		
	}
	


	
	sum_2006_angle_201+=date1;
  last_201_angle=current_201_angle;
	                                      
	sum_2006_angle_202+=date2;
  last_202_angle=current_202_angle;
	
	sum_2006_angle_203+=date3;
  last_203_angle=current_203_angle;
	
	sum_2006_angle_204+=date4;
  last_204_angle=current_204_angle;


	
}

