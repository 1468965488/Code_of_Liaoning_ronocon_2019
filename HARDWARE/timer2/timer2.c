#include "main.h"
extern struct __motor_feedback motor_feedback; 
extern struct _motor_feedback Motor_feedback;

extern long int sum_2006_angle_201;
extern long int sum_2006_angle_202;
extern long int sum_2006_angle_203;
extern long int sum_2006_angle_204;
extern long int sum_2006_angle_205;
extern long int sum_2006_angle_206;

extern PID_TypeDef motor_pid[6];

u8  running_time_switch=0;    // ����ʱ�䶨ʱ������
u32 running_time=0;
u8 	escape_timer_switch=0;    //���ݶ�ʱ������
u16 escape_timer=0;
u8  Correction_switch=0;      //�������õĶ�ʱ��
u16 Correction_time=0;
u8  stop_switch=0;            //�ж�ֹͣ�Ķ�ʱ��
u16 stop_time=0;
u8  squeeze_switch=0;         //�����趨ʱ��
u16 squeeze_time=0;
u8  detect_switch=0;          //�����ɫȥ����ʱ��
u16 detect_time=0;
u8  black_switch=0;        	  //�����ɫȥ����ʱ��
u16 black_time=0;
u8  white_switch=0;        	  //�����ɫȥ����ʱ��
u16 white_time=0;
u8  fire_switch_1=0;          //�������ܿ�Ķ�ʱ��
u16 fire_time_1=0;
u8  fire_switch_2=0;          //�����м��Ķ�ʱ��
u16 fire_time_2=0;
u16 deadline_1=0;
u16 deadline_2=0;
u8  desert_switch=0;
u16 desert_time=0;

u8 choice_203=0;
u8 choice_204=0;

extern float pos_x;
extern float pos_y;
static float last_x=0.f,last_y=0.f;
float chassic_speed=0.f;


void TIM2_Int_Init(u16 arr,u16 psc)
{	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	
  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period=arr; 											//�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler=psc; 										//��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	//�Զ���������ֵ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2,ENABLE);
}

void TIM10_Int_Init(u16 arr,u16 psc)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);//TIM7ʱ��ʹ��    
	
	//��ʱ��TIM10��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM7�ж�,��������ж�
	TIM_Cmd(TIM10,ENABLE);//ʹ�ܶ�ʱ��7
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
}

extern float laser_right;
extern float laser_left;
//�����ʱ���������ּ�ʱ����
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) ///����ж�
	{
		if(running_time%500==0)
		LED0=!LED0;
		if(running_time_switch)  			running_time++;
		if(escape_timer_switch) 		  escape_timer++;
		if(Correction_switch)			  	Correction_time++;
		if(stop_switch)  							stop_time++;
		if(squeeze_switch) 						squeeze_time++;
		if(detect_switch)						  detect_time++;
		if(black_switch)              black_time++;
		if(white_switch)              white_time++;
		if(fire_switch_1)							{fire_time_1++;deadline_1++;}
		if(fire_switch_2)							{fire_time_2++;deadline_2++;}
		if(desert_switch)             desert_time++;
		
		if(running_time%6==0)   //��λϵͳ����ÿ5���뷵��һ�Σ�ÿ5������һ�ο��Ա�֤ÿ�ζ������ݣ�������Ϊ������ʹ�������ʧ��
		{
			chassic_speed=100*sqrt((pos_x-last_x)*(pos_x-last_x)+(pos_y-last_y)*(pos_y-last_y));
			last_x=pos_x;
			last_y=pos_y;
		}
		
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //����жϱ�־λ
	}
	
}



//�����ʱ���������㷢��canָ��
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET)
	{
				if(motor_feedback.rm3508_201_temp>70 || motor_feedback.rm3508_202_temp>70 || motor_feedback.rm3508_203_temp>70)
		{
				rm3508_pid[0].output=0;
				rm3508_pid[1].output=0;
				rm3508_pid[2].output=0;
		}
		else
		{
				rm3508_pid[0].f_cal_pid(&rm3508_pid[0],motor_feedback.rm3508_201_speed);
				rm3508_pid[1].f_cal_pid(&rm3508_pid[1],motor_feedback.rm3508_202_speed);
				rm3508_pid[2].f_cal_pid(&rm3508_pid[2],motor_feedback.rm3508_203_speed);
		}
		
			motor_pid[0].f_cal_pid(&motor_pid[0],sum_2006_angle_201);
			motor_pid[1].f_cal_pid(&motor_pid[1],sum_2006_angle_202);
			motor_pid[4].f_cal_pid(&motor_pid[4],sum_2006_angle_205);
			motor_pid[5].f_cal_pid(&motor_pid[5],sum_2006_angle_206);

		if(choice_203==1)
			motor_pid[2].f_cal_pid(&motor_pid[2],sum_2006_angle_203);
		else
			motor_pid[2].output=speed_203_control(1000,Motor_feedback.rm2006_203_speed);
		
		if(choice_204==1)
			motor_pid[3].f_cal_pid(&motor_pid[3],sum_2006_angle_204);
		else
			motor_pid[3].output=speed_204_control(-1000,Motor_feedback.rm2006_204_speed);

				Transmit_Motor_Data();
				Transmit_Motor_Data_can2();
				Transmit_Motor_Data_can_2();

		TIM_ClearITPendingBit(TIM10,TIM_IT_Update);
	}
}
					
					
					
					





