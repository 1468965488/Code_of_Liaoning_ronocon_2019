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

u8  running_time_switch=0;    // 运行时间定时器开关
u32 running_time=0;
u8 	escape_timer_switch=0;    //逃逸定时器开关
u16 escape_timer=0;
u8  Correction_switch=0;      //矫正所用的定时器
u16 Correction_time=0;
u8  stop_switch=0;            //判断停止的定时器
u16 stop_time=0;
u8  squeeze_switch=0;         //挤球步骤定时器
u16 squeeze_time=0;
u8  detect_switch=0;          //检测颜色去抖定时器
u16 detect_time=0;
u8  black_switch=0;        	  //检测颜色去抖定时器
u16 black_time=0;
u8  white_switch=0;        	  //检测颜色去抖定时器
u16 white_time=0;
u8  fire_switch_1=0;          //发射四周框的定时器
u16 fire_time_1=0;
u8  fire_switch_2=0;          //发射中间框的定时器
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
	
	TIM_TimeBaseStructure.TIM_Period=arr; 											//自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler=psc; 										//定时器分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 	//自动重载周期值
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数
	
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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);//TIM7时钟使能    
	
	//定时器TIM10初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE ); //使能指定的TIM7中断,允许更新中断
	TIM_Cmd(TIM10,ENABLE);//使能定时器7
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
}

extern float laser_right;
extern float laser_left;
//这个定时器用来各种计时功能
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) ///溢出中断
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
		
		if(running_time%6==0)   //定位系统数据每5毫秒返回一次，每5毫秒检测一次可以保证每次都有数据，不会因为检测速率过快数据失常
		{
			chassic_speed=100*sqrt((pos_x-last_x)*(pos_x-last_x)+(pos_y-last_y)*(pos_y-last_y));
			last_x=pos_x;
			last_y=pos_y;
		}
		
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //清除中断标志位
	}
	
}



//这个定时器用来计算发送can指令
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
					
					
					
					





