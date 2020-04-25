#include "main.h"

  /**
  ******************************************************************************
  * @file			main.c
  * @author		机电创新团队
  * @version	V1.0.3
  * @date			2019/7/30
  * @brief    
  ******************************************************************************
  * @attention u3_printf("A:%d;  AX:%d;  AY:%d;  X:%d;  Y:%d;  W:%d;\r\n",(int)original_angle,(int)xangle,(int)yangle,(int)original_x,(int)original_y,(int)w_z);
  ******************************************************************************
  */
PID_TypeDef motor_pid[6];
PID_TypeDef rm3508_pid[3];
PID_TypeDef motor_u3[2];
extern u32 running_time;
extern DBUS dbus;
extern unsigned char dbus_buf[DBUS_BUF_SIZE];
extern struct __motor_feedback motor_feedback; 
extern struct _motor_feedback Motor_feedback;

extern float chassic_speed;
extern u8 running_time_switch;
extern u8  stop_switch;            //判断停止的定时器
extern u16 stop_time;
extern u8  Correction_switch; //矫正所用的定时器
extern u16 Correction_time; 
extern u8 In_fire;             //正在发射   在逃逸时如果是正在发射时进入的逃逸，直接回发射

extern struct _color color;
u8 pink_in_repository=0;       //二级储藏室中的粉球数目
u8 good_in_repository=0;			 //二级储藏室中黑白球数目
u8 The_Most_Important_Thing =0;
u8 BOI_1=0;
u8 BOI_2=0;
u8 edge_g =0;      //是否在扫边的标志   0表示在走圆  1表示在扫边
int target[5][2]={0};  //框的坐标，

int pwm_channel_1;
int pwm_channel_2;
extern float laser_left;
extern float laser_right;
extern u8 zhangyucheng;
extern u8 rso_sunshine;
extern float real_dis_1;
int main(void)
{ 
	//车的状态，顺逆时针  矫正状态     抽签结果   发射状态   	
	u8 mode_id=0,state=0,cor_state=1,situation=0,fire_state=0,start=0,id=0;
	float pwm=1500.f;
	float pwm2=1500.f;
	Main_Init();
	
	while(1)
	{
//		BOI_1=1;
//		BOI_2=4;
//		The_Most_Important_Thing=0;                     //1表示红场，0 表示蓝场
		if(The_Most_Important_Thing==1)
		{
			target[1][0]=-2200;
			target[1][1]= 4600;
			target[2][0]= 2200;                    //分别为框的x坐标，y坐标
			target[2][1]= 4600;
			target[3][0]= 2200;
			target[3][1]=  200;
			target[4][0]=-2200;
			target[4][1]=  200;
		}
		else
		{
			target[1][0]= 2200;
			target[1][1]=  200;
			target[2][0]=-2200;
			target[2][1]=  200;
			target[3][0]=-2200;
			target[3][1]= 4600;
			target[4][0]= 2200;
			target[4][1]= 4600;
		}
		target[0][0]=0;
		target[0][1]=2400;
		mode_id =1;
		state=0;
		pwm_channel_1=1400;
		pwm_channel_2=1400;
		if(!left_diaplasis_sw)
			stop_switch=1;
		else
		{
			stop_switch=0;
			stop_time=0;
		}
		if(stop_time>100)
		{
			stop_switch=0;
			stop_time=0;
			start=1;
		}
		if(self_203_init() && self_204_init() && start)
		{		
			running_time_switch=1;          //开启运行时间定时器
			break;
		}
		
		key_scanf();
		LCD_DisplayNum(100,100,BOI_1,5,24,0);
		LCD_DisplayNum(100,130,BOI_2,5,24,0);
		LCD_DisplayNum(100,160,The_Most_Important_Thing,5,24,0);
		if(The_Most_Important_Thing==0)
			LCD_Fill_onecolor(0,240,260,320,BLUE);
		else
			LCD_Fill_onecolor(0,240,260,320,RED);
	}
	
	while(1) 
	{
			squeeze_ball();
			laser_refersh();
			TIM_SetCompare1(TIM3,pwm_channel_1);
			TIM_SetCompare2(TIM3,pwm_channel_2);

			Valar_Morghuils();

		LCD_DisplayNum(100,100,color.R,5,24,0);
		LCD_DisplayNum(100,130,color.G,5,24,0);
		LCD_DisplayNum(100,160,color.B,5,24,0);

		

if(mode_id==1)  				
{
	
	if(rso_sunshine<=2) 
		rainbow(5800,state);
	else
		de_ja_vu_5(state,5800);
			
		if(chassic_speed<340)
		{
			stop_switch=1;
		}
		else
		{
			stop_switch=0;
			stop_time=0;
		}
		if(stop_time>800)
		{
				mode_id=2;
		}
		if(come_and_get_your_love_2())
		//if(pink_in_repository>=1 && good_in_repository>=1)
			mode_id=3;
	}
if(mode_id==2)       //进入逃逸状态
{
		if(	Escape(state)==1)
		{
			stop_switch=0;
			stop_time=0;
			if(In_fire==1)
				mode_id=3;
			else
				mode_id=1;
		}
}
if(mode_id==3)       //进入矫正发射状态
{
//	if(cor_state==3)
//	LCD_DisplayN;um_color(155,20,999999,6,16,0,WHITE,BLUE);
	situation=codelf();
	In_fire=1;
	if(cor_state==1)
	{
//		if(situation>4 && rso_sunshine<=2)
//			  Ready_to_Endgame_4(state);   //这个函数会一直跑  第三个参数是红蓝场
//		else
		Ready_to_Endgame_3(state);
		if(left_diaplasis_sw==1 || right_diaplasis_ww==1)			//非两个行程开关同时触发
	{
		Correction_switch=0;
		Correction_time=0;
		if(chassic_speed<100)   //速度小于150
			stop_switch=1;
		else
		{
			stop_switch=0;
			stop_time=0;     //过程中装上别的车会进逃逸
		} 
		if(stop_time>800)
		{
			stop_time=0;
			cor_state=3;                  //没有复位成功，认为是撞上其它车了
		}

	}
	else  //行程开关触发了
	{
		Correction_switch=1;
		if(Correction_time>300)    //连续触发400毫秒
		cor_state=2;          
	}
}
	if(cor_state==2)            
	{
		stop_switch=0;
		stop_time=0;
		Correction_switch=0;     //先关掉定时器
		Correction_time=0;
		
		fire_state=I_Am_Champion(situation,state);
		if(fire_state==1)   //发射完成
		{
			In_fire=0;
			mode_id=1;
			cor_state=1;
		}
		else if(fire_state==2)
		{
			//In_fire=0;
			//mode_id=1;
			cor_state=3;       //被撞出可发射区域或被撞偏离角度太大
			//In_fire=0;         //清除因为在发射而不能进逃逸的标志位
		}

	}
	if(cor_state==3)
	{
		My_circle(0,2400,1560,5500,state);   //给一个可以走到死区的圆
			zhangyucheng=1;
		if(chassic_speed<200)   //速度小于150
			stop_switch=1;
		else
		{
			stop_switch=0;
			stop_time=0;     //过程中装上别的车会进逃逸
		} 
		if(stop_time>800)
		{		
			mode_id=2;                     //进入正常逃逸部分
		}
		if(corner(pos_x,pos_y))      //进入死区就回到第一步
			cor_state=1;
		
	}
	
	
}

//									if(mode_id==4)     //边走边射状态
//									{-2.093e-07*x*x*x + 0.001404 *x*x - 2.195 *x + 5289;

//									switch(fww_state)
//									{
//										case 0:
//										if(advanced_walking(4000,state)==1)
//										fww_state=1;
//										in_advance=1;
//										break;
//										
//										case 1:
//											
//											if(1==1)
//											{
//												fww_state=2;
//											}
//											else
//											{
//												mode_id=3;
//											}
//										break;
//											
//										case 2:
//											if(advanced_walking(4000,state)==1)
//												fww_state=3;
//											break;
//											
//										case 3:
//												in_advance=0;
//												mode_id=1;
//												motor_pid[1].target=0;
//												motor_pid[0].target=0;
//											break;
//									}


//										if(chassic_speed<340)
//										{
//											stop_switch=1;
//										}
//										else
//										{
//											stop_switch=0;
//											stop_time=0;
//										}
//										
//										if(stop_time>800)
//										{
//												mode_id=2;
//											
//										}


//									}


	}
	
}

//		M_Angle_control(0,2000);
//		LCD_DisplayNum_color(155,20,dbus.rc.ch0 ,6,16,0,WHITE,BLUE);
//		LCD_DisplayNum_color(155,40,dbus.rc.ch1 ,6,16,0,WHITE,BLUE);
//		LCD_DisplayNum_color(155,60,dbus.rc.ch2 ,6,16,0,WHITE,BLUE);
//		LCD_DisplayNum_color(155,80,dbus.rc.ch3 ,6,16,0,WHITE,BLUE);
//		LCD_DisplayNum_color(155,100,dbus.rc.s1 ,6,16,0,WHITE,BLUE);
//		LCD_DisplayNum_color(155,120,dbus.rc.s2 ,6,16,0,WHITE,BLUE);








































/*
Night gathers, and now my watch begins.          长夜将至，吾之守望自此始
It shall not end until my death.                 直至我生命的尽头
I shall take no wife, hold no lands,             我将不娶妻，不封爵
father no children.                              不养子
I shall wear no crowns and win no glory.         不戴上皇冠，不赢得荣耀
I shall live and die at my post.                 我将在我的位置上生存和死去
I am the sword in the darkness.                  我就是黑暗中的利剑
I am the watcher on the walls.                   我就是长城上的守望者
I am the fire that burns against the cold,       我就是抵御严寒的火焰
the light that brings the dawn,                  带来黎明的光芒
the horn that wakes the sleepers,                惊醒沉睡者的号角
the shield that guards the realms of men.        守护人民的坚盾
I pledge my life and honor to the Night's Watch, 我将我的生命和荣耀奉献给守夜人
for this night and all the nights to come.       今夜如是，夜夜如是
*/








































/*

         ___          ___          __________           __          __               ________                   _____                                   
         \  \        /  /         /  ______  \         |  |        |  |            /  ______  \                /  _  \                                                           
          \  \      /  /         |  /      \  |        |  |        |  |           /  /      \__|              /  / \  \                         
           \  \    /  /          | |        | |        |	|				 |  |           |  |                       /  /   \  \                    
            \  \  /  /           | |        | |        |  |        |  |           |  |                      /  /     \  \                  
             \  \/  /            | |        | |        |  |        |  |           |  \________             /  /       \  \               
              \    /             | |        | |        |  |        |  |            \ _______  \           /  /         \  \               
               |  |              | |        | |        |  |        |  |                     \  |         |  |___________|  |                
               |  |              | |        | |        |	|				 |	|	                    |  |         |   ___________   |                               
               |  |              | |        | |        |  |        |  |            __       |  |         |  |           |  |                   
               |  |              | \________/ |        |   \______/   |           |  \______/  |         |  |           |  |                          
               |  |               \__________/          \____________/             \ ________ /          |__|           |__|                             
	             一一                                                                                                                                         
*/


/*	                                                    我永远喜欢泠鸢yousa																																																																			
*/

