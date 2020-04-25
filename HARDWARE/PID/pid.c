#include "main.h"


enum PId_Argument
{
	 Angle_kp =60 ,Angle_ki = 0, Angle_kd  =0
}; 

float Angle_err_sum=0,Angle_err_last=0;
float Angle_control(float ex_angle,float act_angle)
{
	double adjust=0.f,err=0.f;
	err=ex_angle-act_angle;
	if(err> 180)	 err-=360;
	if(err<-180)   err+=360;
	Angle_err_sum+=err;
	adjust=Angle_kp * err + Angle_ki*Angle_err_sum +Angle_kd*(err-Angle_err_last);
	Angle_err_last=err;
	return adjust;
}


/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
	PID_TypeDef * pid, 
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	float deadband,
	uint16_t period,
	long int  max_err,
	long int  target,

	float 	kp, 
	float 	ki, 
	float 	kd)
{
	pid->id = id;		
	
	pid->ControlPeriod = period;             //没用到
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

/*中途更改参数设定--------------------------------------------------------------*/
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*pid计算-----------------------------------------------------------------------*/

	
static float pid_calculate(PID_TypeDef* pid, float measure)//, int16_t target)
{
	pid->measure = measure;
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	pid->err = pid->target - pid->measure;
	
	//是否进入死区
	if((abs(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		//积分是否超出限制
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		//pid输出和
		pid->output = pid->pout + pid->iout + pid->dout;
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	}

	return pid->output;
}

/*pid结构体初始化，每一个pid参数需要调用一次-----------------------------------------------------*/
void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}



int left_speed_control(int ex,int act)
{
	
	static float 	u3_kp =0.525;
	static float  u3_ki =0.012;
	static float  u3_kd =0;

	float err=0.f;
	float adjust=0.f;
	static float err_last=0;
	static float err_sum=0.f;
	err=ex-act;
	err_sum+=err;
	adjust=u3_kp*err+u3_ki*err_sum+u3_kd*(err-err_last);
	err_last=err;
	if(adjust>1900) adjust=1900;
	if(adjust<1170) adjust=1170;
	return  (int)adjust;
	
}

int right_speed_control(int ex,int act)
{
	static float 	u3_kp =0.525;
	static float  u3_ki =0.012;
	static float  u3_kd =0;

	int err=0;
	float adjust=0.f;
	static int err_last=0;
	static float err_sum=0.f;
	err=ex-act;
	//if(err>-1000 )  
	err_sum+=err;
	adjust=u3_kp*err+u3_ki*err_sum+u3_kd*(err-err_last);
	err_last=err;
	if(adjust>1900) adjust=1900;
	if(adjust<1170) adjust=1170;

	return (int)adjust;
	
}


#define speed_203_kp 3
#define speed_203_ki 0
#define speed_203_kd 0
int speed_203_control(int ex,int act)
{
	float adjust,err;
	static  int err_last=0,err_sum=0;
	err=ex-act;
	err_sum+=err;
	adjust=speed_203_kp*err +speed_203_ki*err_sum+speed_203_kd*(err-err_last);
	err_last=err;
	if(adjust>8000)  adjust= 8000;
	if(adjust<-8000) adjust=-8000;
	return (int)adjust;

}
#define speed_204_kp 3
#define speed_204_ki 0
#define speed_204_kd 0
int speed_204_control(int ex,int act)
{
	float adjust,err;
	static  int err_last=0,err_sum=0;
	err=ex-act;
	err_sum+=err;
	adjust=speed_204_kp*err +speed_204_ki*err_sum+speed_204_kd*(err-err_last);
	err_last=err;
	if(adjust>8000)  adjust= 8000;
	if(adjust<-8000) adjust=-8000;
	return (int)adjust;

}



void s_pid_init(speed_pid* pid,float kp,float ki,float kd)
{
	pid->kd=kp;
	pid->ki=ki;
	pid->kd=kd;
	
	pid->max_out=1800;
	pid->max_iout=1900;
	pid->max_err=5000;
	pid->dead_band=20;
	pid->min_out=1170;
//	pid->max_err_sum=4000;
}

int s_pid_calc(speed_pid* pid, int target,int measure)
{
	pid->target=target;
	pid->measure=measure;
	pid->err=pid->target-pid->measure;
	
	if(pid->err>pid->max_err)           //误差极大限制
		pid->err=pid->measure;
	if(pid->err<-pid->max_err)
		pid->err=-pid->measure;
	
	if(abs(pid->err)>pid->dead_band)          //不在死区
	{
			pid->p_out=pid->kp*pid->err;       //计算比例项
		
			if(abs(pid->err_sum)>pid->max_iout/pid->ki)   //超出积分最大值只累加相反的误差
			{
				if(pid->err_sum>0)
				{
					if(pid->err<0)
					pid->err_sum+=pid->err;
				}
				else
				{
					if(pid->err>0)
					pid->err_sum+=pid->err;
				}
			}
			else
			{
				pid->err_sum+=pid->err;
			}
		
			pid->i_out=pid->ki*pid->err_sum;
		
		if(pid->i_out>pid->max_iout)
			pid->i_out=pid->max_iout;
		if(pid->i_out<-pid->max_iout)
			pid->i_out=-pid->max_iout;
		
		pid->d_out=pid->kd*(pid->err-pid->err_last);
		pid->err_last=pid->err;
		
		pid->out=(int)(pid->p_out+pid->i_out+pid->d_out);
		if(pid->out>pid->max_out)
			pid->out=pid->max_out;
		if(pid->out<pid->min_out)
			pid->out=pid->min_out;
		
	}
	
	return pid->out;
	
}

