#ifndef __PID_H
#define __PID_H	 
#include "sys.h" 



float Angle_control(float ex_angle,float act_angle);

int  left_speed_control(int ex,int act);
int right_speed_control(int ex,int act);

int speed_203_control(int ex,int act);
int speed_204_control(int ex,int act);


typedef enum
{

	PID_Position,
	PID_Speed
	
}PID_ID;

typedef struct _PID_TypeDef
{
	PID_ID id;
	
	float target;							//Ŀ��ֵ
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;
	
	float   measure;					//����ֵ
	float   err;							//���
	float   last_err;      		//�ϴ����
	
	float pout;
	float iout;
	float dout;
	
	float output;						//�������
	float last_output;			//�ϴ����
	
	float MaxOutput;				//����޷�
	float IntegralLimit;		//�����޷�
	float DeadBand;			  //����������ֵ��
	float ControlPeriod;		//��������
	float  Max_Err;					//������
		
	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID������ʼ��
				   PID_ID id,
				   uint16_t maxOutput,
				   uint16_t integralLimit,
				   float deadband,
				   uint16_t controlPeriod,
					long int max_err,     
					long int  target,
				   float kp,
				   float ki,
				   float kd);
				   
	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid���������޸�
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //pid����
}PID_TypeDef;
void pid_init(PID_TypeDef* pid);

extern PID_TypeDef motor_pid[6];
extern PID_TypeDef rm3508_pid[3];
extern PID_TypeDef motor_u3[2];



typedef struct _Speed_PID{
	
	float kp;
	float ki;
	float kd;
	float ki_far;
	float ki_near;
	
	float p_out;
	float i_out;
	float d_out;
	
	float max_out;
	float min_out;
	float max_iout;
	
	int err;
	float err_sum;
	int err_last;
	
	int max_err;
	int measure;
	int target;
	
	int out;
	int dead_band;
	int i_bound;
	
	int max_err_sum;
	
	
}speed_pid;

void s_pid_init(speed_pid* pid,float kp,float ki,float kd);
int s_pid_calc(speed_pid* pid, int target,int measure);


#endif
