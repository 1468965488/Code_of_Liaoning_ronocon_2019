#include <fast_math.h>
#include "main.h"
int min(int a,int b)
{
	return a>=b? b:a;
}
int max(int a,int b)
{
	return a>=b? a:b;
}
//浮点型
float f_min(float a,float b)
{
	return a>b? b:a;
}
float f_max(float a,float b)
{
	return a>b? a:b;
}
short temp_max(short a,short b,short c)
{
	return a>b? a>c? a:c :b>c? b:c; 
}
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


//事实证明，这个文件里的东西基本没什么用
