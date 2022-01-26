/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/09/12
程序功能：位置式PID控制算法
*******************************************************************************/
#include "pid.h"
#include "structconfig.h"

// 1 位置式PID算法 角度环与角速度环共用此函数
void PID_Postion_Cal(PID_TYPE *PID, float target, float measure) //target: 目标值, measure: 测量值
{
	PID->Error = target-measure; 			  //误差Ek=Sk-Xk
	PID->Differ = PID->Error - PID->PreError; //微分量 Ek-E(k-1)
	
	PID->Pout = PID->P * PID->Error;					   //比例控制Pout=Kp*Ek
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral; //积分控制Iout=(0/1)*Ki*(Sk*T) Sk:sum T:采样周期
	PID->Dout = PID->D * PID->Differ;					   //微分控制Dout=Kd*([Ek-E(k-1)]/T)
	
	PID->OutPut = PID->Pout + PID->Iout + PID->Dout; //总控制输出量 Uout
	
	if(Airplane_Enable==1 && RC_Control.THROTTLE>=180) //飞机解锁后再加入积分I，防止积分过调
	{
		if(measure>(PID->Ilimit) || measure<(-PID->Ilimit)) //积分分离
		{
			PID->Ilimit_flag = 0; //超过区间则不积分
		}
		else
		{
			PID->Ilimit_flag = 1; //加入积分：只有在测量值在-Ilimit~Ilimit范围内才加入积分
			PID->Integral += PID->Error; //对误差积分Sk=S(k-1)+Ek
			if(PID->Integral > PID->Irange)
				PID->Integral = PID->Irange;
			if(PID->Integral < -PID->Irange)
				PID->Integral = -PID->Irange;
		}
	}
	else
	{
		PID->Integral = 0;
	}
	PID->PreError = PID->Error; //赋值给前一个误差值
}

// 2 初始化PID结构体里的一些成员值
void PidParameter_init(void) //由于PID参数保存在flash中，所以此函数初始化时不用再初始化这些参数。但是，flash中的
{							 //参数有可能因为误操作而被擦除，若flash读取参数失败，则初始化为默认参数。
	//ROLL轴
	PID_ROL_Rate.Ilimit_flag = 0;  //Roll轴角速度积分的分离标志
	PID_ROL_Rate.Ilimit = 250;     //Roll轴角速度积分范围
	PID_ROL_Rate.Irange = 2000;    //Roll轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	PID_ROL_Angle.Ilimit_flag = 0; //Roll轴角度积分的分离标志
	PID_ROL_Angle.Ilimit = 35;     //Roll轴角度积分范围
	PID_ROL_Angle.Irange = 1000;   //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）

	//PITCH轴
	PID_PIT_Rate.Ilimit_flag = 0;  //Pitch轴角速度积分的分离标志
	PID_PIT_Rate.Ilimit = 250;     //Pitch轴角速度积分范围
	PID_PIT_Rate.Irange = 2000;    //Pitch轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	PID_PIT_Angle.Ilimit_flag = 0; //Roll轴角度积分的分离标志
	PID_PIT_Angle.Ilimit = 35;     //Roll轴角度积分范围
	PID_PIT_Angle.Irange = 1000;   //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	
	//YAW轴
	PID_YAW_Rate.Ilimit_flag = 0;  //Yaw轴角速度积分的分离标志
	PID_YAW_Rate.Ilimit = 150;     //Yaw轴角速度积分范围
	PID_YAW_Rate.Irange = 1200;    //Yaw轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	PID_YAW_Angle.Ilimit_flag = 0; //Yaw轴角度积分的分离标志
	PID_YAW_Angle.Ilimit = 35;     //Yaw轴角度积分范围
	PID_YAW_Angle.Irange = 200;    //Yaw轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
  
//	//高度环
//	PID_ALT_Rate.Ilimit_flag = 0;
//	PID_ALT_Rate.Ilimit = 0;
//	PID_ALT_Rate.Irange = 0;
//	PID_ALT.Ilimit_flag = 0;
//	PID_ALT.Ilimit = 0; //100
//	PID_ALT.Irange = 0; //200
}
