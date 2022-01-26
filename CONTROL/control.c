/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/09/11
程序功能：
*******************************************************************************/
#include "control.h"
#include "structconfig.h"
#include "math.h"
#include "pid.h"
#include "motor.h"

#define RadtoDeg    57.324841f	//弧度到角度(弧度*180/3.1415)
#define DegtoRad    0.0174533f	//角度到弧度(角度*3.1415/180)

//角度环PID 
PID_TYPE PID_ROL_Angle; //里面的参数由pid.c的函数赋值
PID_TYPE PID_PIT_Angle;
PID_TYPE PID_YAW_Angle;
//角速度环PID 
PID_TYPE PID_ROL_Rate;
PID_TYPE PID_PIT_Rate;
PID_TYPE PID_YAW_Rate;

float Pre_THROTTLE, THROTTLE;
float Motor_PWM_1=0.0f, Motor_PWM_2=0.0f, Motor_PWM_3=0.0f, Motor_PWM_4=0.0f;
u8 WiFi_Controlflag=0, Airplane_Enable;

FLOAT_ANGLE Measure_Angle, Target_Angle;

// 1 姿态控制-角度环控制-角速度环控制
/*		acc_in:测量值					rc_in:遥控器设定值
		gyr_in:MPU6050读取的角速度值	armed:记录命令			
	注意：动力分配应一个轴一个轴分配，切勿三个轴同时分配！！！！	*/
void Control(FLOAT_ANGLE *acc_in, FLOAT_XYZ *gyr_in, RC_TYPE *rc_in, u8 armed)
{
	Measure_Angle.rol = acc_in->rol; 
	Measure_Angle.pit = acc_in->pit; 
	Measure_Angle.yaw = acc_in->yaw; 
	Target_Angle.rol = (float)((rc_in->ROLL-1500)/12.0f);
	Target_Angle.pit = (float)((rc_in->PITCH-1500)/12.0f);
	Target_Angle.yaw = (float)((1500-rc_in->YAW)/12.0f); 
	
	if(WiFi_Controlflag) //WiFi模式禁止偏航
	{
		Target_Angle.yaw=0;
	}
	else
	{
		if(fabs(Target_Angle.yaw)<4)
		{
			Target_Angle.yaw=0; //去抖动
		}
	}
	
	if(1 == (GET_FLAG(FLY_MODE))) //无头模式
	{
	  Yaw_Carefree(&Target_Angle,&Measure_Angle);
	}

	//角度环 外环
	PID_Postion_Cal(&PID_ROL_Angle, Target_Angle.rol, Measure_Angle.rol); //ROLL角度环PID 输入角度输出角速度
	PID_Postion_Cal(&PID_PIT_Angle, Target_Angle.pit, Measure_Angle.pit); //PITCH角度环PID 
//	PID_Postion_Cal(&PID_YAW_Angle, Target_Angle.yaw, Measure_Angle.yaw); //YAW角度环PID
	
	//角速度环 内环
	PID_Postion_Cal(&PID_ROL_Rate, PID_ROL_Angle.OutPut, gyr_in->X*RadtoDeg); //ROLL角速度环PID 输入角度环的输出 输出电机控制量
	PID_Postion_Cal(&PID_PIT_Rate, PID_PIT_Angle.OutPut, gyr_in->Y*RadtoDeg); //PITCH角速度环PID
	PID_Postion_Cal(&PID_YAW_Rate, Target_Angle.yaw*PID_YAW_Angle.P, gyr_in->Z*RadtoDeg); //YAW角速度环PID 特殊处理！ 输入角度 输出电机控制量
	
	//动力分配(好好研究！！！！)
	if(rc_in->THROTTLE>180 && armed) //当油门大于180且飞机解锁时 动力分配生效
	{
		Motor_PWM_1 = rc_in->THROTTLE + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;   
		Motor_PWM_2 = rc_in->THROTTLE - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;   
		Motor_PWM_3 = rc_in->THROTTLE - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;   
		Motor_PWM_4 = rc_in->THROTTLE + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;   
	}
	else
	{
		Motor_PWM_1 = 0;
		Motor_PWM_2 = 0;
		Motor_PWM_3 = 0;
		Motor_PWM_4 = 0;
	}
	Motor_Pwm(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4); //数值分配给定时器 输出对应占空比PWM波
}

// 2 航向角不回中控制 因为遥控器航向舵会自动回中
s16 Yaw_Control(float TARGET_YAW)
{
	static s16 YAW=0; //根据目标航向角计算出的不回中角度
	if(Airplane_Enable)
	{
		if(!WiFi_Controlflag) //遥控器控制航向角
		{
			if(TARGET_YAW>2)
				YAW+=2;
			if(TARGET_YAW<-2)
				YAW-=2;
		}
		else //APP遥控时，不允许改变航向角
		{
			YAW=YAW;
		}
	}
	return YAW;
}

// 3 无头角度控制
void Yaw_Carefree(FLOAT_ANGLE *Target_Angle, const FLOAT_ANGLE *Measure_Angle)
{
	float yawRad = fabs(Measure_Angle->yaw) * DegtoRad; //测量的yaw角(rad)
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	float originalRoll = Target_Angle->rol; //目标roll角
	float originalPitch = Target_Angle->pit; //目标pitch角
	
	Target_Angle->rol = originalRoll*cosy + originalPitch*siny;
	Target_Angle->pit = originalPitch*cosy - originalRoll*siny;
}

// 4 飞机姿态安全监测
void Safety_Check(void)
{
	if((fabs(Att_Angle.pit)>45.0f || fabs(Att_Angle.rol)>45.0f) && (fabs(Acc_filt.X)>9.0f || fabs(Acc_filt.Y)>9.0f))
	{							//姿态角的pitch角或roll角有一个大于45度 且滤波后的加速度计数据的绝对值大于9g
		 Airplane_Enable = 0;
		 Motor_PWM_1 = 0;
		 Motor_PWM_2 = 0;
		 Motor_PWM_3 = 0;
		 Motor_PWM_4 = 0;
	}
}
