/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/09/11
�����ܣ�
*******************************************************************************/
#include "control.h"
#include "structconfig.h"
#include "math.h"
#include "pid.h"
#include "motor.h"

#define RadtoDeg    57.324841f	//���ȵ��Ƕ�(����*180/3.1415)
#define DegtoRad    0.0174533f	//�Ƕȵ�����(�Ƕ�*3.1415/180)

//�ǶȻ�PID 
PID_TYPE PID_ROL_Angle; //����Ĳ�����pid.c�ĺ�����ֵ
PID_TYPE PID_PIT_Angle;
PID_TYPE PID_YAW_Angle;
//���ٶȻ�PID 
PID_TYPE PID_ROL_Rate;
PID_TYPE PID_PIT_Rate;
PID_TYPE PID_YAW_Rate;

float Pre_THROTTLE, THROTTLE;
float Motor_PWM_1=0.0f, Motor_PWM_2=0.0f, Motor_PWM_3=0.0f, Motor_PWM_4=0.0f;
u8 WiFi_Controlflag=0, Airplane_Enable;

FLOAT_ANGLE Measure_Angle, Target_Angle;

// 1 ��̬����-�ǶȻ�����-���ٶȻ�����
/*		acc_in:����ֵ					rc_in:ң�����趨ֵ
		gyr_in:MPU6050��ȡ�Ľ��ٶ�ֵ	armed:��¼����			
	ע�⣺��������Ӧһ����һ������䣬����������ͬʱ���䣡������	*/
void Control(FLOAT_ANGLE *acc_in, FLOAT_XYZ *gyr_in, RC_TYPE *rc_in, u8 armed)
{
	Measure_Angle.rol = acc_in->rol; 
	Measure_Angle.pit = acc_in->pit; 
	Measure_Angle.yaw = acc_in->yaw; 
	Target_Angle.rol = (float)((rc_in->ROLL-1500)/12.0f);
	Target_Angle.pit = (float)((rc_in->PITCH-1500)/12.0f);
	Target_Angle.yaw = (float)((1500-rc_in->YAW)/12.0f); 
	
	if(WiFi_Controlflag) //WiFiģʽ��ֹƫ��
	{
		Target_Angle.yaw=0;
	}
	else
	{
		if(fabs(Target_Angle.yaw)<4)
		{
			Target_Angle.yaw=0; //ȥ����
		}
	}
	
	if(1 == (GET_FLAG(FLY_MODE))) //��ͷģʽ
	{
	  Yaw_Carefree(&Target_Angle,&Measure_Angle);
	}

	//�ǶȻ� �⻷
	PID_Postion_Cal(&PID_ROL_Angle, Target_Angle.rol, Measure_Angle.rol); //ROLL�ǶȻ�PID ����Ƕ�������ٶ�
	PID_Postion_Cal(&PID_PIT_Angle, Target_Angle.pit, Measure_Angle.pit); //PITCH�ǶȻ�PID 
//	PID_Postion_Cal(&PID_YAW_Angle, Target_Angle.yaw, Measure_Angle.yaw); //YAW�ǶȻ�PID
	
	//���ٶȻ� �ڻ�
	PID_Postion_Cal(&PID_ROL_Rate, PID_ROL_Angle.OutPut, gyr_in->X*RadtoDeg); //ROLL���ٶȻ�PID ����ǶȻ������ ������������
	PID_Postion_Cal(&PID_PIT_Rate, PID_PIT_Angle.OutPut, gyr_in->Y*RadtoDeg); //PITCH���ٶȻ�PID
	PID_Postion_Cal(&PID_YAW_Rate, Target_Angle.yaw*PID_YAW_Angle.P, gyr_in->Z*RadtoDeg); //YAW���ٶȻ�PID ���⴦�� ����Ƕ� ������������
	
	//��������(�ú��о���������)
	if(rc_in->THROTTLE>180 && armed) //�����Ŵ���180�ҷɻ�����ʱ ����������Ч
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
	Motor_Pwm(Motor_PWM_1, Motor_PWM_2, Motor_PWM_3, Motor_PWM_4); //��ֵ�������ʱ�� �����Ӧռ�ձ�PWM��
}

// 2 ����ǲ����п��� ��Ϊң�����������Զ�����
s16 Yaw_Control(float TARGET_YAW)
{
	static s16 YAW=0; //����Ŀ�꺽��Ǽ�����Ĳ����нǶ�
	if(Airplane_Enable)
	{
		if(!WiFi_Controlflag) //ң�������ƺ����
		{
			if(TARGET_YAW>2)
				YAW+=2;
			if(TARGET_YAW<-2)
				YAW-=2;
		}
		else //APPң��ʱ��������ı亽���
		{
			YAW=YAW;
		}
	}
	return YAW;
}

// 3 ��ͷ�Ƕȿ���
void Yaw_Carefree(FLOAT_ANGLE *Target_Angle, const FLOAT_ANGLE *Measure_Angle)
{
	float yawRad = fabs(Measure_Angle->yaw) * DegtoRad; //������yaw��(rad)
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	float originalRoll = Target_Angle->rol; //Ŀ��roll��
	float originalPitch = Target_Angle->pit; //Ŀ��pitch��
	
	Target_Angle->rol = originalRoll*cosy + originalPitch*siny;
	Target_Angle->pit = originalPitch*cosy - originalRoll*siny;
}

// 4 �ɻ���̬��ȫ���
void Safety_Check(void)
{
	if((fabs(Att_Angle.pit)>45.0f || fabs(Att_Angle.rol)>45.0f) && (fabs(Acc_filt.X)>9.0f || fabs(Acc_filt.Y)>9.0f))
	{							//��̬�ǵ�pitch�ǻ�roll����һ������45�� ���˲���ļ��ٶȼ����ݵľ���ֵ����9g
		 Airplane_Enable = 0;
		 Motor_PWM_1 = 0;
		 Motor_PWM_2 = 0;
		 Motor_PWM_3 = 0;
		 Motor_PWM_4 = 0;
	}
}
