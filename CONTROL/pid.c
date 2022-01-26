/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/09/12
�����ܣ�λ��ʽPID�����㷨
*******************************************************************************/
#include "pid.h"
#include "structconfig.h"

// 1 λ��ʽPID�㷨 �ǶȻ�����ٶȻ����ô˺���
void PID_Postion_Cal(PID_TYPE *PID, float target, float measure) //target: Ŀ��ֵ, measure: ����ֵ
{
	PID->Error = target-measure; 			  //���Ek=Sk-Xk
	PID->Differ = PID->Error - PID->PreError; //΢���� Ek-E(k-1)
	
	PID->Pout = PID->P * PID->Error;					   //��������Pout=Kp*Ek
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral; //���ֿ���Iout=(0/1)*Ki*(Sk*T) Sk:sum T:��������
	PID->Dout = PID->D * PID->Differ;					   //΢�ֿ���Dout=Kd*([Ek-E(k-1)]/T)
	
	PID->OutPut = PID->Pout + PID->Iout + PID->Dout; //�ܿ�������� Uout
	
	if(Airplane_Enable==1 && RC_Control.THROTTLE>=180) //�ɻ��������ټ������I����ֹ���ֹ���
	{
		if(measure>(PID->Ilimit) || measure<(-PID->Ilimit)) //���ַ���
		{
			PID->Ilimit_flag = 0; //���������򲻻���
		}
		else
		{
			PID->Ilimit_flag = 1; //������֣�ֻ���ڲ���ֵ��-Ilimit~Ilimit��Χ�ڲż������
			PID->Integral += PID->Error; //��������Sk=S(k-1)+Ek
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
	PID->PreError = PID->Error; //��ֵ��ǰһ�����ֵ
}

// 2 ��ʼ��PID�ṹ�����һЩ��Աֵ
void PidParameter_init(void) //����PID����������flash�У����Դ˺�����ʼ��ʱ�����ٳ�ʼ����Щ���������ǣ�flash�е�
{							 //�����п�����Ϊ�����������������flash��ȡ����ʧ�ܣ����ʼ��ΪĬ�ϲ�����
	//ROLL��
	PID_ROL_Rate.Ilimit_flag = 0;  //Roll����ٶȻ��ֵķ����־
	PID_ROL_Rate.Ilimit = 250;     //Roll����ٶȻ��ַ�Χ
	PID_ROL_Rate.Irange = 2000;    //Roll����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
	PID_ROL_Angle.Ilimit_flag = 0; //Roll��ǶȻ��ֵķ����־
	PID_ROL_Angle.Ilimit = 35;     //Roll��ǶȻ��ַ�Χ
	PID_ROL_Angle.Irange = 1000;   //Roll��ǶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�

	//PITCH��
	PID_PIT_Rate.Ilimit_flag = 0;  //Pitch����ٶȻ��ֵķ����־
	PID_PIT_Rate.Ilimit = 250;     //Pitch����ٶȻ��ַ�Χ
	PID_PIT_Rate.Irange = 2000;    //Pitch����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
	PID_PIT_Angle.Ilimit_flag = 0; //Roll��ǶȻ��ֵķ����־
	PID_PIT_Angle.Ilimit = 35;     //Roll��ǶȻ��ַ�Χ
	PID_PIT_Angle.Irange = 1000;   //Roll��ǶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
	
	//YAW��
	PID_YAW_Rate.Ilimit_flag = 0;  //Yaw����ٶȻ��ֵķ����־
	PID_YAW_Rate.Ilimit = 150;     //Yaw����ٶȻ��ַ�Χ
	PID_YAW_Rate.Irange = 1200;    //Yaw����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
	PID_YAW_Angle.Ilimit_flag = 0; //Yaw��ǶȻ��ֵķ����־
	PID_YAW_Angle.Ilimit = 35;     //Yaw��ǶȻ��ַ�Χ
	PID_YAW_Angle.Irange = 200;    //Yaw��ǶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
  
//	//�߶Ȼ�
//	PID_ALT_Rate.Ilimit_flag = 0;
//	PID_ALT_Rate.Ilimit = 0;
//	PID_ALT_Rate.Irange = 0;
//	PID_ALT.Ilimit_flag = 0;
//	PID_ALT.Ilimit = 0; //100
//	PID_ALT.Irange = 0; //200
}
