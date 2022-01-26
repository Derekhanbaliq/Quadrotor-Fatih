/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/28
�����ܣ���̬����
��    ע����Ԫ��->DCM�������Ҿ���->ŷ����+Mahony�����˲�
*******************************************************************************/
#include "imu.h"
#include "structconfig.h"
#include "math.h"
#include "mpu9250.h"
#include "filter.h"
#include "stdio.h"

#define Kp_New 0.9f	//�����˲���ǰ���ݵ�Ȩ��
#define Kp_Old 0.1f //�����˲���ʷ���ݵ�Ȩ��

#define G			9.80665f	//m/s^2
#define RadtoDeg    57.324841f	//���ȵ��Ƕ�(����*180/3.1415)
#define DegtoRad    0.0174533f	//�Ƕȵ�����(�Ƕ�*3.1415/180)

#define Acc_Gain 		0.0001220f	//���ٶȼƲ���(��ʼ�����ٶȼ�������+-4g, LSBa=2*4/65535)
#define Gyro_Gain		0.0609756f	//�����ǲ���(��ʼ��������������+-2000��/s, LSBg=2*2000/65535)
#define Gyro_GainRad	0.0010641f	//�����ǲ�������λrad/s (3.1415/180*LSBg)

FLOAT_ANGLE Att_Angle; //�ɻ���̬����
FLOAT_XYZ	Gyro_rad, Gyro_rad_old; //�������ǵĸ�ͨ������������ת��Ϊ������
FLOAT_XYZ	Acc_filt, Gyro_filt, Acc_filt_old; //�˲���ĸ�ͨ������

float accb[3], DCMgb[3][3]; //accb: ���ٶȾ��� DCMgb: �������Ҿ���
u8	AccbUpdate=0;

// 1 ���ټ���1/Sqrt(x)
//����ͨ��Sqrt()����Ҫ��4��See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
static float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//****************************************B
// 2 ��������ȥ��ƫ��������˲�������������
//�˺�����ԭʼ���ݽ���У׼��ȥ��ƫ���˲��������������� Ϊ��̬������׼��
void Prepare_Data(void)
{
	static u8 IIR_mode=1;
	
	MPU9250_Read(); //������ȡ����������
	
	MPU9250_ZeroDriftRemoval(); //��MPU9250���д�����ȥ��ƫ����û�м�����ƫ�������ƫ
	
//	Aver_FilterXYZ(&MPU9250_ACC_RAW, &Acc_filt, 12); //�Լ��ٶ�ԭʼ���ݽ��л��������˲� �����������ĺ�
	SortAver_FilterXYZ(&MPU9250_ACC_RAW, &Acc_filt, 12); //�Լ��ٶ�ԭʼ���ݽ���ȥ��ֵ���������˲� 12������һ��batch ȥ�������Сֵʣ��10��ֵ
	
	//���ٶ�16λADֵ ת���� m/(s^2)��λ��ֵ
	Acc_filt.X=(float)Acc_filt.X*Acc_Gain*G;
	Acc_filt.Y=(float)Acc_filt.Y*Acc_Gain*G;
	Acc_filt.Z=(float)Acc_filt.Z*Acc_Gain*G;
//	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n", Acc_filt.X, Acc_filt.Y, Acc_filt.Z);
	
	//������16λADֵ ת���� rad/s
	Gyro_rad.X=(float)MPU9250_GYRO_RAW.X*Gyro_GainRad;  
	Gyro_rad.Y=(float)MPU9250_GYRO_RAW.Y*Gyro_GainRad;
	Gyro_rad.Z=(float)MPU9250_GYRO_RAW.Z*Gyro_GainRad;
//	printf("gx=%0.2f gy=%0.2f gz=%0.2f\r\n", Gyro_rad.X, Gyro_rad.Y, Gyro_rad.Z);
	
	if(IIR_mode)
	{
		Acc_filt.X=Acc_filt.X*Kp_New+Acc_filt_old.X*Kp_Old;
		Acc_filt.Y=Acc_filt.Y*Kp_New+Acc_filt_old.Y*Kp_Old;
		Acc_filt.Z=Acc_filt.Z*Kp_New+Acc_filt_old.Z*Kp_Old;
		
		Acc_filt_old.X=Acc_filt.X; //�����ֵ
		Acc_filt_old.Y=Acc_filt.Y;
		Acc_filt_old.Z=Acc_filt.Z;
	}
	
//	//�ò���Ӧ����altitude.c��
//	accb[0] = Acc_filt.X;
//	accb[1] = Acc_filt.Y;
//	accb[2] = Acc_filt.Z;
//	if(accb[0]&&accb[1]&&accb[2])
//	{
//		AccbUpdate=1; //accb���������
//	}
}

//****************************************C ��ȡ��̬��
#define Kp     1.50f	//proportional gain governs rate of convergence to accelerometer/magnetometer
						//����������Ƽ��ٶȼƣ������Ƶ���������
#define Ki     0.005f	//integral gain governs rate of convergence of gyroscope biases
						//�����������������ƫ��������ٶ�
						//Kp=Ki=0 ��ȫ����������
#define halfT  0.005f	//half the sample period �������ڵ�һ�� һ����������������Ԫ���е�(1/2)T

float q0=1, q1=0, q2=0, q3=0;     //��Ԫ�� quaternion elements representing the estimated orientation
float exInt=0, eyInt=0, ezInt=0;    //�������� scaled integral error ������PI��������ʽ�е�һ�� ����������̬���

void IMUupdate(FLOAT_XYZ *Gyro_rad, FLOAT_XYZ *Acc_filt, FLOAT_ANGLE *Att_Angle) //Gyro_rad:ָ����ٶȵ�ָ��(��λ�����ǻ���) AccFilt:ָ����ٶȵ�ָ�� Att_Angle:ָ����̬�ǵ�ָ��
{
	u8 i;
	float matrix[9] = {1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f, 1.f}; //��ʼ��DCM����
	float ax=Acc_filt->X, ay=Acc_filt->Y, az=Acc_filt->Z;
	float gx=Gyro_rad->X, gy=Gyro_rad->Y, gz=Gyro_rad->Z;
	float vx, vy, vz; //��Ԫ���Ƶ����������������ٶȱ���
	float ex, ey, ez; //ͨ������g��ʵ��g�����õ����
	float norm; //ģ�ĵ���
	
	float q0q0=q0*q0; //��˼����ʱ�����Ծ���ʼ��
	float q0q1=q0*q1;
	float q0q2=q0*q2;
	float q0q3=q0*q3;
	float q1q1=q1*q1;
	float q1q2=q1*q2;
	float q1q3=q1*q3;
	float q2q2=q2*q2;
	float q2q3=q2*q3;
	float q3q3=q3*q3;
	
	if(ax*ay*az==0) //�жϼ��ٶȼ��Ƿ���Ч
		return; //==0����Ч ����
	
	//���ٶȼƲ�������������(��������ϵb)
	norm=invSqrt(ax*ax+ay*ay+az*az);
	ax=ax*norm;
	ay=ay*norm;
	az=az*norm;
//	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);
	
	//1 ���������ݵ��ں�
	//�����ǻ��ֹ�����������(��������ϵ) Vb*G, G=[0, 0, 1], ��Vb�Ѿ��ǵ�λ����������
	vx=2*(q1q3-q0q2); //debug: ����д���˵Ⱥţ�������
	vy=2*(q0q1+q2q3);
	vz=q0q0-q1q1-q2q2+q3q3;
//	printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz);
	
	//��������������������������������������֮������
	ex=(ay*vz-az*vy); //+ (my*wz - mz*wy);                     
	ey=(az*vx-ax*vz); //+ (mz*wx - mx*wz);
	ez=(ax*vy-ay*vx); //+ (mx*wy - my*wx);
	
	//2 Mahony����
	//��������������л���
	exInt=exInt+ex*Ki; //���ּ��ۼ�							 
	eyInt=eyInt+ey*Ki;
	ezInt=ezInt+ez*Ki;
	
	//�����PI�󲹳�����������
	gx=gx+Kp*ex+exInt; //+P����+I����
	gy=gy+Kp*ey+eyInt;
	gz=gz+Kp*ez+ezInt; //�����gz����û�й۲��߽���У���������Ư�ƣ����ֳ������ǻ������������Լ�
	
	//3 �����Ԫ�� 
	//ͨ��΢�ַ������(����Ԫ����t��΢�ֺ���һ���������������)
	q0=q0+(-q1*gx- q2*gy - q3*gz)*halfT;
	q1=q1+(q0*gx + q2*gz - q3*gy)*halfT;
	q2=q2+(q0*gy - q1*gz + q3*gx)*halfT;
	q3=q3+(q0*gz + q1*gy - q2*gx)*halfT;
	
	//��λ����Ԫ��
	norm=invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0=q0*norm;
	q1=q1*norm;
	q2=q2*norm;  
	q3=q3*norm;
	
	//4 DCM�������Ҿ��� ����������ϵeת������������ϵb
	//���㶨�߲���
	matrix[0]= q0q0 +  q1q1 - q2q2 - q3q3; // 11(ǰ�к���)
	matrix[1]= 2.f  * (q1q2 + q0q3);	   // 12
	matrix[2]= 2.f  * (q1q3 - q0q2);	   // 13
	matrix[3]= 2.f  * (q1q2 - q0q3);	   // 21
	matrix[4]= q0q0 -  q1q1 + q2q2 - q3q3; // 22
	matrix[5]= 2.f  * (q2q3 + q0q1);	   // 23
	matrix[6]= 2.f  * (q1q3 + q0q2);	   // 31
	matrix[7]= 2.f  * (q2q3 - q0q1);	   // 32
	matrix[8]= q0q0 -  q1q1 - q2q2 + q3q3; // 33
	
	//5 ��Ԫ��ת��Ϊŷ����(Z->Y->X)
//	Att_Angle->yaw +=Gyro_rad->Z*RadtoDeg*0.01f; //yaw ҡͷ ����ƫ�����ɻ��ֵ��� ������ƫ���Ư�� ����PID�������⴦��
	Att_Angle->yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f; // yaw
	Att_Angle->pit = -asin(2.f*(q1q3-q0q2))*57.3f; //pitch ��ͷ
	Att_Angle->rol = atan2(2.f*q2q3+2.f*q0q1,q0q0-q1q1-q2q2+q3q3)*57.3f; //roll ��ͷ
//	printf("pitch=%4.2f, roll=%4.2f, yaw=%4.2f\r\n\r\n", Att_Angle->pit, Att_Angle->rol, Att_Angle->yaw);
	
	for(i=0; i<9; i++)
	{
		*(&(DCMgb[0][0])+i) = matrix[i]; //(&(DCMgb[0][0])+i)���ָ�����ָ��ı�����ֵ(*) 
		// *p=*(&a)=value	p=&a	*p=a
	}
	
	//ʧ�ر���(����ʱ��ע�͵�)
//	Safety_Check(); 
}
