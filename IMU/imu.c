/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/28
程序功能：姿态解算
备    注：四元数->DCM方向余弦矩阵->欧拉角+Mahony互补滤波
*******************************************************************************/
#include "imu.h"
#include "structconfig.h"
#include "math.h"
#include "mpu9250.h"
#include "filter.h"
#include "stdio.h"

#define Kp_New 0.9f	//互补滤波当前数据的权重
#define Kp_Old 0.1f //互补滤波历史数据的权重

#define G			9.80665f	//m/s^2
#define RadtoDeg    57.324841f	//弧度到角度(弧度*180/3.1415)
#define DegtoRad    0.0174533f	//角度到弧度(角度*3.1415/180)

#define Acc_Gain 		0.0001220f	//加速度计步长(初始化加速度计满量程+-4g, LSBa=2*4/65535)
#define Gyro_Gain		0.0609756f	//陀螺仪步长(初始化陀螺仪满量程+-2000°/s, LSBg=2*2000/65535)
#define Gyro_GainRad	0.0010641f	//陀螺仪步长，单位rad/s (3.1415/180*LSBg)

FLOAT_ANGLE Att_Angle; //飞机姿态数据
FLOAT_XYZ	Gyro_rad, Gyro_rad_old; //把陀螺仪的各通道读出的数据转换为弧度制
FLOAT_XYZ	Acc_filt, Gyro_filt, Acc_filt_old; //滤波后的各通道数据

float accb[3], DCMgb[3][3]; //accb: 加速度矩阵 DCMgb: 方向余弦矩阵
u8	AccbUpdate=0;

// 1 快速计算1/Sqrt(x)
//比普通的Sqrt()函数要快4倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
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
// 2 对陀螺仪去零偏后的数据滤波赋予物理意义
//此函数对原始数据进行校准：去零偏、滤波、赋予物理意义 为姿态结算做准备
void Prepare_Data(void)
{
	static u8 IIR_mode=1;
	
	MPU9250_Read(); //触发读取，立即返回
	
	MPU9250_ZeroDriftRemoval(); //对MPU9250进行处理，减去零偏，若没有计算零偏则计算零偏
	
//	Aver_FilterXYZ(&MPU9250_ACC_RAW, &Acc_filt, 12); //对加速度原始数据进行滑动窗口滤波 这个不如下面的好
	SortAver_FilterXYZ(&MPU9250_ACC_RAW, &Acc_filt, 12); //对加速度原始数据进行去极值滑动窗口滤波 12个数据一个batch 去掉最大最小值剩下10个值
	
	//加速度16位AD值 转换成 m/(s^2)单位的值
	Acc_filt.X=(float)Acc_filt.X*Acc_Gain*G;
	Acc_filt.Y=(float)Acc_filt.Y*Acc_Gain*G;
	Acc_filt.Z=(float)Acc_filt.Z*Acc_Gain*G;
//	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n", Acc_filt.X, Acc_filt.Y, Acc_filt.Z);
	
	//陀螺仪16位AD值 转换成 rad/s
	Gyro_rad.X=(float)MPU9250_GYRO_RAW.X*Gyro_GainRad;  
	Gyro_rad.Y=(float)MPU9250_GYRO_RAW.Y*Gyro_GainRad;
	Gyro_rad.Z=(float)MPU9250_GYRO_RAW.Z*Gyro_GainRad;
//	printf("gx=%0.2f gy=%0.2f gz=%0.2f\r\n", Gyro_rad.X, Gyro_rad.Y, Gyro_rad.Z);
	
	if(IIR_mode)
	{
		Acc_filt.X=Acc_filt.X*Kp_New+Acc_filt_old.X*Kp_Old;
		Acc_filt.Y=Acc_filt.Y*Kp_New+Acc_filt_old.Y*Kp_Old;
		Acc_filt.Z=Acc_filt.Z*Kp_New+Acc_filt_old.Z*Kp_Old;
		
		Acc_filt_old.X=Acc_filt.X; //储存旧值
		Acc_filt_old.Y=Acc_filt.Y;
		Acc_filt_old.Z=Acc_filt.Z;
	}
	
//	//该部分应用于altitude.c中
//	accb[0] = Acc_filt.X;
//	accb[1] = Acc_filt.Y;
//	accb[2] = Acc_filt.Z;
//	if(accb[0]&&accb[1]&&accb[2])
//	{
//		AccbUpdate=1; //accb三项都有数据
//	}
}

//****************************************C 获取姿态角
#define Kp     1.50f	//proportional gain governs rate of convergence to accelerometer/magnetometer
						//比例增益控制加速度计，磁力计的收敛速率
#define Ki     0.005f	//integral gain governs rate of convergence of gyroscope biases
						//积分增益控制陀螺仪偏差的收敛速度
						//Kp=Ki=0 完全相信陀螺仪
#define halfT  0.005f	//half the sample period 采样周期的一半 一阶龙格库塔法求得四元数中的(1/2)T

float q0=1, q1=0, q2=0, q3=0;     //四元数 quaternion elements representing the estimated orientation
float exInt=0, eyInt=0, ezInt=0;    //误差积分项 scaled integral error 误差补偿中PI控制器公式中的一项 用于消除静态误差

void IMUupdate(FLOAT_XYZ *Gyro_rad, FLOAT_XYZ *Acc_filt, FLOAT_ANGLE *Att_Angle) //Gyro_rad:指向角速度的指针(单位必须是弧度) AccFilt:指向角速度的指针 Att_Angle:指向姿态角的指针
{
	u8 i;
	float matrix[9] = {1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f, 1.f}; //初始化DCM矩阵
	float ax=Acc_filt->X, ay=Acc_filt->Y, az=Acc_filt->Z;
	float gx=Gyro_rad->X, gy=Gyro_rad->Y, gz=Gyro_rad->Z;
	float vx, vy, vz; //四元数推导出的理论重力加速度变量
	float ex, ey, ez; //通过理论g与实际g叉乘求得的误差
	float norm; //模的倒数
	
	float q0q0=q0*q0; //相乘计算耗时，所以均初始化
	float q0q1=q0*q1;
	float q0q2=q0*q2;
	float q0q3=q0*q3;
	float q1q1=q1*q1;
	float q1q2=q1*q2;
	float q1q3=q1*q3;
	float q2q2=q2*q2;
	float q2q3=q2*q3;
	float q3q3=q3*q3;
	
	if(ax*ay*az==0) //判断加速度计是否有效
		return; //==0则无效 返回
	
	//加速度计测量的重力向量(机体坐标系b)
	norm=invSqrt(ax*ax+ay*ay+az*az);
	ax=ax*norm;
	ay=ay*norm;
	az=az*norm;
//	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);
	
	//1 传感器数据的融合
	//陀螺仪积分估计重力向量(机体坐标系) Vb*G, G=[0, 0, 1], 即Vb已经是单位化的向量了
	vx=2*(q1q3-q0q2); //debug: 减号写成了等号！！！！
	vy=2*(q0q1+q2q3);
	vz=q0q0-q1q1-q2q2+q3q3;
//	printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz);
	
	//测量的重力向量与估算的重力向量叉乘求出向量之间的误差
	ex=(ay*vz-az*vy); //+ (my*wz - mz*wy);                     
	ey=(az*vx-ax*vz); //+ (mz*wx - mx*wz);
	ez=(ax*vy-ay*vx); //+ (mx*wy - my*wx);
	
	//2 Mahony误差补偿
	//用上面求出误差进行积分
	exInt=exInt+ex*Ki; //积分即累加							 
	eyInt=eyInt+ey*Ki;
	ezInt=ezInt+ez*Ki;
	
	//将误差PI后补偿到陀螺仪中
	gx=gx+Kp*ex+exInt; //+P补偿+I补偿
	gy=gy+Kp*ey+eyInt;
	gz=gz+Kp*ez+ezInt; //这里的gz由于没有观测者进行校正，会产生漂移，表现出来就是积分自增或者自减
	
	//3 求解四元数 
	//通过微分方程求解(对四元数求t的微分后再一阶龙格库塔法积分)
	q0=q0+(-q1*gx- q2*gy - q3*gz)*halfT;
	q1=q1+(q0*gx + q2*gz - q3*gy)*halfT;
	q2=q2+(q0*gy - q1*gz + q3*gx)*halfT;
	q3=q3+(q0*gz + q1*gy - q2*gx)*halfT;
	
	//单位化四元数
	norm=invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0=q0*norm;
	q1=q1*norm;
	q2=q2*norm;  
	q3=q3*norm;
	
	//4 DCM方向余弦矩阵 将惯性坐标系e转换到集体坐标系b
	//定点定高才用
	matrix[0]= q0q0 +  q1q1 - q2q2 - q3q3; // 11(前列后行)
	matrix[1]= 2.f  * (q1q2 + q0q3);	   // 12
	matrix[2]= 2.f  * (q1q3 - q0q2);	   // 13
	matrix[3]= 2.f  * (q1q2 - q0q3);	   // 21
	matrix[4]= q0q0 -  q1q1 + q2q2 - q3q3; // 22
	matrix[5]= 2.f  * (q2q3 + q0q1);	   // 23
	matrix[6]= 2.f  * (q1q3 + q0q2);	   // 31
	matrix[7]= 2.f  * (q2q3 - q0q1);	   // 32
	matrix[8]= q0q0 -  q1q1 - q2q2 + q3q3; // 33
	
	//5 四元数转换为欧拉角(Z->Y->X)
//	Att_Angle->yaw +=Gyro_rad->Z*RadtoDeg*0.01f; //yaw 摇头 由于偏航角由积分得来 所以有偏差会漂移 所以PID控制特殊处理
	Att_Angle->yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f; // yaw
	Att_Angle->pit = -asin(2.f*(q1q3-q0q2))*57.3f; //pitch 点头
	Att_Angle->rol = atan2(2.f*q2q3+2.f*q0q1,q0q0-q1q1-q2q2+q3q3)*57.3f; //roll 摆头
//	printf("pitch=%4.2f, roll=%4.2f, yaw=%4.2f\r\n\r\n", Att_Angle->pit, Att_Angle->rol, Att_Angle->yaw);
	
	for(i=0; i<9; i++)
	{
		*(&(DCMgb[0][0])+i) = matrix[i]; //(&(DCMgb[0][0])+i)这个指针变量指向的变量的值(*) 
		// *p=*(&a)=value	p=&a	*p=a
	}
	
	//失控保护(调试时可注释掉)
//	Safety_Check(); 
}
