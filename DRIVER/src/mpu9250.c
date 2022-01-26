/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/26
程序功能：MPU9250相关函数
备    注：
*******************************************************************************/
#include "mpu9250.h"
#include "iic.h"
#include "stdio.h"
#include "rgb.h"
#include "delay.h"
#include "structconfig.h"
#include "paramsave.h"

static u8    MPU9250_buff[14];					//加速度计(6) 陀螺仪(6) 温度(2) 的原始数据 14x8bit
INT16_XYZ	 GYRO_OFFSET_RAW, ACC_OFFSET_RAW;	//零点漂移数据
INT16_XYZ	 MPU9250_ACC_RAW, MPU9250_GYRO_RAW;	//读取值原始数据
u8   		 SENSOR_OFFSET_FLAG;	            //传感器校准标志位

//****************************************A 代码移植区
// 1 读一个字节数据到MPU9250寄存器
u8 MPU9250_ReadByte(u8 reg, u8 *data)
{
	if(IIC_ReadOneByte(MPU9250Addr, reg, data))
	   return 1;
	else
	   return 0;
}

// 2 写一个字节数据到MPU9250寄存器
u8 MPU9250_WriteByte(u8 reg, u8 data)
{
	if(IIC_WriteOneByte(MPU9250Addr, reg, data))
	   return 1;
	else
	   return 0;
}

// 3 读指定长度字节数据到MPU9250寄存器
u8 MPU9250_ReadLengthBytes(u8 reg, u8 len, u8 *data)
{
	if(IIC_ReadLengthBytes(MPU9250Addr, reg, len, data))
	   return 1;
	else
	   return 0;
}

// 4 写指定长度字节数据到MPU9250寄存器
u8 MPU9250_WriteLengthBytes(u8 reg, u8 len, u8 *data)
{
	if(IIC_WriteLengthBytes(MPU9250Addr, reg, len, data))
	   return 1;
	else
	   return 0;
}

//****************************************B 通讯检测
// 5 读取MPU9250的WHO_AM_I 返回 0x71(默认值) 0x68(复位值)
u8 MPU9250_getDeviceID(void)
{
    u8 buf;
	MPU9250_ReadByte(MPU9250_RA_WHO_AM_I, &buf);
    return buf;
}

// 6 检测MPU9250是否已经连接
u8 MPU9250_TestConnection(void) 
{
	if(MPU9250_getDeviceID() == 0x71)  
		return 1;
	else 
		return 0;
}

// 7 检测IIC总线上的MPU9250是否存在 没有则亮品红灯
 void MPU9250_Check(void) 
{ 
	while(!MPU9250_TestConnection())
	{
		printf("\rMPU9250 no connect...\r\n");
		RGB_Magnet(); //品红灯长亮直到连接完成
	}
}

//****************************************C
// 8 读取加速度的原始数据
void MPU9250_AccRead(s16 *accData)
{
    u8 buf[6];
   	MPU9250_ReadLengthBytes(MPU9250_RA_ACCEL_XOUT_H, 6, buf);
    accData[0] = (s16)((buf[0] << 8) | buf[1]); //buf[0]左移8位，变成16位数据，再加buf[1]，之后强转得到s16的accData_x
    accData[1] = (s16)((buf[2] << 8) | buf[3]);
    accData[2] = (s16)((buf[4] << 8) | buf[5]);
}

// 9 读取陀螺仪的原始数据
void MPU9250_GyroRead(s16 *gyroData)
{
    u8 buf[6];
	
	MPU9250_ReadLengthBytes(MPU9250_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (s16)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (s16)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (s16)((buf[4] << 8) | buf[5]) ;
}

// 10 读取温度值
void MPU9250_TempRead(float *tempdata)
{
	u8 buf[2];
	short data;
	MPU9250_ReadLengthBytes(MPU9250_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;
	*tempdata = 36.53f + ((float)data/340.0f); //36.53f：把36.53当成浮点数处理
}

//****************************************C
// 11 初始化MPU9250进入工作状态
void MPU9250_Init(void)
{
	//1 检查MCU与MPU9250通讯是否正常
	MPU9250_Check(); 
	//2 设置MPU9250的复位状态+100ms复位延时
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x80); //reg107 电源管理1寄存器
	delay_ms(100);
	//3 唤醒MPU9250并选择PLL为时钟源
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x01); //reg同上
	//4 配置6轴数据全输出
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_2, 0x00); //reg108 电源管理2寄存器
	//5 禁用所有中断
	MPU9250_WriteByte(MPU9250_RA_INT_ENABLE, 0x00); //reg56 中断使能/旁路引脚寄存器
	//6 设置MPU9250为内部采样频率
	MPU9250_WriteByte(MPU9250_RA_SMPLRT_DIV, 0x00); //reg25 采样分频
	//采样频率=陀螺仪输出频率/(1+SMPLRT_DIV)
	//四轴姿态解算频率取500Hz，gyro输出频率取1kHz，而采样频率不能低于500Hz，因此不分频，0x00
	//7 设置陀螺仪与加速度计满量程范围 -> s16 -32768~+32768 不同的满量程范围导致不同的灵敏度
	MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG, 0x08); //加速度计满量程   +-4g		 最低分辨率=2^15/4g=8192LSB/g	即灵敏度
	MPU9250_WriteByte(MPU9250_RA_GYRO_CONFIG, 0x18);  //陀螺仪满量程     +-2000°/s   最低分辨率=2^15/4000=16.4LSB/°/s
	//8 设置陀螺仪与加速度计输出低通滤波
	MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG_2, MPU9250_DLPF_BW_20); //设置加速度计滤波为DLPF=20Hz，0x04
	MPU9250_WriteByte(MPU9250_RA_CONFIG, MPU9250_DLPF_BW_98); //设置陀螺仪输出为1kHz，DLPF=98Hz，0x02
}

//****************************************D
// 12 陀螺仪与加速度计校准
void MPU9250_CalOff(void)
{
	SENSOR_FLAG_SET(GYRO_OFFSET); //陀螺仪校准 
	SENSOR_FLAG_SET(ACC_OFFSET);  //加速度计校准
}

// 13 陀螺仪校准
void MPU9250_CalOff_Gyro(void)
{
	SENSOR_FLAG_SET(GYRO_OFFSET); //陀螺仪校准
}

// 14 加速度计校准
void MPU9250_CalOff_Acc(void)
{
	SENSOR_FLAG_SET(ACC_OFFSET);  //加速度校准
}

// 15 读取加速度与陀螺仪的原始数据
void MPU9250_Read(void)
{
	MPU9250_ReadLengthBytes(MPU9250_RA_ACCEL_XOUT_H, 14, MPU9250_buff);//查询法读取MPU9250的原始数据 14x8bit
}

// 16 MPU9250零点漂移/零偏校准
u8 MPU9250_ZeroDriftCalib(INT16_XYZ value, INT16_XYZ *offset, u16 sensitivity) //value: MPU9250原始数据, offset: 校准后零偏移值, sensitivity: 加速度计灵敏度=8192 (陀螺仪的=0)
{
	static s32 tempgx=0, tempgy=0, tempgz=0; 
	static u16 cnt_a=0; //静态局部变量，表明此变量具有静态存储周期，也就是说该函数执行完不释放内存
	if(cnt_a==0) //cnt_a==0，清零所有数据 准备零点漂移校准
	{
		value.X=0;
		value.Y=0;
		value.Z=0; //MPU9250原始数据
		tempgx=0;
		tempgy=0;
		tempgz=0;
		cnt_a=1;
		sensitivity=0;
		offset->X=0;
		offset->Y=0;
		offset->Z=0; 
	}
	tempgx+=value.X;
	tempgy+=value.Y; 
	tempgz+=value.Z-sensitivity; //sensitivity加速度计灵敏度=MPU9250初始化时设置的灵敏度(8192LSB/g); 陀螺仪校准sensitivity = 0; ????????????????????????????
	if(cnt_a==200) //200个数值取平均
	{
		offset->X=tempgx/cnt_a;
		offset->Y=tempgy/cnt_a;
		offset->Z=tempgz/cnt_a; //offset 校准后零偏移值
		cnt_a = 0;
		return 1; // 1 校准完成
	}
	cnt_a++; //函数调取200次才完成1次校准，但在main函数中，Prepare_Data()函数的调取为100Hz，所以无妨
	return 0; // 0 校准未完成	
}

// 17 MPU9250去零点漂移处理
void MPU9250_ZeroDriftRemoval(void)
{
	//加速度计去零偏AD值 
	MPU9250_ACC_RAW.X =((((s16)MPU9250_buff[0]) << 8) | MPU9250_buff[1]) - ACC_OFFSET_RAW.X; //ACC的X轴原始数据-零点漂移数据
	MPU9250_ACC_RAW.Y =((((s16)MPU9250_buff[2]) << 8) | MPU9250_buff[3]) - ACC_OFFSET_RAW.Y;
	MPU9250_ACC_RAW.Z =((((s16)MPU9250_buff[4]) << 8) | MPU9250_buff[5]) - ACC_OFFSET_RAW.Z;
	//陀螺仪去零偏AD值
	MPU9250_GYRO_RAW.X =((((s16)MPU9250_buff[8]) << 8) | MPU9250_buff[9]) - GYRO_OFFSET_RAW.X;
	MPU9250_GYRO_RAW.Y =((((s16)MPU9250_buff[10]) << 8) | MPU9250_buff[11]) - GYRO_OFFSET_RAW.Y;
	MPU9250_GYRO_RAW.Z =((((s16)MPU9250_buff[12]) << 8) | MPU9250_buff[13]) - GYRO_OFFSET_RAW.Z;
	
	if(GET_FLAG(GYRO_OFFSET)) //陀螺仪进行零漂校准 怎么置1的? structconfig.h直接宏定义了1 
	{
		if(MPU9250_ZeroDriftCalib(MPU9250_GYRO_RAW, &GYRO_OFFSET_RAW, 0))
		{
			SENSOR_FLAG_RESET(GYRO_OFFSET); //置0
			PID_WriteFlash(); //保存陀螺仪的零漂数据 写到paramsave中
			RGB_GYRO_Calib(); //蓝灯闪5次
			SENSOR_FLAG_SET(ACC_OFFSET); //置1 校准加速度计
//			printf("GYRO_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",GYRO_OFFSET_RAW.X,GYRO_OFFSET_RAW.Y,GYRO_OFFSET_RAW.Z);
//			printf("\n");
		}
	}
	if(GET_FLAG(ACC_OFFSET)) //加速度计进行零漂校准
	{
		if(MPU9250_ZeroDriftCalib(MPU9250_ACC_RAW, &ACC_OFFSET_RAW, 8192)) //sensitivity=8192
		{
			SENSOR_FLAG_RESET(ACC_OFFSET); //置0
			PID_WriteFlash(); //保存加速度计的零漂数据
			RGB_ACC_Calib(); //青灯闪5次
//			printf("ACC_OFFSET_RAW Value X=%d  Y=%d  Z=%d\n",ACC_OFFSET_RAW.X,ACC_OFFSET_RAW.Y,ACC_OFFSET_RAW.Z); 
//			printf("\n");
		}
	}
}
