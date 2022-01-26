/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/28
程序功能：结构体定义
备    注：参考小马哥四轴structconfig.h
*******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H
#include "stm32f4xx.h"
#include "stdio.h"
#include "nrf24l01.h"

//****************************************A PID参数调试上位机波形显示
//如果宏定义WIFI_DEBUG开启无线调参，否则就是有线调参					仅用于usart.c中！！
//#define WIFI_DEBUG 		//没宏定义完成相当于还是用下载器调参
//如果想观察哪个轴的PID波形，就取消掉哪个注释，但一次只能取消掉一个注释
//#define ROL_PID_DEBUG    
//#define PIT_PID_DEBUG
//#define YAW_PID_DEBUG
//#define ALT_PID_DEBUGE //已取消在ANO_DT.c中的相关设置！！！！

//****************************************B 飞机状态记录组 SENSOR_OFFSET_FLAG
//标志位组
extern u8 SENSOR_OFFSET_FLAG; //传感器校准标志位，定义在mpu9250.c里！

//每一位对应功能
#define GYRO_OFFSET 0x01  //第一位 陀螺仪校准标志位
#define ACC_OFFSET 	0x02  //第二位 加速度计校准标志位
#define BAR_OFFSET 	0x04  //第三位 气压计校准标志位
#define MAG_OFFSET 	0x08  //第四位 磁力计校准标志位
#define FLY_ENABLE	0x10  //第五位 解锁上锁
#define WiFi_ONOFF 	0x20  //第六位 Wifi开关
#define FLY_MODE   	0x40  //第七位 模式选择(0无头模式(默认) 1有头模式)

//对SENSOR_OFFSET_FLAG的位操作
#define SENSOR_FLAG_SET(FLAG)    SENSOR_OFFSET_FLAG|=FLAG            //标志位置1
#define SENSOR_FLAG_RESET(FLAG)  SENSOR_OFFSET_FLAG&=~FLAG           //标志位置0
#define GET_FLAG(FLAG)          (SENSOR_OFFSET_FLAG&FLAG)==FLAG?1:0  //获取标志位状态

//****************************************C 相关结构体定义
//三轴整形数据结构(MPU9250原始数据)
typedef struct
{
	s16 X;
	s16 Y;
	s16 Z;
}INT16_XYZ; //用于mpu9250 filter

//三轴浮点型
typedef struct
{
	float X;
	float Y;
	float Z;
}FLOAT_XYZ; //用于filter imu control

//姿态结算后的角度
typedef struct
{
	float rol;
	float pit;
	float yaw;
}FLOAT_ANGLE; //用于imu control ANO_DT

//数据拆分宏定义，在发送大于1字节的数据类型时，如int16、float等，需要把数据拆分成单独字节进行发送
#define Byte0(data)       ( *( (char *)(&data)	  ) ) //用于remotedata中
#define Byte1(data)       ( *( (char *)(&data) + 1) )
#define Byte2(data)       ( *( (char *)(&data) + 2) )
#define Byte3(data)       ( *( (char *)(&data) + 3) )
 
//遥控器的数据结构
typedef struct
{
	s16 ROLL;
	s16 PITCH;
	s16 THROTTLE;
	s16 YAW;
}RC_TYPE; //用于control remotedata

//PID算法的数据结构
typedef struct PID
{
	float P;         //参数
	float I;
	float D;
	float Error;     //比例项
	float Integral;  //积分项
	float Differ;    //微分项
	float PreError;
	float PrePreError;
	float Ilimit; 
//  float MotoCompensation; //电机补偿 
	float Irange;
	u8 Ilimit_flag;    //积分分离
	float Pout;
	float Iout;
	float Dout;
	float OutPut;      
}PID_TYPE; //用于pid control

//保存参数的数据结构
typedef struct PIDSave
{
	//陀螺仪校准数据 
	u16  ACC_OFFSET_X;
	u16  ACC_OFFSET_Y;
	u16  ACC_OFFSET_Z;
	u16  GYRO_OFFSET_X;
	u16  GYRO_OFFSET_Y;
	u16  GYRO_OFFSET_Z;
	//角度环
	u16  ROL_Angle_P;
	u16  ROL_Angle_I;
	u16  ROL_Angle_D;
	u16  PIT_Angle_P;
	u16  PIT_Angle_I;
	u16  PIT_Angle_D;
	u16  YAW_Angle_P;
	u16  YAW_Angle_I;
	u16  YAW_Angle_D;
	//角速度环
	u16  ROL_Rate_P;
	u16  ROL_Rate_I;
	u16  ROL_Rate_D;
	u16  PIT_Rate_P;
	u16  PIT_Rate_I;
	u16  PIT_Rate_D;
	u16  YAW_Rate_P;
	u16  YAW_Rate_I;
	u16  YAW_Rate_D;
	//NRF的随机地址
	u16  NRFaddr;
}PID_SAVE; //用于paramsave

//电池电压管理数据结构
typedef struct BAT_TYPE
{
	float BattAdc;
	float BattRealV;
	float BattMeasureV;
	float BattAlarmV;
	float BattFullV;
}BAT_TYPE; //用于power.c

//****************************************D 总结声明全局变量
//姿态解算
extern INT16_XYZ MPU9250_ACC_RAW, MPU9250_GYRO_RAW;	//MPU最新一次原始数据 						定义在mpu9250.c中
extern INT16_XYZ GYRO_OFFSET_RAW, ACC_OFFSET_RAW;	//MPU零点漂移值 	  						定义在mpu9250.c中
extern FLOAT_XYZ Gyro_rad, Gyro_rad_old;			//把陀螺仪各通道读出的数据转换成弧度制 		定义在imu.c中
extern FLOAT_XYZ Acc_filt, Gyro_filt, Acc_filt_old;	//滤波后的各通道数据(及其上一次采集的数据) 	定义在imu.c中
extern float DCMgb[3][3], accb[3]; //定义在imu.c中
//遥控数据缓存
extern u8 NRFAddr;
extern float Motor_PWM_1,Motor_PWM_2,Motor_PWM_3,Motor_PWM_4;
//角度环PID
extern PID_TYPE PID_ROL_Angle;
extern PID_TYPE PID_PIT_Angle;
extern PID_TYPE PID_YAW_Angle;
//角速度环PID
extern PID_TYPE PID_ROL_Rate;
extern PID_TYPE PID_PIT_Rate;
extern PID_TYPE PID_YAW_Rate;
//高度PID
extern PID_TYPE PID_ALT_Rate;
extern PID_TYPE PID_ALT;
//参数保存
extern uint8_t        InitDefaultParam; //初始化默认参数
extern PID_SAVE       PIDflash;   //各轴PID参数保存实例
extern FLOAT_ANGLE    Att_Angle;	//ATT函数计算出的姿态角
extern RC_TYPE        RC_Control;
//功能键
extern u8 Airplane_Enable, Run_flag; 
extern u8 RGB_BATflag;
extern u8 WiFi_LEDflag, WiFi_Controlflag;

extern uint8_t LED_Scan ;
extern uint8_t IMU_Scan ;
extern uint8_t MPU_Scan ;
extern uint8_t IRQ_Scan ;
extern uint8_t BAT_Scan;
extern uint8_t ANO_Scan ;

extern float THROTTLE;
extern BAT_TYPE BAT;

#endif
