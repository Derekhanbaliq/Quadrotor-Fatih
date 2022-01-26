/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/09/20
版    本：V1.0
*******************************************************************************/
#include "stdio.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "led.h"		
#include "rgb.h"
#include "nvic.h"
#include "exti.h"
#include "timer.h"
#include "iic.h"
#include "structconfig.h"
#include "mpu9250.h"
#include "imu.h"
#include "stmflash.h"
#include "motor.h"
#include "power.h"
#include "spi.h"
#include "nrf24l01.h"
#include "remotedata.h"
#include "paramsave.h"
#include "ANO_DT.h"
#include "pid.h"
#include "control.h"

int main(void)
{
	NVIC_Config();
	USART1_Init(460800);
	delay_Init();
	LED_Init();
	RGB_Init();
	EXTIT_Init();
	TIM4_Init();
	IIC_Init();
	MPU9250_Init();
	SPI2_Init();
	NRF24L01_Init();
	MOTOR_Init();
	BAT_Init();
	PID_ReadFlash();
	PidParameter_init(); //PID初始化
	RGB_Cyan(); //初始化完毕
	
	while(1)
	{
		if(ANO_Scan) //500Hz
		{
			ANO_Scan=0;
			ANO_DT_Data_Exchange(); //更新上位机数据
		}
		if(IMU_Scan) //100Hz
		{
			IMU_Scan=0;
			Prepare_Data(); //获取姿态解算数据 函数内的printf注释掉 保证上位机通讯无误码！！！！
			IMUupdate(&Gyro_rad, &Acc_filt, &Att_Angle); //四元数姿态解算
			Control(&Att_Angle, &Gyro_rad, &RC_Control, Airplane_Enable); //姿态控制
		}
		if(LED_Scan) //10Hz
		{
			LED_Scan=0;
			LED_Run();
			if(!Airplane_Enable && Run_flag && !WiFi_LEDflag)
			{
				RGB_Rand(); //飞机上锁状态的
			}
			RGB_BAT_Alarm(); //电池低电压报警
		}
		if(IRQ_Scan) //5Hz
		{
			IRQ_Scan=0;
			NRF_SingalCheck(); //NRF信号中断检测
			SendToRemote(); //发送数据给遥控器
		}
		if(BAT_Scan) //2.5Hz
		{
			BAT_Scan=0;
			NRF_GetAddr(); //分配NRF地址
			BAT_LowVoltageAlarm(); //低电量报警 不管灯管flag
		}
	}
}


////********************************************************************测试部分
//#define NRF_CSN PBout(12)

//s16 GYRO[3], ACC[3];
//float Temperature;
////**********************************

//int main()
//{
//	u8 ID;
//	u8 data;
//	
//	NVIC_Config();
//	USART1_Init(460800); //460800
////	USART2_Init(921600);
//	delay_Init();
//	LED_Init();
//	RGB_Init();
//	EXTIT_Init();
////	TIM4_Init();
//	IIC_Init();
//	MPU9250_Init();
//	SPI2_Init();
//	NRF24L01_Init();
//	MOTOR_Init();
//	BAT_Init();
////	PID_ReadFlash();
//	
//	while(1)
//	{
//		//2 LED测试
//		LED_Run(); //亮灭一次共两秒
//		
//		//1 USART测试
//		printf("Hello world!\r\n");
//		
//		//4 IIC通讯测试
//		IIC_ReadOneByte(0x69, 0x75, &ID);
//		printf("MPU9250 WHO AM I: 0x%x\r\n", ID);
//		//5 MPU9250测试
//		MPU9250_GyroRead(GYRO);
//		MPU9250_AccRead(ACC);
//		MPU9250_TempRead(&Temperature);
//		printf("GYRO_X:%d	GYRO_Y:%d	GYRO_Z:%d\r\n", GYRO[0], GYRO[1], GYRO[2]);
//		printf("ACC_X: %d	ACC_Y: %d	ACC_Z: %d\r\n", ACC[0], ACC[1], ACC[2]);
//		printf("Temperature: %0.2f\r\n", Temperature);
//		//9 获取姿态解算所需数据的测试
//		Prepare_Data(); //之所以在十次左右才报数据是因为滤波必须得有12个数据才可以跑！
//		//10 四元数姿态解算
//		IMUupdate(&Gyro_rad, &Acc_filt, &Att_Angle); //采样速率上来数据就准了
//		
//		//6 SPI&NRF24L01通讯测试
//		//写数据0x03到Si24R1寄存器
//		NRF_CSN=0; //调试别忘了NRF_CSN的使能，在nrf24l01.c中！！！！
//		SPI2_WriteReadByte(0x20+0x01); //0x20: 写配置寄存器  0x01: 寄存器地址
//		SPI2_WriteReadByte(0x03); //0x03: data 
//		NRF_CSN=1; 
//		//从Si24R1寄存器读数据
//		NRF_CSN=0; 
//		SPI2_WriteReadByte(0x01); //0x01: 寄存器地址
//		data=SPI2_WriteReadByte(0xff); //0xff: NOP(实现移位) 执行读操作 先写后读！
//		NRF_CSN=1; 
//		printf("NRF data: 0x%x\r\n", data);
//		
//		//12 遥控器连接测试 同理 Scan速率必须上来
//		NRF_GetAddr();
//		SendToRemote(); //发送数据给遥控器
//		
//		//8 获取电池电压测试
//		BAT_GetVoltage();
//		
//		//13 上位机通讯测试
//		ANO_DT_Data_Exchange();
//		
//		printf("\r\n");
//		
//		//3 RGB测试
//		RGB_Fly();
////		delay_ms(1000);
//		RGB_Off();
////		delay_ms(1000);
//		
//		//7 电机测试
////		Motor_Pwm(100, 0, 0, 0); //左上角的1号电机转 注意安全！！！！
//		
//	}
//}
