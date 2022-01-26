/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/09/20
��    ����V1.0
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
	PidParameter_init(); //PID��ʼ��
	RGB_Cyan(); //��ʼ�����
	
	while(1)
	{
		if(ANO_Scan) //500Hz
		{
			ANO_Scan=0;
			ANO_DT_Data_Exchange(); //������λ������
		}
		if(IMU_Scan) //100Hz
		{
			IMU_Scan=0;
			Prepare_Data(); //��ȡ��̬�������� �����ڵ�printfע�͵� ��֤��λ��ͨѶ�����룡������
			IMUupdate(&Gyro_rad, &Acc_filt, &Att_Angle); //��Ԫ����̬����
			Control(&Att_Angle, &Gyro_rad, &RC_Control, Airplane_Enable); //��̬����
		}
		if(LED_Scan) //10Hz
		{
			LED_Scan=0;
			LED_Run();
			if(!Airplane_Enable && Run_flag && !WiFi_LEDflag)
			{
				RGB_Rand(); //�ɻ�����״̬��
			}
			RGB_BAT_Alarm(); //��ص͵�ѹ����
		}
		if(IRQ_Scan) //5Hz
		{
			IRQ_Scan=0;
			NRF_SingalCheck(); //NRF�ź��жϼ��
			SendToRemote(); //�������ݸ�ң����
		}
		if(BAT_Scan) //2.5Hz
		{
			BAT_Scan=0;
			NRF_GetAddr(); //����NRF��ַ
			BAT_LowVoltageAlarm(); //�͵������� ���ܵƹ�flag
		}
	}
}


////********************************************************************���Բ���
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
//		//2 LED����
//		LED_Run(); //����һ�ι�����
//		
//		//1 USART����
//		printf("Hello world!\r\n");
//		
//		//4 IICͨѶ����
//		IIC_ReadOneByte(0x69, 0x75, &ID);
//		printf("MPU9250 WHO AM I: 0x%x\r\n", ID);
//		//5 MPU9250����
//		MPU9250_GyroRead(GYRO);
//		MPU9250_AccRead(ACC);
//		MPU9250_TempRead(&Temperature);
//		printf("GYRO_X:%d	GYRO_Y:%d	GYRO_Z:%d\r\n", GYRO[0], GYRO[1], GYRO[2]);
//		printf("ACC_X: %d	ACC_Y: %d	ACC_Z: %d\r\n", ACC[0], ACC[1], ACC[2]);
//		printf("Temperature: %0.2f\r\n", Temperature);
//		//9 ��ȡ��̬�����������ݵĲ���
//		Prepare_Data(); //֮������ʮ�����Ҳű���������Ϊ�˲��������12�����ݲſ����ܣ�
//		//10 ��Ԫ����̬����
//		IMUupdate(&Gyro_rad, &Acc_filt, &Att_Angle); //���������������ݾ�׼��
//		
//		//6 SPI&NRF24L01ͨѶ����
//		//д����0x03��Si24R1�Ĵ���
//		NRF_CSN=0; //���Ա�����NRF_CSN��ʹ�ܣ���nrf24l01.c�У�������
//		SPI2_WriteReadByte(0x20+0x01); //0x20: д���üĴ���  0x01: �Ĵ�����ַ
//		SPI2_WriteReadByte(0x03); //0x03: data 
//		NRF_CSN=1; 
//		//��Si24R1�Ĵ���������
//		NRF_CSN=0; 
//		SPI2_WriteReadByte(0x01); //0x01: �Ĵ�����ַ
//		data=SPI2_WriteReadByte(0xff); //0xff: NOP(ʵ����λ) ִ�ж����� ��д�����
//		NRF_CSN=1; 
//		printf("NRF data: 0x%x\r\n", data);
//		
//		//12 ң�������Ӳ��� ͬ�� Scan���ʱ�������
//		NRF_GetAddr();
//		SendToRemote(); //�������ݸ�ң����
//		
//		//8 ��ȡ��ص�ѹ����
//		BAT_GetVoltage();
//		
//		//13 ��λ��ͨѶ����
//		ANO_DT_Data_Exchange();
//		
//		printf("\r\n");
//		
//		//3 RGB����
//		RGB_Fly();
////		delay_ms(1000);
//		RGB_Off();
////		delay_ms(1000);
//		
//		//7 �������
////		Motor_Pwm(100, 0, 0, 0); //���Ͻǵ�1�ŵ��ת ע�ⰲȫ��������
//		
//	}
//}
