/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/28
�����ܣ��ṹ�嶨��
��    ע���ο�С�������structconfig.h
*******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H
#include "stm32f4xx.h"
#include "stdio.h"
#include "nrf24l01.h"

//****************************************A PID����������λ��������ʾ
//����궨��WIFI_DEBUG�������ߵ��Σ�����������ߵ���					������usart.c�У���
//#define WIFI_DEBUG 		//û�궨������൱�ڻ���������������
//�����۲��ĸ����PID���Σ���ȡ�����ĸ�ע�ͣ���һ��ֻ��ȡ����һ��ע��
//#define ROL_PID_DEBUG    
//#define PIT_PID_DEBUG
//#define YAW_PID_DEBUG
//#define ALT_PID_DEBUGE //��ȡ����ANO_DT.c�е�������ã�������

//****************************************B �ɻ�״̬��¼�� SENSOR_OFFSET_FLAG
//��־λ��
extern u8 SENSOR_OFFSET_FLAG; //������У׼��־λ��������mpu9250.c�

//ÿһλ��Ӧ����
#define GYRO_OFFSET 0x01  //��һλ ������У׼��־λ
#define ACC_OFFSET 	0x02  //�ڶ�λ ���ٶȼ�У׼��־λ
#define BAR_OFFSET 	0x04  //����λ ��ѹ��У׼��־λ
#define MAG_OFFSET 	0x08  //����λ ������У׼��־λ
#define FLY_ENABLE	0x10  //����λ ��������
#define WiFi_ONOFF 	0x20  //����λ Wifi����
#define FLY_MODE   	0x40  //����λ ģʽѡ��(0��ͷģʽ(Ĭ��) 1��ͷģʽ)

//��SENSOR_OFFSET_FLAG��λ����
#define SENSOR_FLAG_SET(FLAG)    SENSOR_OFFSET_FLAG|=FLAG            //��־λ��1
#define SENSOR_FLAG_RESET(FLAG)  SENSOR_OFFSET_FLAG&=~FLAG           //��־λ��0
#define GET_FLAG(FLAG)          (SENSOR_OFFSET_FLAG&FLAG)==FLAG?1:0  //��ȡ��־λ״̬

//****************************************C ��ؽṹ�嶨��
//�����������ݽṹ(MPU9250ԭʼ����)
typedef struct
{
	s16 X;
	s16 Y;
	s16 Z;
}INT16_XYZ; //����mpu9250 filter

//���ḡ����
typedef struct
{
	float X;
	float Y;
	float Z;
}FLOAT_XYZ; //����filter imu control

//��̬�����ĽǶ�
typedef struct
{
	float rol;
	float pit;
	float yaw;
}FLOAT_ANGLE; //����imu control ANO_DT

//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ����int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define Byte0(data)       ( *( (char *)(&data)	  ) ) //����remotedata��
#define Byte1(data)       ( *( (char *)(&data) + 1) )
#define Byte2(data)       ( *( (char *)(&data) + 2) )
#define Byte3(data)       ( *( (char *)(&data) + 3) )
 
//ң���������ݽṹ
typedef struct
{
	s16 ROLL;
	s16 PITCH;
	s16 THROTTLE;
	s16 YAW;
}RC_TYPE; //����control remotedata

//PID�㷨�����ݽṹ
typedef struct PID
{
	float P;         //����
	float I;
	float D;
	float Error;     //������
	float Integral;  //������
	float Differ;    //΢����
	float PreError;
	float PrePreError;
	float Ilimit; 
//  float MotoCompensation; //������� 
	float Irange;
	u8 Ilimit_flag;    //���ַ���
	float Pout;
	float Iout;
	float Dout;
	float OutPut;      
}PID_TYPE; //����pid control

//������������ݽṹ
typedef struct PIDSave
{
	//������У׼���� 
	u16  ACC_OFFSET_X;
	u16  ACC_OFFSET_Y;
	u16  ACC_OFFSET_Z;
	u16  GYRO_OFFSET_X;
	u16  GYRO_OFFSET_Y;
	u16  GYRO_OFFSET_Z;
	//�ǶȻ�
	u16  ROL_Angle_P;
	u16  ROL_Angle_I;
	u16  ROL_Angle_D;
	u16  PIT_Angle_P;
	u16  PIT_Angle_I;
	u16  PIT_Angle_D;
	u16  YAW_Angle_P;
	u16  YAW_Angle_I;
	u16  YAW_Angle_D;
	//���ٶȻ�
	u16  ROL_Rate_P;
	u16  ROL_Rate_I;
	u16  ROL_Rate_D;
	u16  PIT_Rate_P;
	u16  PIT_Rate_I;
	u16  PIT_Rate_D;
	u16  YAW_Rate_P;
	u16  YAW_Rate_I;
	u16  YAW_Rate_D;
	//NRF�������ַ
	u16  NRFaddr;
}PID_SAVE; //����paramsave

//��ص�ѹ�������ݽṹ
typedef struct BAT_TYPE
{
	float BattAdc;
	float BattRealV;
	float BattMeasureV;
	float BattAlarmV;
	float BattFullV;
}BAT_TYPE; //����power.c

//****************************************D �ܽ�����ȫ�ֱ���
//��̬����
extern INT16_XYZ MPU9250_ACC_RAW, MPU9250_GYRO_RAW;	//MPU����һ��ԭʼ���� 						������mpu9250.c��
extern INT16_XYZ GYRO_OFFSET_RAW, ACC_OFFSET_RAW;	//MPU���Ư��ֵ 	  						������mpu9250.c��
extern FLOAT_XYZ Gyro_rad, Gyro_rad_old;			//�������Ǹ�ͨ������������ת���ɻ����� 		������imu.c��
extern FLOAT_XYZ Acc_filt, Gyro_filt, Acc_filt_old;	//�˲���ĸ�ͨ������(������һ�βɼ�������) 	������imu.c��
extern float DCMgb[3][3], accb[3]; //������imu.c��
//ң�����ݻ���
extern u8 NRFAddr;
extern float Motor_PWM_1,Motor_PWM_2,Motor_PWM_3,Motor_PWM_4;
//�ǶȻ�PID
extern PID_TYPE PID_ROL_Angle;
extern PID_TYPE PID_PIT_Angle;
extern PID_TYPE PID_YAW_Angle;
//���ٶȻ�PID
extern PID_TYPE PID_ROL_Rate;
extern PID_TYPE PID_PIT_Rate;
extern PID_TYPE PID_YAW_Rate;
//�߶�PID
extern PID_TYPE PID_ALT_Rate;
extern PID_TYPE PID_ALT;
//��������
extern uint8_t        InitDefaultParam; //��ʼ��Ĭ�ϲ���
extern PID_SAVE       PIDflash;   //����PID��������ʵ��
extern FLOAT_ANGLE    Att_Angle;	//ATT�������������̬��
extern RC_TYPE        RC_Control;
//���ܼ�
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
