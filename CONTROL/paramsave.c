/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/09/11
�����ܣ���������
*******************************************************************************/
#include "paramsave.h"
#include "structconfig.h"
#include "stmflash.h"
#include "stdio.h"

#define INT160(dwTemp)       (*(int16_t*)(&dwTemp))
#define INT161(dwTemp)       (*((int16_t *)(&dwTemp) + 1))
#define INT162(dwTemp)       (*((int16_t *)(&dwTemp) + 2))
#define INT163(dwTemp)       (*((int16_t *)(&dwTemp) + 3))

PID_SAVE PIDflash;

// 1 ��Ҫ������ֵ��Flash�Ĵ洢�ṹ��
void ParamsToTable(void) //����Flash�Ĵ洢�Ľṹ��Ϊu16������PID����Ϊfloat������λ����
{						 //���PID����������1000Ȼ��ǿתΪu16
	//��ƫ����
	PIDflash.ACC_OFFSET_X = ACC_OFFSET_RAW.X;
	PIDflash.ACC_OFFSET_Y = ACC_OFFSET_RAW.Y;
	PIDflash.ACC_OFFSET_Z = ACC_OFFSET_RAW.Z; 
	PIDflash.GYRO_OFFSET_X = GYRO_OFFSET_RAW.X;
	PIDflash.GYRO_OFFSET_Y = GYRO_OFFSET_RAW.Y;
	PIDflash.GYRO_OFFSET_Z = GYRO_OFFSET_RAW.Z;
	
	//�ǶȻ�����PID����
	PIDflash.ROL_Angle_P = (u16)(PID_ROL_Angle.P*1000); //PIDflash��Ĳ�����control.c�е�PID_TYPE�Ĳ������壬�Ѿ������ϸ������
	PIDflash.ROL_Angle_I = (u16)(PID_ROL_Angle.I*1000);
	PIDflash.ROL_Angle_D = (u16)(PID_ROL_Angle.D*1000);
	PIDflash.PIT_Angle_P = (u16)(PID_PIT_Angle.P*1000);
	PIDflash.PIT_Angle_I = (u16)(PID_PIT_Angle.I*1000);
	PIDflash.PIT_Angle_D = (u16)(PID_PIT_Angle.D*1000);
	PIDflash.YAW_Angle_P = (u16)(PID_YAW_Angle.P*1000);
	PIDflash.YAW_Angle_I = (u16)(PID_YAW_Angle.I*1000);
	PIDflash.YAW_Angle_D = (u16)(PID_YAW_Angle.D*1000);
	
	//���ٶȻ�PID����
	PIDflash.ROL_Rate_P  = (u16)(PID_ROL_Rate.P*1000);
	PIDflash.ROL_Rate_I  = (u16)(PID_ROL_Rate.I*1000);
	PIDflash.ROL_Rate_D  = (u16)(PID_ROL_Rate.D*1000);
	PIDflash.PIT_Rate_P  = (u16)(PID_PIT_Rate.P*1000);
	PIDflash.PIT_Rate_I  = (u16)(PID_PIT_Rate.I*1000);
	PIDflash.PIT_Rate_D  = (u16)(PID_PIT_Rate.D*1000);
	PIDflash.YAW_Rate_P  = (u16)(PID_YAW_Rate.P*1000);
	PIDflash.YAW_Rate_I  = (u16)(PID_YAW_Rate.I*1000);
	PIDflash.YAW_Rate_D  = (u16)(PID_YAW_Rate.D*1000);
	
	//�����ȡ��NRF��ַ����ֽ�
	PIDflash.NRFaddr = NRFAddr;
}

// 2 ��Flash�ж����Ĳ�����ֵ��ȫ�ֱ���
void TableToParams(void)
{
	//��ƫ����
	ACC_OFFSET_RAW.X = PIDflash.ACC_OFFSET_X;
	ACC_OFFSET_RAW.Y = PIDflash.ACC_OFFSET_Y;
	ACC_OFFSET_RAW.Z = PIDflash.ACC_OFFSET_Z;
	GYRO_OFFSET_RAW.X = PIDflash.GYRO_OFFSET_X;
	GYRO_OFFSET_RAW.Y = PIDflash.GYRO_OFFSET_Y;
	GYRO_OFFSET_RAW.Z = PIDflash.GYRO_OFFSET_Z;
	
	//�ǶȻ�����PID����
	PID_ROL_Angle.P = PIDflash.ROL_Angle_P/1000.0f;
	PID_ROL_Angle.I = PIDflash.ROL_Angle_I/1000.0f;
	PID_ROL_Angle.D = PIDflash.ROL_Angle_D/1000.0f;
	PID_PIT_Angle.P = PIDflash.PIT_Angle_P/1000.0f;
	PID_PIT_Angle.I = PIDflash.PIT_Angle_I/1000.0f;
	PID_PIT_Angle.D = PIDflash.PIT_Angle_D/1000.0f;
	PID_YAW_Angle.P = PIDflash.YAW_Angle_P/1000.0f;
	PID_YAW_Angle.I = PIDflash.YAW_Angle_I/1000.0f;
	PID_YAW_Angle.D = PIDflash.YAW_Angle_D/1000.0f;
		
	//���ٶȻ�PID����	
	PID_ROL_Rate.P = PIDflash.ROL_Rate_P/1000.0f;
	PID_ROL_Rate.I = PIDflash.ROL_Rate_I/1000.0f;
	PID_ROL_Rate.D = PIDflash.ROL_Rate_D/1000.0f;
	PID_PIT_Rate.P = PIDflash.PIT_Rate_P/1000.0f;
	PID_PIT_Rate.I = PIDflash.PIT_Rate_I/1000.0f;
	PID_PIT_Rate.D = PIDflash.PIT_Rate_D/1000.0f;
	PID_YAW_Rate.P = PIDflash.YAW_Rate_P/1000.0f;
	PID_YAW_Rate.I = PIDflash.YAW_Rate_I/1000.0f;
	PID_YAW_Rate.D = PIDflash.YAW_Rate_D/1000.0f;
	
	//�����ȡ��NRF��ַ����ֽ�
	NRFAddr = PIDflash.NRFaddr;
}

// 3 Ĭ�ϲ���
void DefaultParams(void) //���������Flash�е����ݶ������ˣ����ô˺������³�ʼ����д��flash
{          
	//��ƫ����
	PIDflash.ACC_OFFSET_X  = 0;
	PIDflash.ACC_OFFSET_Y  = 0;
	PIDflash.ACC_OFFSET_Z  = 0;
	PIDflash.GYRO_OFFSET_X = 0;
	PIDflash.GYRO_OFFSET_Y = 0;
	PIDflash.GYRO_OFFSET_Z = 0;
	
	//�ǶȻ�PID����
	PIDflash.ROL_Angle_P = 1800; //1800
	PIDflash.ROL_Angle_I = 10;	 //8
	PIDflash.ROL_Angle_D = 50;	 //50
	PIDflash.PIT_Angle_P = 1800; //1800
	PIDflash.PIT_Angle_I = 10;	 //8
	PIDflash.PIT_Angle_D = 50; 	 //50
	PIDflash.YAW_Angle_P = 3500; //3500
	PIDflash.YAW_Angle_I = 50;	 //0
	PIDflash.YAW_Angle_D = 1000; //1000
	
	//���ٶȻ�PID����
	PIDflash.ROL_Rate_P  = 1000;  //930
	PIDflash.ROL_Rate_I  = 15;	 //5
	PIDflash.ROL_Rate_D  = 900;	 //860
	PIDflash.PIT_Rate_P  = 1000;  //930
	PIDflash.PIT_Rate_I  = 15;    //5
	PIDflash.PIT_Rate_D  = 900;  //860
	PIDflash.YAW_Rate_P  = 2000; //2000
	PIDflash.YAW_Rate_I  = 50;   //50
	PIDflash.YAW_Rate_D  = 1000; //1000
}

// 4 ��flash�洢�ṹ������ȫ������
void ParamsClearAll(void)
{
	//��ƫ����
	PIDflash.ACC_OFFSET_X = 0;
	PIDflash.ACC_OFFSET_Y = 0;
	PIDflash.ACC_OFFSET_Z = 0;
	PIDflash.GYRO_OFFSET_X = 0;
	PIDflash.GYRO_OFFSET_Y = 0;
	PIDflash.GYRO_OFFSET_Z = 0;
	
	//�ǶȻ�����PID����
	PIDflash.ROL_Angle_P = 0;
	PIDflash.ROL_Angle_I = 0;
	PIDflash.ROL_Angle_D = 0;
	PIDflash.PIT_Angle_P = 0;
	PIDflash.PIT_Angle_I = 0;
	PIDflash.PIT_Angle_D = 0;
	PIDflash.YAW_Angle_P = 0;
	PIDflash.YAW_Angle_I = 0;
	PIDflash.YAW_Angle_D = 0;

	//���ٶȻ�PID����	
	PIDflash.ROL_Rate_P = 0;
	PIDflash.ROL_Rate_I = 0; 
	PIDflash.ROL_Rate_D = 0;
	PIDflash.PIT_Rate_P = 0; 
	PIDflash.PIT_Rate_I = 0;
	PIDflash.PIT_Rate_D = 0; 
	PIDflash.YAW_Rate_P = 0;
	PIDflash.YAW_Rate_I = 0; 
	PIDflash.YAW_Rate_D = 0; 
	
	//�����ȡ��NRF��ַ����ֽ�
	PIDflash.NRFaddr = 0;
}

// 5 ��flash�еĴ洢������Ԫ���ݶ�����
void PID_ClearFlash(void)
{
	u8 size,len;
	ParamsClearAll(); //�������
	len = sizeof(PIDflash);
	size = len/(4+(len%4)?1:0); //��������ݳ���
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
}

// 6 ����Ҫ�Ĳ���д��flash
void PID_WriteFlash(void)
{
	u8 size,len;
	ParamsToTable(); //������ת��������
	len = sizeof(PIDflash);
	size = len/(4+(len%4)?1:0); //��������ݳ���
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
}

// 7 ��flash��ȡ����
void PID_ReadFlash(void)
{
	u8 size,len;
	len =  sizeof(PIDflash);
	size = len/(4+(len%4)?1:0);//��������ݳ���
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
	TableToParams();//������ת���ɸ�����
	//��ȡ����ʧ��
	if(PIDflash.PIT_Rate_P ==0xFFFF && PIDflash.ROL_Rate_P==0xFFFF && PIDflash.YAW_Rate_P==0xFFFF)
	{
		DefaultParams_WriteFlash();
//		printf("Flash Read Error!!!\r\n");
	}
	else
	{
//		printf("Flash Read OK!!!\r\n");
	}
	/*
	printf("\r***************PID Angle loop parameter****************\r\n");
	printf("\rPID_ROL_Angle.P = %0.2f\r\n",PID_ROL_Angle.P);
	printf("\rPID_ROL_Angle.I = %0.2f\r\n",PID_ROL_Angle.I);
	printf("\rPID_ROL_Angle.D = %0.2f\r\n",PID_ROL_Angle.D);
	printf("\rPID_PIT_Angle.P = %0.2f\r\n",PID_PIT_Angle.P);
	printf("\rPID_PIT_Angle.I = %0.2f\r\n",PID_PIT_Angle.I);
	printf("\rPID_PIT_Angle.D = %0.2f\r\n",PID_PIT_Angle.D);
	printf("\rPID_YAW_Angle.P = %0.2f\r\n",PID_YAW_Angle.P);
	printf("\rPID_YAW_Angle.I = %0.2f\r\n",PID_YAW_Angle.I);
	printf("\rPID_YAW_Angle.D = %0.2f\r\n",PID_YAW_Angle.D);
	printf("\r***************PID AngleRate loop parameter**************\r\n");
	printf("\rPID_ROL_Rate.P = %0.2f\r\n",PID_ROL_Rate.P );
	printf("\rPID_ROL_Rate.I = %0.2f\r\n",PID_ROL_Rate.I );
	printf("\rPID_ROL_Rate.D = %0.2f\r\n",PID_ROL_Rate.D );
	printf("\rPID_PIT_Rate.P = %0.2f\r\n",PID_PIT_Rate.P );
	printf("\rPID_PIT_Rate.I = %0.2f\r\n",PID_PIT_Rate.I );
	printf("\rPID_PIT_Rate.D = %0.2f\r\n",PID_PIT_Rate.D );
	printf("\rPID_YAW_Rate.P = %0.2f\r\n",PID_YAW_Rate.P );
	printf("\rPID_YAW_Rate.I = %0.2f\r\n",PID_YAW_Rate.I );
	printf("\rPID_YAW_Rate.D = %0.2f\r\n",PID_YAW_Rate.D );
//	printf("\rNRFaddr = 0x%x\r\n", NRFAddr);
	*/
}

// 8 ��Ĭ��PID����д��flash
void DefaultParams_WriteFlash(void)
{
	u8 size,len;
	DefaultParams();//��ʼ��Ĭ�ϲ���
	len =  sizeof(PIDflash);
	size = len/(4+(len%4)?1:0); //��������ݳ���
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
}
