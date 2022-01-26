/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/26
�����ܣ�MPU9250��غ���
��    ע��
*******************************************************************************/
#include "mpu9250.h"
#include "iic.h"
#include "stdio.h"
#include "rgb.h"
#include "delay.h"
#include "structconfig.h"
#include "paramsave.h"

static u8    MPU9250_buff[14];					//���ٶȼ�(6) ������(6) �¶�(2) ��ԭʼ���� 14x8bit
INT16_XYZ	 GYRO_OFFSET_RAW, ACC_OFFSET_RAW;	//���Ư������
INT16_XYZ	 MPU9250_ACC_RAW, MPU9250_GYRO_RAW;	//��ȡֵԭʼ����
u8   		 SENSOR_OFFSET_FLAG;	            //������У׼��־λ

//****************************************A ������ֲ��
// 1 ��һ���ֽ����ݵ�MPU9250�Ĵ���
u8 MPU9250_ReadByte(u8 reg, u8 *data)
{
	if(IIC_ReadOneByte(MPU9250Addr, reg, data))
	   return 1;
	else
	   return 0;
}

// 2 дһ���ֽ����ݵ�MPU9250�Ĵ���
u8 MPU9250_WriteByte(u8 reg, u8 data)
{
	if(IIC_WriteOneByte(MPU9250Addr, reg, data))
	   return 1;
	else
	   return 0;
}

// 3 ��ָ�������ֽ����ݵ�MPU9250�Ĵ���
u8 MPU9250_ReadLengthBytes(u8 reg, u8 len, u8 *data)
{
	if(IIC_ReadLengthBytes(MPU9250Addr, reg, len, data))
	   return 1;
	else
	   return 0;
}

// 4 дָ�������ֽ����ݵ�MPU9250�Ĵ���
u8 MPU9250_WriteLengthBytes(u8 reg, u8 len, u8 *data)
{
	if(IIC_WriteLengthBytes(MPU9250Addr, reg, len, data))
	   return 1;
	else
	   return 0;
}

//****************************************B ͨѶ���
// 5 ��ȡMPU9250��WHO_AM_I ���� 0x71(Ĭ��ֵ) 0x68(��λֵ)
u8 MPU9250_getDeviceID(void)
{
    u8 buf;
	MPU9250_ReadByte(MPU9250_RA_WHO_AM_I, &buf);
    return buf;
}

// 6 ���MPU9250�Ƿ��Ѿ�����
u8 MPU9250_TestConnection(void) 
{
	if(MPU9250_getDeviceID() == 0x71)  
		return 1;
	else 
		return 0;
}

// 7 ���IIC�����ϵ�MPU9250�Ƿ���� û������Ʒ���
 void MPU9250_Check(void) 
{ 
	while(!MPU9250_TestConnection())
	{
		printf("\rMPU9250 no connect...\r\n");
		RGB_Magnet(); //Ʒ��Ƴ���ֱ���������
	}
}

//****************************************C
// 8 ��ȡ���ٶȵ�ԭʼ����
void MPU9250_AccRead(s16 *accData)
{
    u8 buf[6];
   	MPU9250_ReadLengthBytes(MPU9250_RA_ACCEL_XOUT_H, 6, buf);
    accData[0] = (s16)((buf[0] << 8) | buf[1]); //buf[0]����8λ�����16λ���ݣ��ټ�buf[1]��֮��ǿת�õ�s16��accData_x
    accData[1] = (s16)((buf[2] << 8) | buf[3]);
    accData[2] = (s16)((buf[4] << 8) | buf[5]);
}

// 9 ��ȡ�����ǵ�ԭʼ����
void MPU9250_GyroRead(s16 *gyroData)
{
    u8 buf[6];
	
	MPU9250_ReadLengthBytes(MPU9250_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (s16)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (s16)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (s16)((buf[4] << 8) | buf[5]) ;
}

// 10 ��ȡ�¶�ֵ
void MPU9250_TempRead(float *tempdata)
{
	u8 buf[2];
	short data;
	MPU9250_ReadLengthBytes(MPU9250_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;
	*tempdata = 36.53f + ((float)data/340.0f); //36.53f����36.53���ɸ���������
}

//****************************************C
// 11 ��ʼ��MPU9250���빤��״̬
void MPU9250_Init(void)
{
	//1 ���MCU��MPU9250ͨѶ�Ƿ�����
	MPU9250_Check(); 
	//2 ����MPU9250�ĸ�λ״̬+100ms��λ��ʱ
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x80); //reg107 ��Դ����1�Ĵ���
	delay_ms(100);
	//3 ����MPU9250��ѡ��PLLΪʱ��Դ
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x01); //regͬ��
	//4 ����6������ȫ���
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_2, 0x00); //reg108 ��Դ����2�Ĵ���
	//5 ���������ж�
	MPU9250_WriteByte(MPU9250_RA_INT_ENABLE, 0x00); //reg56 �ж�ʹ��/��·���żĴ���
	//6 ����MPU9250Ϊ�ڲ�����Ƶ��
	MPU9250_WriteByte(MPU9250_RA_SMPLRT_DIV, 0x00); //reg25 ������Ƶ
	//����Ƶ��=���������Ƶ��/(1+SMPLRT_DIV)
	//������̬����Ƶ��ȡ500Hz��gyro���Ƶ��ȡ1kHz��������Ƶ�ʲ��ܵ���500Hz����˲���Ƶ��0x00
	//7 ��������������ٶȼ������̷�Χ -> s16 -32768~+32768 ��ͬ�������̷�Χ���²�ͬ��������
	MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG, 0x08); //���ٶȼ�������   +-4g		 ��ͷֱ���=2^15/4g=8192LSB/g	��������
	MPU9250_WriteByte(MPU9250_RA_GYRO_CONFIG, 0x18);  //������������     +-2000��/s   ��ͷֱ���=2^15/4000=16.4LSB/��/s
	//8 ��������������ٶȼ������ͨ�˲�
	MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG_2, MPU9250_DLPF_BW_20); //���ü��ٶȼ��˲�ΪDLPF=20Hz��0x04
	MPU9250_WriteByte(MPU9250_RA_CONFIG, MPU9250_DLPF_BW_98); //�������������Ϊ1kHz��DLPF=98Hz��0x02
}

//****************************************D
// 12 ����������ٶȼ�У׼
void MPU9250_CalOff(void)
{
	SENSOR_FLAG_SET(GYRO_OFFSET); //������У׼ 
	SENSOR_FLAG_SET(ACC_OFFSET);  //���ٶȼ�У׼
}

// 13 ������У׼
void MPU9250_CalOff_Gyro(void)
{
	SENSOR_FLAG_SET(GYRO_OFFSET); //������У׼
}

// 14 ���ٶȼ�У׼
void MPU9250_CalOff_Acc(void)
{
	SENSOR_FLAG_SET(ACC_OFFSET);  //���ٶ�У׼
}

// 15 ��ȡ���ٶ��������ǵ�ԭʼ����
void MPU9250_Read(void)
{
	MPU9250_ReadLengthBytes(MPU9250_RA_ACCEL_XOUT_H, 14, MPU9250_buff);//��ѯ����ȡMPU9250��ԭʼ���� 14x8bit
}

// 16 MPU9250���Ư��/��ƫУ׼
u8 MPU9250_ZeroDriftCalib(INT16_XYZ value, INT16_XYZ *offset, u16 sensitivity) //value: MPU9250ԭʼ����, offset: У׼����ƫ��ֵ, sensitivity: ���ٶȼ�������=8192 (�����ǵ�=0)
{
	static s32 tempgx=0, tempgy=0, tempgz=0; 
	static u16 cnt_a=0; //��̬�ֲ������������˱������о�̬�洢���ڣ�Ҳ����˵�ú���ִ���겻�ͷ��ڴ�
	if(cnt_a==0) //cnt_a==0�������������� ׼�����Ư��У׼
	{
		value.X=0;
		value.Y=0;
		value.Z=0; //MPU9250ԭʼ����
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
	tempgz+=value.Z-sensitivity; //sensitivity���ٶȼ�������=MPU9250��ʼ��ʱ���õ�������(8192LSB/g); ������У׼sensitivity = 0; ????????????????????????????
	if(cnt_a==200) //200����ֵȡƽ��
	{
		offset->X=tempgx/cnt_a;
		offset->Y=tempgy/cnt_a;
		offset->Z=tempgz/cnt_a; //offset У׼����ƫ��ֵ
		cnt_a = 0;
		return 1; // 1 У׼���
	}
	cnt_a++; //������ȡ200�β����1��У׼������main�����У�Prepare_Data()�����ĵ�ȡΪ100Hz�������޷�
	return 0; // 0 У׼δ���	
}

// 17 MPU9250ȥ���Ư�ƴ���
void MPU9250_ZeroDriftRemoval(void)
{
	//���ٶȼ�ȥ��ƫADֵ 
	MPU9250_ACC_RAW.X =((((s16)MPU9250_buff[0]) << 8) | MPU9250_buff[1]) - ACC_OFFSET_RAW.X; //ACC��X��ԭʼ����-���Ư������
	MPU9250_ACC_RAW.Y =((((s16)MPU9250_buff[2]) << 8) | MPU9250_buff[3]) - ACC_OFFSET_RAW.Y;
	MPU9250_ACC_RAW.Z =((((s16)MPU9250_buff[4]) << 8) | MPU9250_buff[5]) - ACC_OFFSET_RAW.Z;
	//������ȥ��ƫADֵ
	MPU9250_GYRO_RAW.X =((((s16)MPU9250_buff[8]) << 8) | MPU9250_buff[9]) - GYRO_OFFSET_RAW.X;
	MPU9250_GYRO_RAW.Y =((((s16)MPU9250_buff[10]) << 8) | MPU9250_buff[11]) - GYRO_OFFSET_RAW.Y;
	MPU9250_GYRO_RAW.Z =((((s16)MPU9250_buff[12]) << 8) | MPU9250_buff[13]) - GYRO_OFFSET_RAW.Z;
	
	if(GET_FLAG(GYRO_OFFSET)) //�����ǽ�����ƯУ׼ ��ô��1��? structconfig.hֱ�Ӻ궨����1 
	{
		if(MPU9250_ZeroDriftCalib(MPU9250_GYRO_RAW, &GYRO_OFFSET_RAW, 0))
		{
			SENSOR_FLAG_RESET(GYRO_OFFSET); //��0
			PID_WriteFlash(); //���������ǵ���Ư���� д��paramsave��
			RGB_GYRO_Calib(); //������5��
			SENSOR_FLAG_SET(ACC_OFFSET); //��1 У׼���ٶȼ�
//			printf("GYRO_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",GYRO_OFFSET_RAW.X,GYRO_OFFSET_RAW.Y,GYRO_OFFSET_RAW.Z);
//			printf("\n");
		}
	}
	if(GET_FLAG(ACC_OFFSET)) //���ٶȼƽ�����ƯУ׼
	{
		if(MPU9250_ZeroDriftCalib(MPU9250_ACC_RAW, &ACC_OFFSET_RAW, 8192)) //sensitivity=8192
		{
			SENSOR_FLAG_RESET(ACC_OFFSET); //��0
			PID_WriteFlash(); //������ٶȼƵ���Ư����
			RGB_ACC_Calib(); //�����5��
//			printf("ACC_OFFSET_RAW Value X=%d  Y=%d  Z=%d\n",ACC_OFFSET_RAW.X,ACC_OFFSET_RAW.Y,ACC_OFFSET_RAW.Z); 
//			printf("\n");
		}
	}
}
