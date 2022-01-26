/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/24
�����ܣ�IIC�������
��    ע����Ҫ�ο��Լ�ѧF1ʱд��myiic.c&h�����ֽ��С��������iic.c&h
*******************************************************************************/
#include "iic.h"
#include "math.h"
#include "nvic.h"
#include "delay.h"

//****************************************A
//Ϊ���ڹ۲죬�궨�岿��д��.c��	��ģ��ɽ�һ����װ����Ϊ����ֲ�Ĵ���
//SCL-->PB6
//SDA-->PB7
#define PB6 GPIO_Pin_6 //gpio.h: (uint16_t)0x0040 =64
#define PB7 GPIO_Pin_7 //gpio.h: (uint16_t)0x0080 =128

//��ƫ����
#define Offset_PB6 (u8)(log(PB6)/log(2)) //ln(64)/ln(2)=log_2(64)=6  ƫ��6λ
#define Offset_PB7 (u8)(log(PB7)/log(2)) //ln(128)/ln(2)=log_2(128)=7 ƫ��7λ

//IO�������� ֱ�ӿ���GPIO�˿�ģʽ�Ĵ���GPIOx_MODER�ı��������ģʽ���ο�F4���Ĳο��ֲ�P187
#define SDA_IN()	{GPIOB->MODER&=~(3<<(Offset_PB7*2)); GPIOB->MODER|=0<<Offset_PB7*2;}
#define SDA_OUT()	{GPIOB->MODER&=~(3<<(Offset_PB7*2)); GPIOB->MODER|=1<<Offset_PB7*2;}
//����SDA_IN()�����仰(OUT)
//3=(Binary)11, ����14λ�ı�MODER6[1:0]Ϊ11��~ȡ��Ϊ00�����뼴��Ϊ00������ԭλ
//00(B01)����14λ�����Ϊ00(B01)���������(���)ģʽ���趨����λ״̬������&ͨ�����ģʽ

// IO��������
#define IIC_SCL		PBout(6) //SCL
#define IIC_SDA		PBout(7) //SDA
#define READ_SDA	PBin(7) //����SDA

//****************************************B
// 1 ��ʼ��IIC
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; //���������ó�����ʱ��PB7��Ч
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD; //��©���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP; //���������ó�����ʱ��PB7��Ч
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_6|GPIO_Pin_7); //��1=���� ��PBout(6)=0; PBout(7)=0;
}

//****************************************C IICͨ�ŵ�ģ��
// 2 ����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT(); //SDA��Ϊ���ģʽ
	IIC_SCL=0;
	delay_us(1);
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1; //������ ����״̬
	delay_us(4);
	IIC_SDA=0; //SCL=1, SDA:1->0  START: when CLK is high, DATA change from high to low
	delay_us(4);
	IIC_SCL=0; //ǯסIIC���� SDA�ı仯��Ч
}

// 3 ����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL=0;
	delay_us(1);
	IIC_SDA=0; //SCL=1, SDA:0->1  STOP: when CLK is high DATA change from low to high
	delay_us(4);
	IIC_SCL=1;
	delay_us(4);
	IIC_SDA=1; //����IIC���߽����ź�
	delay_us(4);
}

// 4 �ȴ�Ӧ���źŵĵ���
//return 1: ����Ӧ��ʧ�ܣ�return 0: ����Ӧ��ɹ�
u8 IIC_WaitAck(void)
{
	u8 ucErrTime=0;
	SDA_IN(); //SDA��Ϊ����״̬(����)
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	while(READ_SDA) //PBin(7)
	{
		ucErrTime++;
		if(ucErrTime>250) //�ο�С��������ԭ�ӵĴ��붼����� ����debugģʽ�¼�ʱ
		{				  //�յ�ACK��SDA=0����ѭ�����յ�NACK��SDA=1����ʱ����
			IIC_Stop();
			return 1; //����Ӧ��ʧ��
		}
	}
	IIC_SCL=0; //ǯ������
	return 0; //����Ӧ��ɹ�
}

// 5 ����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}


// 6 ����NACKӦ��
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}

// 7 IIC����1���ֽ�    �������� �ӻ�����
void IIC_SendByte(u8 data)
{
	u8 i;
	SDA_OUT();
	IIC_SCL=0;
	for(i=0; i<8; i++)
	{
		//IIC_SDA=(txd&0x80)>>7
		if((data&0x80)>>7) //data��10000000�����X0000000�������λ����7λ�����λ ��X=0����1����if
			IIC_SDA=1;
		else
			IIC_SDA=0;
		data<<=1; //data=data<<1 ����1λ ��7λ(���λ)ѭ��8�ε���0λ(���λ)
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;
		delay_us(2); //delay_us()��ʱ���ο�����ԭ�ӵ�iic����
	}
}

// 8 IIC��ȡ1���ֽ�    �ӻ����� ������ȡ
u8 IIC_ReadByte(u8 ack)
{
	u8 i, data=0; //data: received data
	SDA_IN(); //��������ģʽ���������մӻ���������
	for(i=0; i<8; i++)
	{
		IIC_SCL=0;
		delay_us(2);
		IIC_SCL=1;
		data<<=1; //data=data<<1
		if(READ_SDA) //if PBin(7)==1
			data++; //eg 01X x=1 010+1=011 <<1 011X x=0 0110+0=0110...
		delay_us(1);
	}
	if(ack)
		IIC_Ack(); //ϣ���Է������������ݣ����ͷ������������һ������
	else
		IIC_NAck(); //ϣ���������ݴ��䣺���յ�NACK������ֹͣ�źţ���������
	return data;
}

//****************************************D ��װ�õ�IICͨ��
// 9 ��ȡָ���豸��ָ���Ĵ�����һ��ֵ
//IIC_Addr: Ŀ���豸��ַ��reg�Ĵ�����ַ��*buf����ȡ����Ҫ�洢�ĵ�ַ ָ��
u8 IIC_ReadOneByte(u8 IIC_Addr, u8 reg, u8 *data)
{
	//IIC����ͨѶ����д���/д
	//1 Dummy Write �ȸ���slave��д��ַ
	IIC_Start();
	IIC_SendByte(IIC_Addr<<1|0); //���ʹӻ���ַ&W ��<<��| ��ַԭ����7λ�����ƺ����λ��0����8λ
	if(IIC_WaitAck()) //�ȴ�slave��ACK�ظ�
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
	IIC_WaitAck();
	
	//2 Read(/Write) �ٸ���slave��(/д)��ʵ������
	IIC_Start();
	IIC_SendByte(IIC_Addr<<1|1); //���ʹӻ���ַ&R
	IIC_WaitAck();
	*data=IIC_ReadByte(0); //buf���ָ���������ֵΪrecieved data
	IIC_Stop();
	return 0; //return 1ʧ�� 0�ɹ�
}

// 10 д��ָ���豸ָ���Ĵ�����һ��ֵ
u8 IIC_WriteOneByte(u8 IIC_Addr, u8 reg, u8 data)
{
	//��ͨ��IICдͨ��
	IIC_Start();
	IIC_SendByte(IIC_Addr<<1|0);
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg);
	IIC_WaitAck();
	IIC_SendByte(data);
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}

// 11 ��ȡָ���豸ָ���Ĵ�����length��ֵ
u8 IIC_ReadLengthBytes(u8 IIC_Addr, u8 reg, u8 length, u8 *data)
{
	u8 cnt=0, temp;
	IIC_Start();
	IIC_SendByte(IIC_Addr<<1|0);
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg);
	IIC_WaitAck();
	IIC_Start();
	IIC_SendByte(IIC_Addr<<1|1);
	IIC_WaitAck();
	for(cnt=0; cnt<length; cnt++) //��IIC_ReadOneByte()�������ڴ�
	{
		if(cnt!=(length-1))
			temp=IIC_ReadByte(1);
		else
			temp=IIC_ReadByte(0); //���һ���ֽ���NACK
		data[cnt]=temp;
	}
	IIC_Stop();
	return 0; //return 1ʧ�� 0�ɹ�
}

// 12 д��ָ���豸ָ���Ĵ�����length��ֵ
u8 IIC_WriteLengthBytes(u8 IIC_Addr, u8 reg, u8 length, u8 *data)
{
	u8 cnt=0;
	IIC_Start();
	IIC_SendByte(IIC_Addr<<1|0);
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg);
	IIC_WaitAck();
	for(cnt=0; cnt<length; cnt++)
	{
		IIC_SendByte(data[cnt]);
		if(IIC_WaitAck())
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
}
