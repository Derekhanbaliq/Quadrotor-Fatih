/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/24
程序功能：IIC软件驱动
备    注：主要参考自己学F1时写的myiic.c&h，部分借鉴小马哥四轴的iic.c&h
*******************************************************************************/
#include "iic.h"
#include "math.h"
#include "nvic.h"
#include "delay.h"

//****************************************A
//为便于观察，宏定义部分写在.c中	该模块可进一步封装，成为可移植的代码
//SCL-->PB6
//SDA-->PB7
#define PB6 GPIO_Pin_6 //gpio.h: (uint16_t)0x0040 =64
#define PB7 GPIO_Pin_7 //gpio.h: (uint16_t)0x0080 =128

//求偏移量
#define Offset_PB6 (u8)(log(PB6)/log(2)) //ln(64)/ln(2)=log_2(64)=6  偏移6位
#define Offset_PB7 (u8)(log(PB7)/log(2)) //ln(128)/ln(2)=log_2(128)=7 偏移7位

//IO方向设置 直接控制GPIO端口模式寄存器GPIOx_MODER改变输入输出模式，参考F4中文参考手册P187
#define SDA_IN()	{GPIOB->MODER&=~(3<<(Offset_PB7*2)); GPIOB->MODER|=0<<Offset_PB7*2;}
#define SDA_OUT()	{GPIOB->MODER&=~(3<<(Offset_PB7*2)); GPIOB->MODER|=1<<Offset_PB7*2;}
//解释SDA_IN()的两句话(OUT)
//3=(Binary)11, 左移14位改变MODER6[1:0]为11，~取反为00，求与即改为00即擦除原位
//00(B01)左移14位，求或为00(B01)，完成输入(输出)模式的设定：复位状态的输入&通用输出模式

// IO操作函数
#define IIC_SCL		PBout(6) //SCL
#define IIC_SDA		PBout(7) //SDA
#define READ_SDA	PBin(7) //输入SDA

//****************************************B
// 1 初始化IIC
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; //仅仅对配置成输入时的PB7有效
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD; //开漏输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP; //仅仅对配置成输入时的PB7有效
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_6|GPIO_Pin_7); //置1=拉高 即PBout(6)=0; PBout(7)=0;
}

//****************************************C IIC通信的模块
// 2 生成IIC起始信号
void IIC_Start(void)
{
	SDA_OUT(); //SDA变为输出模式
	IIC_SCL=0;
	delay_us(1);
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1; //都拉高 空闲状态
	delay_us(4);
	IIC_SDA=0; //SCL=1, SDA:1->0  START: when CLK is high, DATA change from high to low
	delay_us(4);
	IIC_SCL=0; //钳住IIC总线 SDA的变化无效
}

// 3 生成IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL=0;
	delay_us(1);
	IIC_SDA=0; //SCL=1, SDA:0->1  STOP: when CLK is high DATA change from low to high
	delay_us(4);
	IIC_SCL=1;
	delay_us(4);
	IIC_SDA=1; //发送IIC总线结束信号
	delay_us(4);
}

// 4 等待应答信号的到来
//return 1: 接收应答失败；return 0: 接收应答成功
u8 IIC_WaitAck(void)
{
	u8 ucErrTime=0;
	SDA_IN(); //SDA变为输入状态(主机)
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	while(READ_SDA) //PBin(7)
	{
		ucErrTime++;
		if(ucErrTime>250) //参考小马哥和正点原子的代码都是如此 可在debug模式下计时
		{				  //收到ACK，SDA=0跳出循环；收到NACK，SDA=1，超时跳出
			IIC_Stop();
			return 1; //接收应答失败
		}
	}
	IIC_SCL=0; //钳制总线
	return 0; //接收应答成功
}

// 5 产生ACK应答
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


// 6 产生NACK应答
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

// 7 IIC发送1个字节    主机发送 从机接收
void IIC_SendByte(u8 data)
{
	u8 i;
	SDA_OUT();
	IIC_SCL=0;
	for(i=0; i<8; i++)
	{
		//IIC_SDA=(txd&0x80)>>7
		if((data&0x80)>>7) //data和10000000求与得X0000000并将最高位右移7位至最低位 看X=0还是1决定if
			IIC_SDA=1;
		else
			IIC_SDA=0;
		data<<=1; //data=data<<1 左移1位 第7位(最高位)循环8次到第0位(最低位)
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;
		delay_us(2); //delay_us()的时长参考正点原子的iic代码
	}
}

// 8 IIC读取1个字节    从机发送 主机读取
u8 IIC_ReadByte(u8 ack)
{
	u8 i, data=0; //data: received data
	SDA_IN(); //开启输入模式，主机接收从机返回数据
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
		IIC_Ack(); //希望对方继续发送数据，发送方会继续发送下一个数据
	else
		IIC_NAck(); //希望结束数据传输：接收到NACK后会产生停止信号，结束传输
	return data;
}

//****************************************D 封装好的IIC通信
// 9 读取指定设备的指定寄存器的一个值
//IIC_Addr: 目标设备地址，reg寄存器地址，*buf：读取数据要存储的地址 指针
u8 IIC_ReadOneByte(u8 IIC_Addr, u8 reg, u8 *data)
{
	//IIC复合通讯，先写后读/写
	//1 Dummy Write 先告诉slave读写地址
	IIC_Start();
	IIC_SendByte(IIC_Addr<<1|0); //发送从机地址&W 先<<再| 地址原本是7位，左移后最低位＋0，变8位
	if(IIC_WaitAck()) //等待slave的ACK回复
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();
	
	//2 Read(/Write) 再告诉slave读(/写)的实际内容
	IIC_Start();
	IIC_SendByte(IIC_Addr<<1|1); //发送从机地址&R
	IIC_WaitAck();
	*data=IIC_ReadByte(0); //buf这个指针变量被赋值为recieved data
	IIC_Stop();
	return 0; //return 1失败 0成功
}

// 10 写入指定设备指定寄存器的一个值
u8 IIC_WriteOneByte(u8 IIC_Addr, u8 reg, u8 data)
{
	//普通的IIC写通信
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

// 11 读取指定设备指定寄存器的length个值
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
	for(cnt=0; cnt<length; cnt++) //和IIC_ReadOneByte()的区别在此
	{
		if(cnt!=(length-1))
			temp=IIC_ReadByte(1);
		else
			temp=IIC_ReadByte(0); //最后一个字节是NACK
		data[cnt]=temp;
	}
	IIC_Stop();
	return 0; //return 1失败 0成功
}

// 12 写入指定设备指定寄存器的length个值
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
