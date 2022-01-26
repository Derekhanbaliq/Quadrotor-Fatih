#ifndef IIC_H
#define IIC_H

#include "stm32f4xx.h"
#include "nvic.h" //λ����

//С�������ı�ע����ֲ������������DMP������Ԫ��ʱ��ע��ӻ���ַҪ���ó�0xD0��

void IIC_Init(void);

void IIC_Start(void);
void IIC_Stop(void);
u8 IIC_WaitAck(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_SendByte(u8 data);
u8 IIC_ReadByte(u8 data);

u8 IIC_ReadOneByte(u8 IIC_Addr, u8 reg, u8 *data);
u8 IIC_WriteOneByte(u8 IIC_Addr, u8 reg, u8 data);
u8 IIC_ReadLengthBytes(u8 IIC_Addr, u8 reg, u8 length, u8 *data);
u8 IIC_WriteLengthBytes(u8 IIC_Addr, u8 reg, u8 length, u8 *data);

#endif
