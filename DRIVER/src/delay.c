/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/20
�����ܣ�SysTickʵ����ʱ����
��    ע����������װ����ʱ��ʵ��(�����жϼ�ʱ)�������ǿ��ж���ʱ����
��    ����С���DragonFly������ԭ��Minifly��delay�������ֵĴ���
*******************************************************************************/
#include "stm32f4xx.h"
#include "delay.h"

static u32 fac_ms; //����ϵ��
static u32 fac_us; //΢��ϵ��

//****************************************A
// 1 ��ʱ������ʼ��
void delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //ѡ��ϵͳʱ��(HCLK/8)��ΪSysTick��ʱ��ʱ�� ((uint32_t)0xFFFFFFFB)
	SysTick->CTRL = 0x00;
	fac_ms = (SystemCoreClock/8)/1000; //��ȡ����ϵ�� 		(100MHz/8)/1000=12500Hz
	fac_us = (SystemCoreClock/8)/1000000; //��ȡ΢��ϵ��
}

// 2 ���뼶��ʱ
void delay_ms(u32 nTime)
{
	u32 temp;
	
	//SysTickֻ��CTRL, LOAD, VAL, CALIB�ĸ��Ĵ������ƣ�ֱ�Ӳ��ݸ���
	SysTick->CTRL = 0x00; 			//ʹ��SysTick��ʱ��
	SysTick->LOAD = nTime*fac_ms;	//������װ��ֵ(��λ����)���Ӹ�ֵ�����ƴ���
	SysTick->VAL  = 0x00; 			//��ռ�����
	SysTick->CTRL|= 0x01; 			//����Systick��ʱ��
	
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16))); //ѭ���ȴ���ʱ����
	//����temp��0x01��λ�룬Ϊ0
	//֮��1<<16����16λ����temp��λ�룬Ϊ1
	//��������������ʱ����ѭ��
	//����SysTick->CTRL��bit0=1��bit16=1����������
	
	SysTick->CTRL = 0x00; //ʹ��SysTick��ʱ��
	SysTick->VAL  = 0x00; //��ռ�����
}

// 3 ΢�뼶��ʱ
void delay_us(u32 nTime)
{
	u32 temp;
	
	SysTick->CTRL = 0x00; 			//ʹ��SysTick��ʱ��
	SysTick->LOAD = nTime*fac_us;	//������װ��ֵ
	SysTick->VAL  = 0x00; 			//��ռ�����
	SysTick->CTRL|= 0x01; 			//����Systick��ʱ��
	
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16))); //ѭ���ȴ���ʱ����
	
	SysTick->CTRL = 0x00; //ʹ��SysTick��ʱ��
	SysTick->VAL  = 0x00; //��ռ�����
}

//****************************************B
// 4 ���Լ�ʱ
void delay(u32 timers)
{
	u16 i,j;
	for(i=0;i<timers;i++)
	{
		for(j=0;j<0xffff;j++)
		{}
	}
}
