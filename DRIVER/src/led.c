/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/20
�����ܣ�LED����
*******************************************************************************/
#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"

// 1 LED��GPIO��ʼ��
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//����PB8Ϊ�������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; //������Ҳ������
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_8); //USER_LED��
}

// 2 LED��ת������ָʾMCU�Ƿ���
void LED_Run(void)
{
	static u8 flag=1;
	if(flag)
	{
		flag=0;
		GPIO_ResetBits(GPIOB,GPIO_Pin_8); //USER_LED�� �ɲο�����ԭ��ͨ���궨���
	}
	else
	{
		flag=1;
		GPIO_SetBits(GPIOB,GPIO_Pin_8); //USER_LED��
	}
}
