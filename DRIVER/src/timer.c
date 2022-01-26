/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/23
�����ܣ�TIM4��ʱ���ĳ�ʼ��
��    ע��void TIM4_IRQHandler(void) ��nvic.c��
*******************************************************************************/
#include "stm32f4xx.h"
#include "timer.h"

// 1 TIM4��ʱ���ĳ�ʼ��
void TIM4_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimBaseInitStructure;
	
	//1 ʹ�ܶ�ʱ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	//2 ��ʼ����ʱ�� ����arr&psc
	TIM_TimBaseInitStructure.TIM_Period=1000-1; //arr
	TIM_TimBaseInitStructure.TIM_Prescaler=100-1; //psc
	TIM_TimBaseInitStructure.TIM_ClockDivision=0;
	TIM_TimBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimBaseInitStructure);
	//Tout=((arr+1)*(psc+1))/Tclk
	//Tclk: TIM4������ʱ��Ƶ�� 100MHz->��ʱ����ͼ��SYSCLKmax=100MHz������Ƶ���APB1=100MHz
	//arr: �Զ���װ��ֵ
	//psc: ʱ��Ԥ��Ƶϵ��
	//(psc+1))/Tclk: һ�����ڵ�ʱ�䳤�� 1us
	//Tout: TIM4���ʱ�� 1ms����һ��
	
	//3 ������ʱ���ж� & ����NVIC(��nvic.c��������)
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	//4 ʹ�ܶ�ʱ��
	TIM_Cmd(TIM4, ENABLE);
}

//5 ��д�жϷ�����void TIM4_IRQHandler(void) (��nvic.c��)
