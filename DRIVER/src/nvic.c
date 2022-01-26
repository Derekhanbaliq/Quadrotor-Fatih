/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/21
�����ܣ�����NVIC/��Ƕ�����жϿ�����
��    ע��h�ļ���λ�������Ķ��壬�ò��ֲο�С���DragonFly��nvicconfig.h
*******************************************************************************/
#include "stm32f4xx.h"
#include "nvic.h"
#include "delay.h"
#include "led.h"
#include "ANO_DT.h"
#include "remotedata.h"
#include "structconfig.h"

u8 LED_Scan=0;
u8 IMU_Scan=0;
u8 MPU_Scan=0;
u8 IRQ_Scan=0;
u8 BAT_Scan=0;
u8 ANO_Scan=0;

static u8 RxBuffer[30];
static u8 RxCounter=0;

// 1 ������Ŀ�е������жϵ����ȼ�
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�����жϷ���2
	
	//TIM4��ʱ���ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//USART2�ж�
	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//USART1�ж�
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//NRF24L01��IRQ�ж�
	NVIC_InitStructure.NVIC_IRQChannel=EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// 2 USART1�жϷ�����
void USART1_IRQHandler(void) //���ߵ�����usart1
{							 //��������������֡�Ľ��� �����ж�������ж�����ܽ���Ա�����
	u8 clr=clr, res; //u8 clear=clear; //��ֹ����ʱ������
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET) //RXNE: �����ж� �����⵽USART1�Ľ����жϱ�־λ=1(SET��)
	{
		res = USART1->DR; //дDR����жϱ�־
		ANO_DT_Data_Receive_Prepare(res); //��λ�����ݽ��������
	}
	else if(USART_GetITStatus(USART1, USART_IT_IDLE)!=RESET) //IDLE: �����ж�
	{
		clr = USART1->SR; //��SR�Ĵ���
		clr = USART1->DR; //��DR�Ĵ���(�ȶ�SR�ٶ�DR������Ϊ�����IDLE�ж�)
	}
	//���USART1�ϵ��жϱ�־λ
	USART_ClearITPendingBit(USART1, USART_IT_RXNE); //RXNE=0, IDLE=1
}

// 3 USART2�жϷ����� ��λ����Wifiң�����ݹ���
void USART2_IRQHandler(void) //��λ����WiFiң����ò�Ҫͬʱ�ã���ң������һ֡���ݽ�����ɲŴ��������ж�;
{							 //����WiFi���ߵ��ε�ʱ����USART2������λ������;
	u8 clr=clr, res;         //��������������֡�Ľ��� �����ж�������ж�����ܽ���Ա�����;
	if(USART_GetITStatus(USART2, USART_IT_RXNE)!=RESET) //�����ж�
	{
		res = USART2->DR; //дDR����жϱ�־
		ANO_DT_Data_Receive_Prepare(res); //��λ�����ݽ��������
		RxBuffer[RxCounter++]=res; 
	}
	else if(USART_GetITStatus(USART2, USART_IT_IDLE)!=RESET) //�����ж�
	{
		clr = USART2->SR; //��SR�Ĵ���
		clr = USART2->DR; //��DR�Ĵ���(�ȶ�SR���ٶ�DR������Ϊ�����IDLE�ж�)
		WiFi_Data_ReceiveAnalysis(RxBuffer,RxCounter-1); //WiFiң�����ݵĽ���
		RxCounter = 0;
	}
	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}

// 4 TIM4��ʱ���жϷ�����
void TIM4_IRQHandler(void) //�˺������������������ʱ���׼����ͬ���ж�ʱ���Ӧ��ͬ��Ƶ�ʣ�һЩ����Ե���ʱ��Ҫ���Ϊ�ϸ�Ŀ��ô˷�
{						   //ɨ��Ƶ��=1000Hz/��Ƶϵ��
	static u16 ms2=0, ms5=0, ms10=0, ms100=0, ms200=0, ms400=0; //��Ƶϵ��
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)!=RESET)
	{
		ms2++;
		ms5++;
		ms10++;
		ms100++;
		ms200++;
		ms400++;
		if(ms2 >= 2) //500Hz
		{
			ms2 = 0;
			ANO_Scan = 1;
		}
		if(ms5 >= 5) //200Hz
		{
			ms5 = 0;
			MPU_Scan = 1;
		}
		if(ms10 >= 10) //100Hz
		{
			ms10 = 0;
			IMU_Scan = 1;
		}
		if(ms100 >= 100) //10Hz
		{
			ms100 = 0;
			LED_Scan = 1;
		}
		if(ms200 >= 200) //5Hz
		{
			ms200 = 0;
			IRQ_Scan = 1;
		}
		if(ms400 >= 400) //2.5Hz
		{
			ms400 = 0;
			BAT_Scan = 1;
		}
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
}
