/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/22
�����ܣ�����NRF��IRQ��EXTI(�ⲿ�ж�)
��    ע��void EXTI2_IRQHandler(void)��nrf.c��
*******************************************************************************/
#include "stm32f4xx.h"
#include "exti.h"
#include "led.h"
#include "delay.h"
#include "stdio.h"

// 1 EXTI��GPIO���ã�NRF��IRQ��IO PB2
void EXTI_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 2 ��ʼ��EXTI
void EXTIT_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_EXTIT, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //ʹ��ϵͳ����ʱ��
	//����EXTIһ��Ҫ��ʼ��ϵͳ���ÿ�����ʱ��SYSCFG����ͨ���Ĵ���������EXTI�����ŵ�����
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2); //���ж���EXTI_Line2��PB2����
	
	EXTI_GPIO_Config();
	EXTI_DeInit(); //��λ�ⲿ�ж�
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line2; //�ж���2
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt; //�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling; //�½��ش���
	EXTI_Init(&EXTI_InitStructure);
}

//void EXTI2_IRQHandler(void)��nrf24l01.c���������
