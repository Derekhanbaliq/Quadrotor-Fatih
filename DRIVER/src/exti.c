/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/22
程序功能：配置NRF的IRQ的EXTI(外部中断)
备    注：void EXTI2_IRQHandler(void)在nrf.c里
*******************************************************************************/
#include "stm32f4xx.h"
#include "exti.h"
#include "led.h"
#include "delay.h"
#include "stdio.h"

// 1 EXTI的GPIO配置，NRF的IRQ的IO PB2
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

// 2 初始化EXTI
void EXTIT_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_EXTIT, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //使能系统配置时钟
	//配置EXTI一定要初始化系统配置控制器时钟SYSCFG，它通过寄存器控制着EXTI与引脚的连接
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2); //将中断线EXTI_Line2与PB2连接
	
	EXTI_GPIO_Config();
	EXTI_DeInit(); //复位外部中断
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line2; //中断线2
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt; //中断模式
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling; //下降沿触发
	EXTI_Init(&EXTI_InitStructure);
}

//void EXTI2_IRQHandler(void)在nrf24l01.c里！！！！！
