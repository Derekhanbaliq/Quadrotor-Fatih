/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/23
程序功能：TIM4定时器的初始化
备    注：void TIM4_IRQHandler(void) 在nvic.c中
*******************************************************************************/
#include "stm32f4xx.h"
#include "timer.h"

// 1 TIM4定时器的初始化
void TIM4_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimBaseInitStructure;
	
	//1 使能定时器时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	//2 初始化定时器 配置arr&psc
	TIM_TimBaseInitStructure.TIM_Period=1000-1; //arr
	TIM_TimBaseInitStructure.TIM_Prescaler=100-1; //psc
	TIM_TimBaseInitStructure.TIM_ClockDivision=0;
	TIM_TimBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimBaseInitStructure);
	//Tout=((arr+1)*(psc+1))/Tclk
	//Tclk: TIM4的输入时钟频率 100MHz->查时钟树图，SYSCLKmax=100MHz，不分频后的APB1=100MHz
	//arr: 自动重装载值
	//psc: 时钟预分频系数
	//(psc+1))/Tclk: 一个周期的时间长度 1us
	//Tout: TIM4溢出时间 1ms计数一次
	
	//3 开启定时器中断 & 配置NVIC(在nvic.c中已配置)
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	//4 使能定时器
	TIM_Cmd(TIM4, ENABLE);
}

//5 编写中断服务函数void TIM4_IRQHandler(void) (在nvic.c中)
