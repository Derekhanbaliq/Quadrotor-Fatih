/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/20
程序功能：LED驱动
*******************************************************************************/
#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"

// 1 LED的GPIO初始化
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//配置PB8为推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; //不上拉也不下拉
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_8); //USER_LED灭
}

// 2 LED运转，用来指示MCU是否工作
void LED_Run(void)
{
	static u8 flag=1;
	if(flag)
	{
		flag=0;
		GPIO_ResetBits(GPIOB,GPIO_Pin_8); //USER_LED亮 可参考正点原子通过宏定义简化
	}
	else
	{
		flag=1;
		GPIO_SetBits(GPIOB,GPIO_Pin_8); //USER_LED灭
	}
}
