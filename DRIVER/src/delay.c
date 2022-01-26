/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/20
程序功能：SysTick实现延时函数
备    注：仅采用重装载延时法实现(不开中断计时)，不考虑开中断延时法！
参    考：小马哥DragonFly与正点原子Minifly的delay函数部分的代码
*******************************************************************************/
#include "stm32f4xx.h"
#include "delay.h"

static u32 fac_ms; //毫秒系数
static u32 fac_us; //微秒系数

//****************************************A
// 1 延时函数初始化
void delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //选择系统时钟(HCLK/8)作为SysTick定时器时钟 ((uint32_t)0xFFFFFFFB)
	SysTick->CTRL = 0x00;
	fac_ms = (SystemCoreClock/8)/1000; //获取毫秒系数 		(100MHz/8)/1000=12500Hz
	fac_us = (SystemCoreClock/8)/1000000; //获取微秒系数
}

// 2 毫秒级计时
void delay_ms(u32 nTime)
{
	u32 temp;
	
	//SysTick只受CTRL, LOAD, VAL, CALIB四个寄存器控制，直接操纵更简单
	SysTick->CTRL = 0x00; 			//使能SysTick定时器
	SysTick->LOAD = nTime*fac_ms;	//计算重装载值(单位：次)；从该值倒数计次数
	SysTick->VAL  = 0x00; 			//清空计数器
	SysTick->CTRL|= 0x01; 			//启用Systick定时器
	
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16))); //循环等待延时结束
	//变量temp与0x01按位与，为0
	//之后，1<<16左移16位，与temp按位与，为1
	//则条件成立，此时结束循环
	//即，SysTick->CTRL的bit0=1且bit16=1，条件成立
	
	SysTick->CTRL = 0x00; //使能SysTick定时器
	SysTick->VAL  = 0x00; //清空计数器
}

// 3 微秒级计时
void delay_us(u32 nTime)
{
	u32 temp;
	
	SysTick->CTRL = 0x00; 			//使能SysTick定时器
	SysTick->LOAD = nTime*fac_us;	//计算重装载值
	SysTick->VAL  = 0x00; 			//清空计数器
	SysTick->CTRL|= 0x01; 			//启用Systick定时器
	
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16))); //循环等待延时结束
	
	SysTick->CTRL = 0x00; //使能SysTick定时器
	SysTick->VAL  = 0x00; //清空计数器
}

//****************************************B
// 4 粗略计时
void delay(u32 timers)
{
	u16 i,j;
	for(i=0;i<timers;i++)
	{
		for(j=0;j<0xffff;j++)
		{}
	}
}
