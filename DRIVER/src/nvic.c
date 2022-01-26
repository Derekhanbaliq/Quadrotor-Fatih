/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/21
程序功能：配置NVIC/内嵌向量中断控制器
备    注：h文件有位带操作的定义，该部分参考小马哥DragonFly的nvicconfig.h
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

// 1 配置项目中的所有中断的优先级
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断分组2
	
	//TIM4定时器中断优先级
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//USART2中断
	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//USART1中断
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//NRF24L01的IRQ中断
	NVIC_InitStructure.NVIC_IRQChannel=EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// 2 USART1中断服务函数
void USART1_IRQHandler(void) //插线调参用usart1
{							 //对于连续的数据帧的接收 接收中断与空闲中断配合能解决对报问题
	u8 clr=clr, res; //u8 clear=clear; //防止编译时报错？？
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET) //RXNE: 接收中断 如果检测到USART1的接收中断标志位=1(SET了)
	{
		res = USART1->DR; //写DR清楚中断标志
		ANO_DT_Data_Receive_Prepare(res); //上位机数据接收与解析
	}
	else if(USART_GetITStatus(USART1, USART_IT_IDLE)!=RESET) //IDLE: 空闲中断
	{
		clr = USART1->SR; //读SR寄存器
		clr = USART1->DR; //读DR寄存器(先读SR再读DR，就是为了清除IDLE中断)
	}
	//清除USART1上的中断标志位
	USART_ClearITPendingBit(USART1, USART_IT_RXNE); //RXNE=0, IDLE=1
}

// 3 USART2中断服务函数 上位机与Wifi遥控数据共用
void USART2_IRQHandler(void) //上位机与WiFi遥控最好不要同时用，当遥控数据一帧数据接收完成才触发空闲中断;
{							 //当用WiFi无线调参的时候用USART2接收上位机数据;
	u8 clr=clr, res;         //对于连续的数据帧的接收 接收中断与空闲中断配合能解决对报问题;
	if(USART_GetITStatus(USART2, USART_IT_RXNE)!=RESET) //接收中断
	{
		res = USART2->DR; //写DR清除中断标志
		ANO_DT_Data_Receive_Prepare(res); //上位机数据接收与解析
		RxBuffer[RxCounter++]=res; 
	}
	else if(USART_GetITStatus(USART2, USART_IT_IDLE)!=RESET) //空闲中断
	{
		clr = USART2->SR; //读SR寄存器
		clr = USART2->DR; //读DR寄存器(先读SR，再读DR，就是为了清除IDLE中断)
		WiFi_Data_ReceiveAnalysis(RxBuffer,RxCounter-1); //WiFi遥控数据的解析
		RxCounter = 0;
	}
	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}

// 4 TIM4定时器中断服务函数
void TIM4_IRQHandler(void) //此函数是整个程序的运行时间基准，不同的中断时间对应不同的频率，一些计算对调用时间要求较为严格的可用此法
{						   //扫描频率=1000Hz/分频系数
	static u16 ms2=0, ms5=0, ms10=0, ms100=0, ms200=0, ms400=0; //分频系数
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
