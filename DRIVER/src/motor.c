/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/09/03
程序功能：电机驱动
*******************************************************************************/
#include "stm32f4xx.h"
#include "motor.h"

#define Motor_PwmMax 1000
s16 MOTOR1_PWM=0;
s16 MOTOR2_PWM=0;
s16 MOTOR3_PWM=0;
s16 MOTOR4_PWM=0;

// 1 TIM3 PWM输出通道GPIO设置
/*	TIM3 CH1(PWM1) -> PA6
	TIM3 CH2(PWM2) -> PA7
	TIM3 CH3(PWM3) -> PB0
	TIM3 CH4(PWM4) -> PB1  */
void TIM3_GPIOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM3时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 		 //复用模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;	 //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP; 		 //上拉输入
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	//复用TIM3的通道到AF2 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); //重映射remap是F1的方式，F4直接修改复用寄存器AFIO就行
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); //F1里的AFIO时钟变成了F4中的SYSCFG时钟了
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
}

// 2 输出PWM的初始化
void MOTOR_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//TIM3的GPIO配置
	TIM3_GPIOConfig();
	
	//初始化定时器 配置arr&psc
	TIM_TimeBaseInitStructure.TIM_Period=1000-1; //arr
	TIM_TimeBaseInitStructure.TIM_Prescaler=100-1; //psc
	TIM_TimeBaseInitStructure.TIM_ClockDivision=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	//Tout=((arr+1)*(psc+1))/Tclk=1ms 详细分析见timer.c文件！
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; //选择PWM模式1：向上计数时，一旦TIMx_CNT<TIMx_CCR1的通道1为有效电平，否则为无效电平 PWM2模式与之相反！
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable; //使能比较输出
	TIM_OCInitStructure.TIM_Pulse=0; //比较值CCRx的值 后边还会重新设置！
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; //输出极性高
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure); //初始化输出比较参数
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); //使能预装载寄存器
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //寄存器预装载
	
	TIM_Cmd(TIM3, ENABLE); //使能TIM3
}

// 3 电机输出数值转换成PWM输出 该函数用在control.c的动力分配中
void Motor_Pwm(s16 MOTOR1_PWM, s16 MOTOR2_PWM, s16 MOTOR3_PWM, s16 MOTOR4_PWM) //debug: MOTO1R_PWM
{
	if(MOTOR1_PWM>Motor_PwmMax)	MOTOR1_PWM=Motor_PwmMax;
	if(MOTOR2_PWM>Motor_PwmMax)	MOTOR2_PWM=Motor_PwmMax;
	if(MOTOR3_PWM>Motor_PwmMax)	MOTOR3_PWM=Motor_PwmMax;
	if(MOTOR4_PWM>Motor_PwmMax)	MOTOR4_PWM=Motor_PwmMax;
	
	if(MOTOR1_PWM<0) MOTOR1_PWM=0;
	if(MOTOR2_PWM<0) MOTOR2_PWM=0;
	if(MOTOR3_PWM<0) MOTOR3_PWM=0;
	if(MOTOR4_PWM<0) MOTOR4_PWM=0;
	
	TIM3->CCR1=MOTOR1_PWM; //CCRx的值与ARR作比较实现PWM
	TIM3->CCR2=MOTOR2_PWM;
	TIM3->CCR3=MOTOR3_PWM;
	TIM3->CCR4=MOTOR4_PWM;
}
