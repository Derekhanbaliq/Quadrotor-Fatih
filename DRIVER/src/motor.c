/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/09/03
�����ܣ��������
*******************************************************************************/
#include "stm32f4xx.h"
#include "motor.h"

#define Motor_PwmMax 1000
s16 MOTOR1_PWM=0;
s16 MOTOR2_PWM=0;
s16 MOTOR3_PWM=0;
s16 MOTOR4_PWM=0;

// 1 TIM3 PWM���ͨ��GPIO����
/*	TIM3 CH1(PWM1) -> PA6
	TIM3 CH2(PWM2) -> PA7
	TIM3 CH3(PWM3) -> PB0
	TIM3 CH4(PWM4) -> PB1  */
void TIM3_GPIOConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM3ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 		 //����ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;	 //�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP; 		 //��������
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	//����TIM3��ͨ����AF2 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); //��ӳ��remap��F1�ķ�ʽ��F4ֱ���޸ĸ��üĴ���AFIO����
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); //F1���AFIOʱ�ӱ����F4�е�SYSCFGʱ����
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3); 
}

// 2 ���PWM�ĳ�ʼ��
void MOTOR_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//TIM3��GPIO����
	TIM3_GPIOConfig();
	
	//��ʼ����ʱ�� ����arr&psc
	TIM_TimeBaseInitStructure.TIM_Period=1000-1; //arr
	TIM_TimeBaseInitStructure.TIM_Prescaler=100-1; //psc
	TIM_TimeBaseInitStructure.TIM_ClockDivision=0;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	//Tout=((arr+1)*(psc+1))/Tclk=1ms ��ϸ������timer.c�ļ���
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; //ѡ��PWMģʽ1�����ϼ���ʱ��һ��TIMx_CNT<TIMx_CCR1��ͨ��1Ϊ��Ч��ƽ������Ϊ��Ч��ƽ PWM2ģʽ��֮�෴��
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable; //ʹ�ܱȽ����
	TIM_OCInitStructure.TIM_Pulse=0; //�Ƚ�ֵCCRx��ֵ ��߻����������ã�
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; //������Ը�
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure); //��ʼ������Ƚϲ���
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); //ʹ��Ԥװ�ؼĴ���
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //�Ĵ���Ԥװ��
	
	TIM_Cmd(TIM3, ENABLE); //ʹ��TIM3
}

// 3 ��������ֵת����PWM��� �ú�������control.c�Ķ���������
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
	
	TIM3->CCR1=MOTOR1_PWM; //CCRx��ֵ��ARR���Ƚ�ʵ��PWM
	TIM3->CCR2=MOTOR2_PWM;
	TIM3->CCR3=MOTOR3_PWM;
	TIM3->CCR4=MOTOR4_PWM;
}
