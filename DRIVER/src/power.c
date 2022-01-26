/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/09/06
�����ܣ���Դ����
*******************************************************************************/
#include "power.h"
#include "structconfig.h"
#include "filter.h"
#include "stdio.h"
#include "nrf24l01.h"
#include "rgb.h"

BAT_TYPE BAT=
{
	.BattAdc=0,			//��Դ��ѹ�ɼ�ADCֵ--12λ2������
	.BattRealV=3.31f,	//ʵ�ʲ����ķɻ������ѹ(ע��˵�ѹ�����ײ⣬��������ĵ�ѹ��׼)
	.BattMeasureV=0,	//���������ʵ�ʵ�ص�ѹ
	.BattAlarmV=3.2f,	//��ص͵�ѹ����˲ʱֵ(��ֵ��Ҫ���ݻ���ͬ����ʵ�⣬ʵ��380mhΪ2.8V)
	.BattFullV=4.2f,	//��س�����ĵ�ѹֵ4.2V
};

u8 RGB_BATflag = 0;

// 1 ��Դ�����ʼ��
void BAT_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN; //ģ������ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;        
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //PA0 VBAT_TEST
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4); //PA4 ESP_EN
	GPIO_ResetBits(GPIOA,GPIO_Pin_5); //PA5 ESP_BOOT
	
	//ADCͨ������(ADCʱ��Ƶ����ò�����36MHz) //����ԭ�ӵ�F1��15MHz
	ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent; //����ģʽ
	ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div4; //4��Ƶ fplck2/4=25MHz 
	ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled; //DMAʧ��
	ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles; //��������֮����5��ʱ��
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//ADC1������ʼ��
	ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b; //12λ��������
	ADC_InitStructure.ADC_ScanConvMode=DISABLE; //ʧ��ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE; //ʧ������ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None; //������������ʹ���������
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right; //�����Ҷ���
	ADC_InitStructure.ADC_NbrOfConversion=1; //һ��ת���ڹ���������
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles); //ADC1��ADC_Channel_0���й���ת������

	ADC_Cmd(ADC1, ENABLE); //ʹ��ADC1
}

// 2 ��ȡ��ز������ѹ��ADCֵ
u16 BAT_GetAdc(u8 ch) //ch: ADC����ͨ��
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_144Cycles); //ADC1ͨ�� ��������144������
	ADC_SoftwareStartConv(ADC1); //ʹ��ָ����ADC1���ת����������
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); //�ȴ�ת������
	return ADC_GetConversionValue(ADC1); //�������1��ADC1�������ת�����
}

// 3 ��ȡ��ص�ѹ
void BAT_GetVoltage(void)
{
	float V;
	Aver_Filter((float)BAT_GetAdc(ADC_Channel_0), &BAT.BattAdc, 6); //�����˲�һ�µ�ѹֵ����߾���
	if(BAT.BattAdc)
	{
		V=BAT.BattAdc*BAT.BattRealV/4095.0f; //BattAdc*3.31/(12λ����)
	}
	BAT.BattMeasureV=2*V; //����ԭ��ͼ����Դʵ�ʵ�ѹ=ADC������ѹ*2
	printf("Test Voltage :%0.2f   temp:%0.0f \r\n ", BAT.BattMeasureV, BAT.BattAdc);
}

// 4 �͵�������
void BAT_LowVoltageAlarm(void)
{
	static u8 cnt=0, cnt1=0;
	BAT_GetVoltage();
	if(Airplane_Enable) //������control.c��
	{
		if(BAT.BattMeasureV<BAT.BattAlarmV) //����ʱ����
		{
			if(cnt1++>10)
			{
				cnt1=0;
				RGB_BATflag=1;
			}
		}
		else
		{
			cnt1=0;
			RGB_BATflag=0;
		}
	}
	else
	{
		if(BAT.BattMeasureV<3.7f) //���ʱ����(380mhʱ��3.5V)
		{
			if(cnt++>10)
			{
				Run_flag=0;
				cnt=0;
				RGB_BATflag=1;
			}
		}
		else
		{
			Run_flag=1;
			cnt=0;
			RGB_BATflag=0;
		}
	}
}

// 5 ����WiFiģ��
void WiFi_Switch(u8 flag)
{
	if(flag) //����WiFi
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4); //ESP_EN
		GPIO_SetBits(GPIOA, GPIO_Pin_5); //ESP_BOOT
	}
	else     //�ر�WiFi
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_4); //ESP_EN
		GPIO_ResetBits(GPIOA, GPIO_Pin_5); //ESP_BOOT
	}
}

// 6 ����NRF24L01ģ��
void NRF24L01_Switch(u8 flag)
{
	if(flag) //����NRF2401
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_8);   //NRF_CE
		NRF24L01_WriteReg(W_REGISTER+CONFIG, 0x0A);	 
	}
	else     //�ر�NRF2401
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_8); //NRF_CE
		NRF24L01_WriteReg(W_REGISTER+CONFIG, 0x0F);	 
	}
}
