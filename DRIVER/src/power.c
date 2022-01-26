/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/09/06
程序功能：电源管理
*******************************************************************************/
#include "power.h"
#include "structconfig.h"
#include "filter.h"
#include "stdio.h"
#include "nrf24l01.h"
#include "rgb.h"

BAT_TYPE BAT=
{
	.BattAdc=0,			//电源电压采集ADC值--12位2进制数
	.BattRealV=3.31f,	//实际测量的飞机供电电压(注意此电压必须亲测，否则测量的电压不准)
	.BattMeasureV=0,	//程序测量的实际电池电压
	.BattAlarmV=3.2f,	//电池低电压报警瞬时值(该值需要根据机身不同重量实测，实测380mh为2.8V)
	.BattFullV=4.2f,	//电池充满电的电压值4.2V
};

u8 RGB_BATflag = 0;

// 1 电源管理初始化
void BAT_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN; //模拟输入模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;        
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //PA0 VBAT_TEST
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4); //PA4 ESP_EN
	GPIO_ResetBits(GPIOA,GPIO_Pin_5); //PA5 ESP_BOOT
	
	//ADC通用设置(ADC时钟频率最好不超过36MHz) //正点原子的F1是15MHz
	ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent; //独立模式
	ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div4; //4分频 fplck2/4=25MHz 
	ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled; //DMA失能
	ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles; //两个采样之间间隔5个时钟
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//ADC1参数初始化
	ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b; //12位采样精度
	ADC_InitStructure.ADC_ScanConvMode=DISABLE; //失能扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE; //失能连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None; //不开启触发，使用软件出发
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right; //数据右对齐
	ADC_InitStructure.ADC_NbrOfConversion=1; //一个转化在规则序列中
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_144Cycles); //ADC1的ADC_Channel_0进行规则转换配置

	ADC_Cmd(ADC1, ENABLE); //使能ADC1
}

// 2 获取电池采样点电压的ADC值
u16 BAT_GetAdc(u8 ch) //ch: ADC采样通道
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_144Cycles); //ADC1通道 采样周期144个周期
	ADC_SoftwareStartConv(ADC1); //使能指定的ADC1软件转换启动功能
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); //等待转换结束
	return ADC_GetConversionValue(ADC1); //返回最近1次ADC1规则组的转换结果
}

// 3 获取电池电压
void BAT_GetVoltage(void)
{
	float V;
	Aver_Filter((float)BAT_GetAdc(ADC_Channel_0), &BAT.BattAdc, 6); //滑动滤波一下电压值，提高精度
	if(BAT.BattAdc)
	{
		V=BAT.BattAdc*BAT.BattRealV/4095.0f; //BattAdc*3.31/(12位精度)
	}
	BAT.BattMeasureV=2*V; //根据原理图，电源实际电压=ADC测量电压*2
	printf("Test Voltage :%0.2f   temp:%0.0f \r\n ", BAT.BattMeasureV, BAT.BattAdc);
}

// 4 低电量报警
void BAT_LowVoltageAlarm(void)
{
	static u8 cnt=0, cnt1=0;
	BAT_GetVoltage();
	if(Airplane_Enable) //定义在control.c中
	{
		if(BAT.BattMeasureV<BAT.BattAlarmV) //飞行时测量
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
		if(BAT.BattMeasureV<3.7f) //落地时测量(380mh时是3.5V)
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

// 5 开关WiFi模块
void WiFi_Switch(u8 flag)
{
	if(flag) //开启WiFi
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4); //ESP_EN
		GPIO_SetBits(GPIOA, GPIO_Pin_5); //ESP_BOOT
	}
	else     //关闭WiFi
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_4); //ESP_EN
		GPIO_ResetBits(GPIOA, GPIO_Pin_5); //ESP_BOOT
	}
}

// 6 开关NRF24L01模块
void NRF24L01_Switch(u8 flag)
{
	if(flag) //开启NRF2401
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_8);   //NRF_CE
		NRF24L01_WriteReg(W_REGISTER+CONFIG, 0x0A);	 
	}
	else     //关闭NRF2401
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_8); //NRF_CE
		NRF24L01_WriteReg(W_REGISTER+CONFIG, 0x0F);	 
	}
}
