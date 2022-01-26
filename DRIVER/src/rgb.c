/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/21
程序功能：RGB全彩灯设置
*******************************************************************************/
#include "stm32f4xx.h"
#include "rgb.h"
#include "delay.h"
#include "stdlib.h"
#include "structconfig.h"

//宏定义PB9电平
#define SET1 GPIOB->BSRRL|=1<<9; //1左移9位即0x0A00并与BSRRL按位或得到新的BSRRL，即bit8(第九位)置1
#define SET0 GPIOB->BSRRH|=1<<9; //BSRRH的BRy置1，低电平；同理L，BRy=1则为高电平

u8 Run_flag=1;

//****************************************A
// 1 配置RGB的GPIO
void RGB_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//PB9 RGB单总线GPIO
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //必须为高速，因为确定RGB灯需要纳秒级延时
	//而100MHz理论上切换电平为10ns，但执行程序也花时间，因此大于10ns，因此更该用100MHz且还需用__nop()
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//****************************************B
// 2 写0 H300ns+L900ns
void RGB_Write0(void)
{
	u8 cnt1=3, cnt2=9;
	
	SET1;
	while(cnt1--)
	{
		__nop(); //一个的运行时间为10ns，但是有偏差，而且还要考虑while的调用
	}
	SET0;
	while(cnt2--)
	{
		__nop();
		__nop();
	}
}

// 3 写1 H600ns+L600ns
void RGB_Write1(void)
{
	u8 cnt1=6, cnt2=6;
	
	SET1;
	while(cnt1--)
	{
		__nop();
		__nop();
	}
	SET0;
	while(cnt2--)
	{
		__nop();
		__nop();
	}
}

// 4 RGB复位 L80us
void RGB_Reset(void)
{
	u16 cnt1=600, cnt2=1;
	
	SET0;
	while(cnt1--)
		__nop();
	SET1;
	while(cnt2--)
		__nop();
}

// 5 写一个字节的数据
void RGB_Write_Byte(u8 data)
{
	int i=0;
	for(i=0; i<8; i++)
	{
		if((data<<i)&0x80) //data左移0-7位依次和10000000按位与，只要有一致则执行写1
			RGB_Write1();
		else
			RGB_Write0();
	}
}

// 6 设置一个灯的色彩 GRBx RGB√
void RGB_Set_Color(u8 red, u8 green, u8 blue) //使用函数应该是RGB!
{
	RGB_Write_Byte(green);
	RGB_Write_Byte(red);
	RGB_Write_Byte(blue);
}

//****************************************C
// 7 红灯 R			RGB CYM OW off
void RGB_Red(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0xFF, 0, 0);
	}
}

// 8 绿灯 G
void RGB_Green(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0, 0xFF, 0);
	}
}

// 9 蓝灯 B
void RGB_Blue(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0, 0, 0xFF);
	}
}

// 10 青灯 C
void RGB_Cyan(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0, 0xFF, 0xFF);
	}
}

// 11 黄灯 Y
void RGB_Yellow(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0xFF, 0xFF, 0);
	}
}

// 12 品红灯 M
void RGB_Magnet(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0xFF, 0, 0xFF);
	}
}

// 13 橙灯 O
void RGB_Orange(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0xFF, 0x10, 0);
	}
}

// 14 白灯 W
void RGB_White(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0x0F, 0x0F, 0x0F);
	}
}

// 15 随机变换颜色
void RGB_Rand(void)
{
	u8 i, red=0, green=0, blue=0;
	for(i=0; i<4; i++)
	{
		red=rand()%255;
		green=rand()%255;
		blue=rand()%255;
		RGB_Set_Color(red, green, blue);
	}
}

// 16 灭灯
void RGB_Off(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0, 0, 0);
	}
}

//****************************************D
// 17 飞行状态，两红两白
void RGB_Fly(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		if(i<2)
			RGB_Set_Color(0xFF, 0, 0);
		else
			RGB_Set_Color(0xFF, 0xFF, 0xFF);
	}
}

// 18 陀螺仪校准完成，蓝灯闪烁5次
void RGB_GYRO_Calib(void)
{
	int i;
	RGB_Off();
//	delay_ms(100);
	for(i=0; i<5; i++)
	{
		RGB_Blue();
		delay_ms(100);
		RGB_Off();
		delay_ms(100);
	}
	RGB_Off();
//	delay_ms(100);
}

// 19 加速度计校准完成，青灯闪烁5次
void RGB_ACC_Calib(void)
{
	int i;
	RGB_Off();
//	delay_ms(100);
	for(i=0; i<5; i++)
	{
		RGB_Cyan();
		delay_ms(100);
		RGB_Off();
		delay_ms(100);
	}
	RGB_Off();
//	delay_ms(100);
}

//// 20 气压计校准完成，蓝灯闪烁5次
//void RGB_BARO_Calib(void)
//{
//	int i;
//	RGB_Off();
//	delay_ms(100);
//	for(i=0; i<5; i++)
//	{
//		RGB_Blue();
//		delay_ms(100);
//		RGB_Off();
//		delay_ms(100);
//	}
//	RGB_Off();
//	delay_ms(100);
//}

// 21 WIFI开关提示，青灯闪
void RGB_WIFI_Switch(void)
{
	static uint8_t cnt=0,flag = 0;
	if(WiFi_LEDflag==1)//wifi开启指示灯
	{
		if(cnt++>6) 
		{
			cnt = 0;
			WiFi_LEDflag = 0;
			Run_flag = 1; //定义在rgb.c开头
			RGB_Off();
		}
		else 
		{
			if(flag)
			{
				flag = 0;
				RGB_Blue();
			}
			else
			{
				flag = 1;
				RGB_Off();
			}
		}
	}
	else if(WiFi_LEDflag==2)//wifi关闭指示灯
	{
		if(cnt++>6) 
		{
			cnt = 0;
			WiFi_LEDflag = 0;
			Run_flag = 1; //打开运行灯
			RGB_Off();
		}else
		{
			if(flag)
			{
				flag = 0;
				RGB_Orange();
			}
			else
			{
				flag = 1;
				RGB_Off();
			}
		}
	}	
}

// 22 低电量，红灯闪
void RGB_BAT_Alarm(void)
{
	static u8 flag = 0;
	if(RGB_BATflag)
	{
		if(flag)
		{
			flag = 0;
			RGB_Red();
		}
		else
		{
			flag = 1;
			RGB_Off();
		}
	}
}

// 24 四轴解锁，随机亮灯
void RGB_Unlock(uint8_t N, uint8_t flag)
{
	static uint8_t cnt = 0;
	if(flag && cnt++>N)
	{ 
	 cnt = 0;
	 RGB_Rand();	
	}	
}

//// 25 连接OneNET 没用到 
//void OneNET_LED(uint8_t color[],uint8_t num)
//{
//	uint8_t i;
//	for(i=0;i<num;i++)
//	{
//		RGB_Set_Color(color[1],color[0],color[2]);
//	}
//}
