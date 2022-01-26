/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/21
�����ܣ�RGBȫ�ʵ�����
*******************************************************************************/
#include "stm32f4xx.h"
#include "rgb.h"
#include "delay.h"
#include "stdlib.h"
#include "structconfig.h"

//�궨��PB9��ƽ
#define SET1 GPIOB->BSRRL|=1<<9; //1����9λ��0x0A00����BSRRL��λ��õ��µ�BSRRL����bit8(�ھ�λ)��1
#define SET0 GPIOB->BSRRH|=1<<9; //BSRRH��BRy��1���͵�ƽ��ͬ��L��BRy=1��Ϊ�ߵ�ƽ

u8 Run_flag=1;

//****************************************A
// 1 ����RGB��GPIO
void RGB_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//PB9 RGB������GPIO
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //����Ϊ���٣���Ϊȷ��RGB����Ҫ���뼶��ʱ
	//��100MHz�������л���ƽΪ10ns����ִ�г���Ҳ��ʱ�䣬��˴���10ns����˸�����100MHz�һ�����__nop()
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//****************************************B
// 2 д0 H300ns+L900ns
void RGB_Write0(void)
{
	u8 cnt1=3, cnt2=9;
	
	SET1;
	while(cnt1--)
	{
		__nop(); //һ��������ʱ��Ϊ10ns��������ƫ����һ�Ҫ����while�ĵ���
	}
	SET0;
	while(cnt2--)
	{
		__nop();
		__nop();
	}
}

// 3 д1 H600ns+L600ns
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

// 4 RGB��λ L80us
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

// 5 дһ���ֽڵ�����
void RGB_Write_Byte(u8 data)
{
	int i=0;
	for(i=0; i<8; i++)
	{
		if((data<<i)&0x80) //data����0-7λ���κ�10000000��λ�룬ֻҪ��һ����ִ��д1
			RGB_Write1();
		else
			RGB_Write0();
	}
}

// 6 ����һ���Ƶ�ɫ�� GRBx RGB��
void RGB_Set_Color(u8 red, u8 green, u8 blue) //ʹ�ú���Ӧ����RGB!
{
	RGB_Write_Byte(green);
	RGB_Write_Byte(red);
	RGB_Write_Byte(blue);
}

//****************************************C
// 7 ��� R			RGB CYM OW off
void RGB_Red(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0xFF, 0, 0);
	}
}

// 8 �̵� G
void RGB_Green(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0, 0xFF, 0);
	}
}

// 9 ���� B
void RGB_Blue(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0, 0, 0xFF);
	}
}

// 10 ��� C
void RGB_Cyan(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0, 0xFF, 0xFF);
	}
}

// 11 �Ƶ� Y
void RGB_Yellow(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0xFF, 0xFF, 0);
	}
}

// 12 Ʒ��� M
void RGB_Magnet(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0xFF, 0, 0xFF);
	}
}

// 13 �ȵ� O
void RGB_Orange(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0xFF, 0x10, 0);
	}
}

// 14 �׵� W
void RGB_White(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0x0F, 0x0F, 0x0F);
	}
}

// 15 ����任��ɫ
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

// 16 ���
void RGB_Off(void)
{
	u8 i;
	for(i=0;i<4;i++)
	{
		RGB_Set_Color(0, 0, 0);
	}
}

//****************************************D
// 17 ����״̬����������
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

// 18 ������У׼��ɣ�������˸5��
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

// 19 ���ٶȼ�У׼��ɣ������˸5��
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

//// 20 ��ѹ��У׼��ɣ�������˸5��
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

// 21 WIFI������ʾ�������
void RGB_WIFI_Switch(void)
{
	static uint8_t cnt=0,flag = 0;
	if(WiFi_LEDflag==1)//wifi����ָʾ��
	{
		if(cnt++>6) 
		{
			cnt = 0;
			WiFi_LEDflag = 0;
			Run_flag = 1; //������rgb.c��ͷ
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
	else if(WiFi_LEDflag==2)//wifi�ر�ָʾ��
	{
		if(cnt++>6) 
		{
			cnt = 0;
			WiFi_LEDflag = 0;
			Run_flag = 1; //�����е�
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

// 22 �͵����������
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

// 24 ����������������
void RGB_Unlock(uint8_t N, uint8_t flag)
{
	static uint8_t cnt = 0;
	if(flag && cnt++>N)
	{ 
	 cnt = 0;
	 RGB_Rand();	
	}	
}

//// 25 ����OneNET û�õ� 
//void OneNET_LED(uint8_t color[],uint8_t num)
//{
//	uint8_t i;
//	for(i=0;i<num;i++)
//	{
//		RGB_Set_Color(color[1],color[0],color[2]);
//	}
//}
