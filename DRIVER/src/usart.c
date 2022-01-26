/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/20
程序功能：USART驱动
备    注：USART1&2的NVIC配置和IRQ_Handler在nvic.c中
		  注意target中的Use MicroLIB一定要打钩！！！！
*******************************************************************************/
#include "stm32f4xx.h"
#include "usart.h"
#include "stdio.h"

//****************************************A
// 1 USART1的GPIO配置
void USART1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//PA9/Tx引脚 复用推挽输出模式 PuPd不用配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 		//复用模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//既不上拉也不下拉
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//PA10/Rx引脚 复用上拉输入模式 OType和Speed不用配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 	//复用模式??为什么不是_IN?
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 	//上拉输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 2 USART1的初始化
void USART1_Init(u32 Baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART1_GPIO_Config(); //先配置USART1需要的GPIO引脚
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	USART_InitStructure.USART_BaudRate = Baudrate;					//指定波特率
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx; 	//发送模式
	USART_InitStructure.USART_Parity = USART_Parity_No; 			//无校验位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 			//一位停止位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//字长八位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE); //串口接收中断使能
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE); //串口空闲中断使能
	
	USART_Cmd(USART1, ENABLE); //使能USART1
}

//****************************************B
// 3 USART2的GPIO配置
void USART2_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//PA2/Tx引脚 复用推挽输出模式 PuPd不用配置 ESP_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 	//复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//PA3/Rx引脚 复用上拉输入模式 OType和Speed不用配置 ESP_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 	//输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 	//上拉模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 4 USART2的初始化
void USART2_Init(u32 Baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART2_GPIO_Config(); //先配置USART2需要的GPIO引脚
	
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //USART2是APB1，USART1是APB2
	
	USART_InitStructure.USART_BaudRate = Baudrate;				//指定波特率
	USART_InitStructure.USART_Mode = USART_Mode_Tx; 			//发送模式
	USART_InitStructure.USART_Parity = USART_Parity_No; 		//无校验位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 		//一位停止位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长八位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART2, ENABLE); //使能USART2
}

//****************************************C
// 5 将printf()重定向到USART1；定式函数
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (u8)ch); //USART1的发送函数
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET) //等待发送完成
	{}
	return ch;
}

//****************************************D
// 6 USART发送指定长度的数据
//若开启WIFI调参功能，则数据从USART2传到ESP8266，然后经ESP8266再传到上位机
//若未开WIFI调参功能，则数据从USART1的TX&RX经elink下载器再传到上位机，有线调参
void USART_Send(u8 *data, u8 length)
{
	u8 i;
//	#if defined (WIFI_DEBUG) //开启WIFI调参
//		for(i=0; i<length; i++)
//		{
//			USART_SendData(USART2, *(data+i));
//			while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
//			{}
//		}
//	#else //有线调参
		for(i=0; i<length; i++)
		{
			USART_SendData(USART1, *(data+i));
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
			{}
		}
//	#endif
}

//****************************************E
// 7 USART1&2的NVIC配置和IRQ_Handler在nvic.c中
