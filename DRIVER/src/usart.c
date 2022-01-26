/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/20
�����ܣ�USART����
��    ע��USART1&2��NVIC���ú�IRQ_Handler��nvic.c��
		  ע��target�е�Use MicroLIBһ��Ҫ�򹳣�������
*******************************************************************************/
#include "stm32f4xx.h"
#include "usart.h"
#include "stdio.h"

//****************************************A
// 1 USART1��GPIO����
void USART1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//PA9/Tx���� �����������ģʽ PuPd��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 		//����ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//�Ȳ�����Ҳ������
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//PA10/Rx���� ������������ģʽ OType��Speed��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 	//����ģʽ??Ϊʲô����_IN?
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 	//��������ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 2 USART1�ĳ�ʼ��
void USART1_Init(u32 Baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART1_GPIO_Config(); //������USART1��Ҫ��GPIO����
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	USART_InitStructure.USART_BaudRate = Baudrate;					//ָ��������
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx; 	//����ģʽ
	USART_InitStructure.USART_Parity = USART_Parity_No; 			//��У��λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 			//һλֹͣλ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//�ֳ���λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE); //���ڽ����ж�ʹ��
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE); //���ڿ����ж�ʹ��
	
	USART_Cmd(USART1, ENABLE); //ʹ��USART1
}

//****************************************B
// 3 USART2��GPIO����
void USART2_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//PA2/Tx���� �����������ģʽ PuPd�������� ESP_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 	//����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//PA3/Rx���� ������������ģʽ OType��Speed�������� ESP_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 	//����ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 	//����ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 4 USART2�ĳ�ʼ��
void USART2_Init(u32 Baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART2_GPIO_Config(); //������USART2��Ҫ��GPIO����
	
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //USART2��APB1��USART1��APB2
	
	USART_InitStructure.USART_BaudRate = Baudrate;				//ָ��������
	USART_InitStructure.USART_Mode = USART_Mode_Tx; 			//����ģʽ
	USART_InitStructure.USART_Parity = USART_Parity_No; 		//��У��λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 		//һλֹͣλ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ���λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART2, ENABLE); //ʹ��USART2
}

//****************************************C
// 5 ��printf()�ض���USART1����ʽ����
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (u8)ch); //USART1�ķ��ͺ���
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET) //�ȴ��������
	{}
	return ch;
}

//****************************************D
// 6 USART����ָ�����ȵ�����
//������WIFI���ι��ܣ������ݴ�USART2����ESP8266��Ȼ��ESP8266�ٴ�����λ��
//��δ��WIFI���ι��ܣ������ݴ�USART1��TX&RX��elink�������ٴ�����λ�������ߵ���
void USART_Send(u8 *data, u8 length)
{
	u8 i;
//	#if defined (WIFI_DEBUG) //����WIFI����
//		for(i=0; i<length; i++)
//		{
//			USART_SendData(USART2, *(data+i));
//			while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
//			{}
//		}
//	#else //���ߵ���
		for(i=0; i<length; i++)
		{
			USART_SendData(USART1, *(data+i));
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
			{}
		}
//	#endif
}

//****************************************E
// 7 USART1&2��NVIC���ú�IRQ_Handler��nvic.c��
