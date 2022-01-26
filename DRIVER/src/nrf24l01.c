/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/09/01
�����ܣ�NRF24L01/Si24R1����
��    ע����Ҫ�ο�С�������nrf2401.c
*******************************************************************************/
#include "nrf24l01.h"
#include "nvic.h" //λ������
#include "spi.h"
#include "stdio.h"
#include "rgb.h"
#include "delay.h"
#include "remotedata.h"
#include "stdlib.h"
//#include "paramsave.h"

//****************************************A �궨��
//���ű�ע(CS/NSSʹ���������ʽ������6������)
/*	NRF_CSN 	PB12	CSN: Chip Select		Ƭѡ
	NRF_CE  	PA8		CE: Chip Enable			оƬʹ��
	NRF_IRQ 	PB2		IRQ: Interrupt Request	�ж�����
	SCK			PB13
	MISO		PB14
	MOSI		PB15									*/

//�������ŵ�ƽ(λ����������nvic.h�У�)
#define NRF_CSN PBout(12) //Port CSN: =0 SPI�ӿڿ�ʼ����
#define NRF_CE  PAout(8) //Port CE: оƬ�����źţ�����RX/TXģʽ  =1 оƬ��standby(����)ģʽ��ΪIdle-TX(�������)��RXģʽ
//Warning: ��ĺ궨��ûд��.h�ļ��� ���Բ�����������.c�ļ������ã�������

//��ȡ���ŵ�ƽ
#define NRFAddrMax 50 //NRF���һ���ֽڵ�ַ���Ϊ50
u8 NRFAddr=0xFF; //��ʼ��NRF���һ�ֽڵ�ַ

u8 NRF_TX_DATA[TX_PAYLO_WIDTH]; //NRF���ͻ����� 32λ
u8 NRF_RX_DATA[RX_PAYLO_WIDTH]; //NRF���ջ�����

u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0xFF}; //���͵�ַ ����0XFF�ɸı�
u8 RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0xFF}; //���յ�ַ

void NRF24L01_Config(void);
void NRF_GetAddr(void);

//****************************************B ��ʼ��
// 1 NRF24L01�ĳ�ʼ��
void NRF24L01_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12); //PB12 -> Chip Select
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8); //PA8 -> Chip Enable
	
	SPI2_Init(); //SPI2��ʼ��
	
	NRF24L01_Check();
	
	NRF_CSN=1; //Ƭѡ=1 SPI�ӿڲ�����
	NRF_CE=0; //оƬʹ�� standby->Idle-TX/RX
	
//	NRF24L01_Config(); //����NRF24L01�Ƿ���MCUͨ�� �ú����ڻ�ȡ��ַ����ֱ�� ���Դ˴�ע�͵�
}

//****************************************C ��д�Ĵ�����һ�����ݵ�ģ�麯��
// 2 д��1�ֽ����ݵ��Ĵ���
u8 NRF24L01_WriteReg(u8 reg, u8 value) //(reg, value)
{
	u8 status;
	NRF_CSN=0; //SPI�ӿڿ�ʼ����
	status=SPI2_WriteReadByte(reg); //����д��״̬�Ĵ�������
	SPI2_WriteReadByte(value); //д��1���ֽ�
	NRF_CSN=1;
	return status;
}

// 3 ��ȡ1�ֽ����ݵ��Ĵ���
u8 NRF24L01_ReadReg(u8 reg) //(reg)
{
	u8 reg_val;
	NRF_CSN=0;
	SPI2_WriteReadByte(reg); //���Ͷ�ȡ״̬�Ĵ�������
	reg_val=SPI2_WriteReadByte(0xff); //��ȡ1���ֽ�
	NRF_CSN=1;
	return reg_val;
}

// 4 дһ�����ݵ��Ĵ���
u8 NRF24L01_WriteBuf(u8 reg, u8 *pBuf, u8 len) //pBuf: һ�����ݵ�ָ��
{
	u8 status;
	int i;
	NRF_CSN=0;
	status=SPI2_WriteReadByte(reg);
	for(i=0; i<len; i++)
	{
		SPI2_WriteReadByte(*pBuf);
		pBuf++;
	}
	NRF_CSN=1;
	return status;
}

// 5 ��һ�����ݵ��Ĵ���
u8 NRF24L01_ReadBuf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status;
	int i;
	NRF_CSN=0;
	status=SPI2_WriteReadByte(reg);
	for(i=0; i<len; i++)
	{
		*pBuf=SPI2_WriteReadByte(0xff);
		pBuf++;
	}
	NRF_CSN=1;
	return status;
}

//****************************************D MCU��SPI���
// 6 ���NRF24L01��MCU��SPI�����Ƿ�ͨ������
u8 NRF24L01_TestConnection(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i; 	 
	NRF24L01_WriteBuf(W_REGISTER+TX_ADDR, buf, 5); //д������ֽڵĵ�ַ
	NRF24L01_ReadBuf(TX_ADDR, buf, 5); //����д��ĵ�ַ
	for(i=0; i<5; i++)
	if(buf[i]!=0XA5) break;	 							   
	if(i!=5) return 0; //���NRF24L01����
	return 1; //��⵽NRF24L01
}

// 7 NRF24L01ͨ�ż��ķ�װ
void NRF24L01_Check(void)
{
	while(!NRF24L01_TestConnection())
	{
		printf("\rNRF24L01 no connect...\r\n");
		RGB_Yellow(); //�ƵƳ���
	}
}

//****************************************E ����ģʽ���л��������������
// 8 �л�NRF24L01�Ĺ���ģʽ
void NRF_SetMode(u8 mode)
{
	if(mode==IT_TX)
	{
		NRF_CE=0;
		NRF24L01_WriteReg(W_REGISTER+CONFIG, IT_TX); //(reg, value) 0x20+0X00 д����Reg+����ģʽ IT_TX=0x0E
		NRF24L01_WriteReg(W_REGISTER+STATUS, 0X7E); //��������жϣ���ֹһ��ȥ����ģʽ�ʹ����ж�
		NRF_CE=1;
//		delay_us(20);
	}
	else
	{
		NRF_CE=0;
		NRF24L01_WriteReg(W_REGISTER+CONFIG, IT_RX); //����Ϊ����ģʽ
		NRF24L01_WriteReg(W_REGISTER+STATUS, 0X7E); //��������жϣ���ֹһ��ȥ����ģʽ�ʹ����ж�
		NRF_CE=1;
		delay_us(200); //�ȷ��ͳ�һЩ
	}
}

// 9 NRF24L01�����������ã�����ʼ��Ϊ����ģʽ-RX
void NRF24L01_Config(void) //void NRF_GetAddr(void)�õ�
{
	NRF_CE=0; //ʹ������������
	NRF24L01_WriteReg(W_REGISTER+SETUP_AW, 0x03); //����ͨ�ŵ�ַ�ĳ��ȣ�Ĭ��ֵ��0x03������ַ����Ϊ5�ֽ�
	NRF24L01_WriteBuf(W_REGISTER+TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH); //дTX�ڵ��ַ
	NRF24L01_WriteBuf(W_REGISTER+RX_ADDR_P0, (u8*)TX_ADDRESS, RX_ADR_WIDTH); //����TX�ڵ��ַ����ҪΪ��ʹ��ACK
	NRF24L01_WriteReg(W_REGISTER+SETUP_RETR, 0x1A); //�����Զ��ط����ʱ�䣺500us+86us; ����ط�������10��
	NRF24L01_WriteReg(W_REGISTER+EN_RXADDR, 0x01); //ʹ��ͨ��0�Ľ��յ�ַ
	NRF24L01_WriteReg(W_REGISTER+EN_AA, 0x01); //ʹ��ͨ��0�Զ�Ӧ��
	NRF24L01_WriteReg(W_REGISTER+RX_PW_P0, RX_PAYLO_WIDTH); //ѡ��ͨ��0����Ч���ݿ��
	NRF24L01_WriteBuf(W_REGISTER+RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); //дRX�ڵ��ַ
	NRF24L01_WriteReg(W_REGISTER+RF_CH, 30); //����RFͨ��Ϊ40Hz(1-64Hz������)
	NRF24L01_WriteReg(W_REGISTER+RF_SETUP, 0x27); //����TX���������0db���棬2Mbps������������ر�
	NRF_SetMode(IT_RX); //Ĭ��Ϊ����ģʽ		  //ע�⣺����������ر�/����ֱ��Ӱ��ͨ�ţ�Ҫ������������Ҫ�رն��ر�-0x07
	NRF_CE=1; //ʹ����������
}

//****************************************F �������ݰ��Լ���Ӧ���жϴ���
// 10 NRF24L01����һ������
void NRF24L01_TxPacket(u8 *txbuf) //write ֻ��remotedata����ù�
{
	NRF_CE=0;
	NRF24L01_WriteBuf(W_REGISTER+TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH); //дTX�ڵ��ַ
	NRF24L01_WriteBuf(W_REGISTER+RX_ADDR_P0, (u8*)TX_ADDRESS, RX_ADR_WIDTH); //����TX�ڵ��ַ����ҪΪ��ʹ��ACK
	NRF24L01_WriteBuf(W_RX_PAYLOAD, txbuf, TX_PAYLO_WIDTH); //д���ݵ�TX_BUFF
	NRF24L01_WriteReg(W_REGISTER+CONFIG, 0x0E); //����Ϊ����ģʽ�����������ж� ����������
	NRF24L01_WriteReg(W_REGISTER+STATUS, 0X7E); //��������жϣ���ֹһ��ȥ����ģʽ�ʹ����ж�
	NRF_CE=1;
	delay_us(10); //CE�ߵ�ƽ����10us
}

// 11 NRF24L01����һ������
void NRF24L01_RxPacket(u8 *rxbuf) //���ݽ�����Ĭ��״̬ read ֻ��EXTI2_IRQHandler()����ù�
{
	NRF_CE=0;
	NRF24L01_ReadBuf(R_RX_PAYLOAD,rxbuf,TX_PAYLO_WIDTH); //��ȡRX����Ч����
	NRF24L01_WriteReg(FLUSH_RX,0xff); //���RX FIFO(ע�⣺��仰�ܱ�Ҫ��)
	NRF_CE=1;
}


// 12 NRF24L01ȫ˫�����ⲿ�жϴ�����
void EXTI2_IRQHandler(void) //exti��gpio�������ʼ����exti.c�У�
{
	u8 sta;
	if(EXTI_GetITStatus(EXTI_Line2)!=RESET) //IRQHandler�̶���ʽ
	{
		
		NRF_CE=0;
		sta=NRF24L01_ReadReg(R_REGISTER+STATUS); //��ȡSTATUS�е����ݣ��Ա��ж�����ʲô�ж�Դ������IRQ�ж�
		// 1 ��������ж� TX_OK write��
		if(sta&TX_OK)
		{
			NRF_SetMode(IT_RX);
			NRF24L01_WriteReg(W_REGISTER+STATUS, TX_OK); //���������ɱ�־
			NRF24L01_WriteReg(FLUSH_TX, 0xff); //���TX_FIFO ��������ܵ������ݶ�ʧ
//			printf("NRF24L01 sent data successfully!\r\n"); //������Ͳ��ɹ� �����ж�3
		}
		// 2 ��������ж� RX_OK read��
		if(sta&RX_OK)
		{
			NRF24L01_RxPacket(NRF_RX_DATA);
			Remote_Data_ReceiveAnalysis();
			NRF24L01_WriteReg(W_REGISTER+STATUS,RX_OK); //���������ɱ�־
//			printf("NRF24L01 received data successfully!\r\n");
		}
		// 3 �ﵽ����ط������ж� MAX_TX (���ݻ�û�з��ͳɹ�)
		if(sta&MAX_TX)
		{
			NRF_SetMode(IT_RX);
			NRF24L01_WriteReg(W_REGISTER+STATUS,MAX_TX); //����ﵽ����ط���־
			NRF24L01_WriteReg(FLUSH_TX,0xff); //���TX_FIFO 
//			printf("NRF24L01 has sent max data!\r\n"); 
		}
		EXTI_ClearITPendingBit(EXTI_Line2); //����жϱ�־λ
	}
}

//****************************************G NRF��ַ������
// 13 ���ɻ��ϵ�NRF24L01��ȡһ����ַ
void NRF_GetAddr(void) //�˺�����Ҫ��С��������ң�����Ķ�Ƶ��������ʹ�ã�����NRFͨ�Ų��ɹ�
{					   //����Լ�����ң������ֱ��ʹ�ù̶���ַ
	if(NRFAddr>NRFAddrMax) //��NRFAddr����NRFAddrMax(50)����˵����ʱNRF��δ��ʼ�����
	{
		srand(SysTick->VAL); //�����������
//		printf("SysTick->VAL:%d\r\n",SysTick->VAL);
		NRFAddr=rand()%NRFAddrMax; //�����ȡNRF���һ�ֽڵ�ַ(��ַ��0~50)
//		PID_WriteFlash(); //����˵�ַflash
	}
	else if(NRFAddr!=TX_ADDRESS[TX_ADR_WIDTH-1])
	{
		TX_ADDRESS[TX_ADR_WIDTH-1]=NRFAddr;
		RX_ADDRESS[TX_ADR_WIDTH-1]=NRFAddr;
		NRF24L01_Config();
//		printf("NRFAddr:%d\r\n",NRFAddr);
	}
}

//****************************************H ͨ�Ų��Ժ���
// 14 NRF24L01ͨ�Ų��Ժ���
void NRF_Test(void) //����ר�ã�
{
	u8 t=0;
	static u8 mode, key;
	mode=' ';
	key=mode;
	for(t=0; t<32; t++)
	{
		key++;
		if(key>('~'))key=' ';
		NRF_TX_DATA[t]=key;	
	}
	mode++; 
	if(mode>'~')mode=' ';  	  		
	NRF24L01_TxPacket(NRF_TX_DATA);
}
