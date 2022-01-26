/*******************************************************************************
��Ŀ���ƣ�Expeditioner-I: Fatih
�������ߣ�Mingyan Zhou/Derek Zhou/������
��д���ڣ�2020/08/31
�����ܣ�SPI����
*******************************************************************************/
#include "spi.h"

// 1 SPI2��ʼ��
void SPI2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	//GPIO��ʼ�� ��ͬ��SPI_GPIOConfig()
	//����PB13, PB14, PB15ΪSPI��Ӧ����
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13|GPIO_PinSource14|GPIO_PinSource15, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);    
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; //SCK MISO MOSI
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; //����ģʽ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //SPI2 SCK
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //SPI2 MISO
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //SPI2 MOSI
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SPI2ģʽ����
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master; //����Ϊ����ģʽ
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex; //˫��ȫ˫��ģʽ
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b; //8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low; //��״̬Ϊ�͵�ƽ
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge; //��һ��ʱ���ز���
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft; //NSS�ź�������� ����ֻ��SPI�����������ߣ�����������
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8; //SPI�Ĳ����� PCLK(50M)/8=6.25M
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB; //�ȷ��͸�λ
	SPI_InitStructure.SPI_CRCPolynomial=7; //CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure); //һ��5��SPI��������SPI2
	
	SPI_Cmd(SPI2, ENABLE); 
}

// 2 SPI2ͨ���ٶ����� ���ı���Ԥ��Ƶϵ��
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI_Cmd(SPI2, DISABLE); //ʧ��SPI2
	SPI2->CR1&=~(0x07<<3); //���ԭ��������
	SPI2->CR1|=SPI_BaudRatePrescaler;
	SPI_Cmd(SPI2, ENABLE); //����ʹ��SPI2
}

// 3 SPI2д��һ���ֽ�
u8 SPI2_WriteReadByte(u8 data) //data--Ҫ���͵�����(write)  u8���ض�ȡ��������(read)
{
	u8 retry=0;
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)) //TXE--���ͻ��������б�־ =1���ͻ�����Ϊ�գ�����д��һ�������͵����ݽ��뻺������
	{//��(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==RESET)
		retry++;
		if(retry>200) return 0; //TXE=0�����ͻ������ǿ�(��û������data)���ҳ���200��retry
	}
	SPI_I2S_SendData(SPI2, data); //����1���ֽ� Write
	retry=0;
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)) //RXNE--���ջ������ǿ� =1�����ڽ��ջ������а�����Ч�Ľ������� ��SPI���ݼĴ���������˱�־
	{
		retry++;
		if(retry>200) return 0; //RXNE=0�����ջ�������(���ݻ�û����)���ҳ���200��retry
	} //=1 ����
	return SPI_I2S_ReceiveData(SPI2); //����1���ֽ� Read ����/���ն���DR�Ĵ��� ��Ϊ��λ�Ĵ�����
}
