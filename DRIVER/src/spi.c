/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/08/31
程序功能：SPI设置
*******************************************************************************/
#include "spi.h"

// 1 SPI2初始化
void SPI2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	//GPIO初始化 等同于SPI_GPIOConfig()
	//复用PB13, PB14, PB15为SPI对应引脚
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13|GPIO_PinSource14|GPIO_PinSource15, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);    
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; //SCK MISO MOSI
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; //复用模式
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //SPI2 SCK
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //SPI2 MISO
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //SPI2 MOSI
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SPI2模式设置
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master; //配置为主机模式
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex; //双线全双工模式
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b; //8位帧结构
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low; //空状态为低电平
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge; //第一个时钟沿捕获
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft; //NSS信号软件管理 所以只有SPI的配置三根线！！！！！！
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8; //SPI的波特率 PCLK(50M)/8=6.25M
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB; //先发送高位
	SPI_InitStructure.SPI_CRCPolynomial=7; //CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure); //一共5个SPI，这里用SPI2
	
	SPI_Cmd(SPI2, ENABLE); 
}

// 2 SPI2通信速度设置 即改变其预分频系数
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI_Cmd(SPI2, DISABLE); //失能SPI2
	SPI2->CR1&=~(0x07<<3); //清除原来的设置
	SPI2->CR1|=SPI_BaudRatePrescaler;
	SPI_Cmd(SPI2, ENABLE); //重新使能SPI2
}

// 3 SPI2写读一个字节
u8 SPI2_WriteReadByte(u8 data) //data--要发送的数据(write)  u8返回读取到的数据(read)
{
	u8 retry=0;
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)) //TXE--发送缓冲器空闲标志 =1则发送缓冲器为空，可以写下一个待发送的数据进入缓冲器中
	{//即(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)==RESET)
		retry++;
		if(retry>200) return 0; //TXE=0即发送缓冲器非空(还没发送完data)，且超过200次retry
	}
	SPI_I2S_SendData(SPI2, data); //发送1个字节 Write
	retry=0;
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)) //RXNE--接收缓冲器非空 =1表明在接收缓冲器中包含有效的接收数据 读SPI数据寄存器可清除此标志
	{
		retry++;
		if(retry>200) return 0; //RXNE=0即接收缓冲器空(数据还没读到)，且超过200次retry
	} //=1 跳出
	return SPI_I2S_ReceiveData(SPI2); //返回1个字节 Read 发送/接收都是DR寄存器 因为移位寄存器！
}
