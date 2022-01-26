/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/09/01
程序功能：NRF24L01/Si24R1驱动
备    注：主要参考小马哥四轴nrf2401.c
*******************************************************************************/
#include "nrf24l01.h"
#include "nvic.h" //位操作！
#include "spi.h"
#include "stdio.h"
#include "rgb.h"
#include "delay.h"
#include "remotedata.h"
#include "stdlib.h"
//#include "paramsave.h"

//****************************************A 宏定义
//引脚备注(CS/NSS使用软件管理方式，所以6个引脚)
/*	NRF_CSN 	PB12	CSN: Chip Select		片选
	NRF_CE  	PA8		CE: Chip Enable			芯片使能
	NRF_IRQ 	PB2		IRQ: Interrupt Request	中断请求
	SCK			PB13
	MISO		PB14
	MOSI		PB15									*/

//设置引脚电平(位操作定义在nvic.h中！)
#define NRF_CSN PBout(12) //Port CSN: =0 SPI接口开始工作
#define NRF_CE  PAout(8) //Port CE: 芯片开启信号，激活RX/TX模式  =1 芯片从standby(待机)模式变为Idle-TX(发射空闲)或RX模式
//Warning: 你的宏定义没写在.h文件中 所以不能再在其他.c文件中引用！！！！

//读取引脚电平
#define NRFAddrMax 50 //NRF最后一个字节地址最大为50
u8 NRFAddr=0xFF; //初始化NRF最后一字节地址

u8 NRF_TX_DATA[TX_PAYLO_WIDTH]; //NRF发送缓冲区 32位
u8 NRF_RX_DATA[RX_PAYLO_WIDTH]; //NRF接收缓冲区

u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0xFF}; //发送地址 最后的0XFF可改变
u8 RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0xFF}; //接收地址

void NRF24L01_Config(void);
void NRF_GetAddr(void);

//****************************************B 初始化
// 1 NRF24L01的初始化
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
	
	SPI2_Init(); //SPI2初始化
	
	NRF24L01_Check();
	
	NRF_CSN=1; //片选=1 SPI接口不工作
	NRF_CE=0; //芯片使能 standby->Idle-TX/RX
	
//	NRF24L01_Config(); //配置NRF24L01是否与MCU通信 该函数在获取地址后再直行 所以此处注释掉
}

//****************************************C 读写寄存器与一组数据的模块函数
// 2 写入1字节数据到寄存器
u8 NRF24L01_WriteReg(u8 reg, u8 value) //(reg, value)
{
	u8 status;
	NRF_CSN=0; //SPI接口开始工作
	status=SPI2_WriteReadByte(reg); //发送写入状态寄存器命令
	SPI2_WriteReadByte(value); //写入1个字节
	NRF_CSN=1;
	return status;
}

// 3 读取1字节数据到寄存器
u8 NRF24L01_ReadReg(u8 reg) //(reg)
{
	u8 reg_val;
	NRF_CSN=0;
	SPI2_WriteReadByte(reg); //发送读取状态寄存器命令
	reg_val=SPI2_WriteReadByte(0xff); //读取1个字节
	NRF_CSN=1;
	return reg_val;
}

// 4 写一组数据到寄存器
u8 NRF24L01_WriteBuf(u8 reg, u8 *pBuf, u8 len) //pBuf: 一组数据的指针
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

// 5 读一组数据到寄存器
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

//****************************************D MCU的SPI检测
// 6 检查NRF24L01与MCU的SPI总线是否通信正常
u8 NRF24L01_TestConnection(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i; 	 
	NRF24L01_WriteBuf(W_REGISTER+TX_ADDR, buf, 5); //写入五个字节的地址
	NRF24L01_ReadBuf(TX_ADDR, buf, 5); //读出写入的地址
	for(i=0; i<5; i++)
	if(buf[i]!=0XA5) break;	 							   
	if(i!=5) return 0; //检测NRF24L01错误
	return 1; //检测到NRF24L01
}

// 7 NRF24L01通信检查的封装
void NRF24L01_Check(void)
{
	while(!NRF24L01_TestConnection())
	{
		printf("\rNRF24L01 no connect...\r\n");
		RGB_Yellow(); //黄灯长亮
	}
}

//****************************************E 工作模式的切换与基本参数设置
// 8 切换NRF24L01的工作模式
void NRF_SetMode(u8 mode)
{
	if(mode==IT_TX)
	{
		NRF_CE=0;
		NRF24L01_WriteReg(W_REGISTER+CONFIG, IT_TX); //(reg, value) 0x20+0X00 写配置Reg+发射模式 IT_TX=0x0E
		NRF24L01_WriteReg(W_REGISTER+STATUS, 0X7E); //清除所有中断，防止一进去发送模式就触发中断
		NRF_CE=1;
//		delay_us(20);
	}
	else
	{
		NRF_CE=0;
		NRF24L01_WriteReg(W_REGISTER+CONFIG, IT_RX); //配置为接收模式
		NRF24L01_WriteReg(W_REGISTER+STATUS, 0X7E); //清除所有中断，防止一进去接收模式就触发中断
		NRF_CE=1;
		delay_us(200); //比发送长一些
	}
}

// 9 NRF24L01基本参数设置，并初始化为接收模式-RX
void NRF24L01_Config(void) //void NRF_GetAddr(void)用到
{
	NRF_CE=0; //使能引脚先拉低
	NRF24L01_WriteReg(W_REGISTER+SETUP_AW, 0x03); //配置通信地址的长度，默认值是0x03，即地址长度为5字节
	NRF24L01_WriteBuf(W_REGISTER+TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH); //写TX节点地址
	NRF24L01_WriteBuf(W_REGISTER+RX_ADDR_P0, (u8*)TX_ADDRESS, RX_ADR_WIDTH); //设置TX节点地址，主要为了使能ACK
	NRF24L01_WriteReg(W_REGISTER+SETUP_RETR, 0x1A); //设置自动重发间隔时间：500us+86us; 最大重发次数：10次
	NRF24L01_WriteReg(W_REGISTER+EN_RXADDR, 0x01); //使能通道0的接收地址
	NRF24L01_WriteReg(W_REGISTER+EN_AA, 0x01); //使能通道0自动应答
	NRF24L01_WriteReg(W_REGISTER+RX_PW_P0, RX_PAYLO_WIDTH); //选择通道0的有效数据宽度
	NRF24L01_WriteBuf(W_REGISTER+RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); //写RX节点地址
	NRF24L01_WriteReg(W_REGISTER+RF_CH, 30); //设置RF通道为40Hz(1-64Hz都可以)
	NRF24L01_WriteReg(W_REGISTER+RF_SETUP, 0x27); //设置TX发射参数，0db增益，2Mbps，低噪声增益关闭
	NRF_SetMode(IT_RX); //默认为接收模式		  //注意：低噪声增益关闭/开启直接影响通信，要开启都开启，要关闭都关闭-0x07
	NRF_CE=1; //使能引脚拉高
}

//****************************************F 发送数据包以及对应的中断处理
// 10 NRF24L01发送一包数据
void NRF24L01_TxPacket(u8 *txbuf) //write 只在remotedata里调用过
{
	NRF_CE=0;
	NRF24L01_WriteBuf(W_REGISTER+TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH); //写TX节点地址
	NRF24L01_WriteBuf(W_REGISTER+RX_ADDR_P0, (u8*)TX_ADDRESS, RX_ADR_WIDTH); //设置TX节点地址，主要为了使能ACK
	NRF24L01_WriteBuf(W_RX_PAYLOAD, txbuf, TX_PAYLO_WIDTH); //写数据到TX_BUFF
	NRF24L01_WriteReg(W_REGISTER+CONFIG, 0x0E); //设置为发送模式，开启所有中断 真正发送了
	NRF24L01_WriteReg(W_REGISTER+STATUS, 0X7E); //清除所有中断，防止一进去发送模式就触发中断
	NRF_CE=1;
	delay_us(10); //CE高电平持续10us
}

// 11 NRF24L01接收一包数据
void NRF24L01_RxPacket(u8 *rxbuf) //数据接收是默认状态 read 只在EXTI2_IRQHandler()里调用过
{
	NRF_CE=0;
	NRF24L01_ReadBuf(R_RX_PAYLOAD,rxbuf,TX_PAYLO_WIDTH); //读取RX的有效数据
	NRF24L01_WriteReg(FLUSH_RX,0xff); //清除RX FIFO(注意：这句话很必要！)
	NRF_CE=1;
}


// 12 NRF24L01全双工的外部中断处理函数
void EXTI2_IRQHandler(void) //exti的gpio配置与初始化在exti.c中！
{
	u8 sta;
	if(EXTI_GetITStatus(EXTI_Line2)!=RESET) //IRQHandler固定格式
	{
		
		NRF_CE=0;
		sta=NRF24L01_ReadReg(R_REGISTER+STATUS); //读取STATUS中的数据，以便判断是由什么中断源触发的IRQ中断
		// 1 发送完成中断 TX_OK write完
		if(sta&TX_OK)
		{
			NRF_SetMode(IT_RX);
			NRF24L01_WriteReg(W_REGISTER+STATUS, TX_OK); //清除发送完成标志
			NRF24L01_WriteReg(FLUSH_TX, 0xff); //清除TX_FIFO 不清除可能导致数据丢失
//			printf("NRF24L01 sent data successfully!\r\n"); //如果发送不成功 进入中断3
		}
		// 2 接收完成中断 RX_OK read完
		if(sta&RX_OK)
		{
			NRF24L01_RxPacket(NRF_RX_DATA);
			Remote_Data_ReceiveAnalysis();
			NRF24L01_WriteReg(W_REGISTER+STATUS,RX_OK); //清除发送完成标志
//			printf("NRF24L01 received data successfully!\r\n");
		}
		// 3 达到最大重发次数中断 MAX_TX (数据还没有发送成功)
		if(sta&MAX_TX)
		{
			NRF_SetMode(IT_RX);
			NRF24L01_WriteReg(W_REGISTER+STATUS,MAX_TX); //清除达到最大重发标志
			NRF24L01_WriteReg(FLUSH_TX,0xff); //清除TX_FIFO 
//			printf("NRF24L01 has sent max data!\r\n"); 
		}
		EXTI_ClearITPendingBit(EXTI_Line2); //清除中断标志位
	}
}

//****************************************G NRF地址的生成
// 13 给飞机上的NRF24L01获取一个地址
void NRF_GetAddr(void) //此函数需要与小马哥四轴的遥控器的对频函数联合使用，否则NRF通信不成功
{					   //如果自己做的遥控器可直接使用固定地址
	if(NRFAddr>NRFAddrMax) //当NRFAddr大于NRFAddrMax(50)，就说明此时NRF还未初始化完成
	{
		srand(SysTick->VAL); //给随机数种子
//		printf("SysTick->VAL:%d\r\n",SysTick->VAL);
		NRFAddr=rand()%NRFAddrMax; //随机获取NRF最后一字节地址(地址：0~50)
//		PID_WriteFlash(); //保存此地址flash
	}
	else if(NRFAddr!=TX_ADDRESS[TX_ADR_WIDTH-1])
	{
		TX_ADDRESS[TX_ADR_WIDTH-1]=NRFAddr;
		RX_ADDRESS[TX_ADR_WIDTH-1]=NRFAddr;
		NRF24L01_Config();
//		printf("NRFAddr:%d\r\n",NRFAddr);
	}
}

//****************************************H 通信测试函数
// 14 NRF24L01通信测试函数
void NRF_Test(void) //测试专用！
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
