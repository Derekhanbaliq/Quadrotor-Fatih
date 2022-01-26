/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/09/03
程序功能：STMFLASH驱动
备    注：STM32F411CCU6，FLASH=256Kbytes
*******************************************************************************/
#include "stmflash.h"
#include "delay.h"
#include "usart.h"

// 1 读取指定地址的字(32位数据)
u32 STMFLASH_ReadWord(u32 faddr) //faddr:读地址
{
	return *(vu32*)faddr; //返回对应数据
}  

// 2 获取某个地址所在的flash扇区
u16 STMFLASH_GetFlashSector(u32 addr) //addr: flash地址
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	return FLASH_Sector_7;	
}

// 3 从指定地址开始写入指定长度的数据		备注：STM32F4的Flash未写扇区默认是0xFFF...F,调用此函数时需要
void STMFLASH_Write(u32 WriteAddr, u32 *pBuffer, u32 NumToWrite) //WriteAddr:起始地址(此地址必须为4的倍数!!) pBuffer:数据指针 NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
{ 
	FLASH_Status status=FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;
	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4) return;	//非法地址
	FLASH_Unlock();	//解锁 
	FLASH_DataCacheCmd(DISABLE); //FLASH擦除期间,必须禁止数据缓存
	addrx=WriteAddr; 				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4; //写入的结束地址
	if(addrx<0X1FFF0000) //只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr) //扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF) //有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3); //VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE) break; //发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr) //写数据
		{
			if(FLASH_ProgramWord(WriteAddr, *pBuffer)!=FLASH_COMPLETE) //写入数据
			{ 
				break; //写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock(); //上锁
} 

// 4 从指定地址开始读出指定长度的数据
void STMFLASH_Read(u32 ReadAddr, u32 *pBuffer, u32 NumToRead) //ReadAddr:起始地址 pBuffer:数据指针 NumToRead:字(4位)数
{
	u32 i;
	for(i=0; i<NumToRead; i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr); //读取4个字节
		ReadAddr+=4; //偏移4个字节
	}
}
