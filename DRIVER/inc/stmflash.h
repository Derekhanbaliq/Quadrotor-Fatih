#ifndef STMFLASH_H
#define STMFLASH_H

#include "stm32f4xx.h"

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000

//FLASH��������ʼ��ַ
//#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//����0��ʼ��ַ, 8 Kbytes	//CCU6�ĳ���
//#define ADDR_FLASH_SECTOR_1     ((u32)0x08002000) 	//����1��ʼ��ַ, 8 Kbytes  
//#define ADDR_FLASH_SECTOR_2     ((u32)0x08004000) 	//����2��ʼ��ַ, 8 Kbytes  
//#define ADDR_FLASH_SECTOR_3     ((u32)0x08006000) 	//����3��ʼ��ַ, 8 Kbytes  
//#define ADDR_FLASH_SECTOR_4     ((u32)0x08008000) 	//����4��ʼ��ַ, 32 Kbytes  
//#define ADDR_FLASH_SECTOR_5     ((u32)0x08010000) 	//����5��ʼ��ַ, 64 Kbytes  
//#define ADDR_FLASH_SECTOR_6     ((u32)0x08020000) 	//����6��ʼ��ַ, 64 Kbytes  
//#define ADDR_FLASH_SECTOR_7     ((u32)0x08030000) 	//����7��ʼ��ַ, 64 Kbytes  

#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//??0????, 16 Kbytes	//CEU6�ĳ���
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//??1????, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//??2????, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//??3????, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//??4????, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//??5????, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//??6????, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//??7????, 128 Kbytes  

//#define FLASH_SAVE_ADDR  0X08030000 //����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.
//									//����д������ʱ����ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.
#define FLASH_SAVE_ADDR  0X08060000

u32 STMFLASH_ReadWord(u32 faddr);	//������  
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����
						   
#endif
