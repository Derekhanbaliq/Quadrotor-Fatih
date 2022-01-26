/*******************************************************************************
项目名称：Expeditioner-I: Fatih
程序作者：Mingyan Zhou/Derek Zhou/周茗岩
编写日期：2020/09/11
程序功能：参数保存
*******************************************************************************/
#include "paramsave.h"
#include "structconfig.h"
#include "stmflash.h"
#include "stdio.h"

#define INT160(dwTemp)       (*(int16_t*)(&dwTemp))
#define INT161(dwTemp)       (*((int16_t *)(&dwTemp) + 1))
#define INT162(dwTemp)       (*((int16_t *)(&dwTemp) + 2))
#define INT163(dwTemp)       (*((int16_t *)(&dwTemp) + 3))

PID_SAVE PIDflash;

// 1 重要参数赋值给Flash的存储结构体
void ParamsToTable(void) //由于Flash的存储的结构体为u16，但是PID参数为float保留三位精度
{						 //因此PID参数都乘以1000然后强转为u16
	//零偏数据
	PIDflash.ACC_OFFSET_X = ACC_OFFSET_RAW.X;
	PIDflash.ACC_OFFSET_Y = ACC_OFFSET_RAW.Y;
	PIDflash.ACC_OFFSET_Z = ACC_OFFSET_RAW.Z; 
	PIDflash.GYRO_OFFSET_X = GYRO_OFFSET_RAW.X;
	PIDflash.GYRO_OFFSET_Y = GYRO_OFFSET_RAW.Y;
	PIDflash.GYRO_OFFSET_Z = GYRO_OFFSET_RAW.Z;
	
	//角度环数据PID参数
	PIDflash.ROL_Angle_P = (u16)(PID_ROL_Angle.P*1000); //PIDflash里的参数由control.c中的PID_TYPE的参数定义，已经经过严格计算了
	PIDflash.ROL_Angle_I = (u16)(PID_ROL_Angle.I*1000);
	PIDflash.ROL_Angle_D = (u16)(PID_ROL_Angle.D*1000);
	PIDflash.PIT_Angle_P = (u16)(PID_PIT_Angle.P*1000);
	PIDflash.PIT_Angle_I = (u16)(PID_PIT_Angle.I*1000);
	PIDflash.PIT_Angle_D = (u16)(PID_PIT_Angle.D*1000);
	PIDflash.YAW_Angle_P = (u16)(PID_YAW_Angle.P*1000);
	PIDflash.YAW_Angle_I = (u16)(PID_YAW_Angle.I*1000);
	PIDflash.YAW_Angle_D = (u16)(PID_YAW_Angle.D*1000);
	
	//角速度环PID参数
	PIDflash.ROL_Rate_P  = (u16)(PID_ROL_Rate.P*1000);
	PIDflash.ROL_Rate_I  = (u16)(PID_ROL_Rate.I*1000);
	PIDflash.ROL_Rate_D  = (u16)(PID_ROL_Rate.D*1000);
	PIDflash.PIT_Rate_P  = (u16)(PID_PIT_Rate.P*1000);
	PIDflash.PIT_Rate_I  = (u16)(PID_PIT_Rate.I*1000);
	PIDflash.PIT_Rate_D  = (u16)(PID_PIT_Rate.D*1000);
	PIDflash.YAW_Rate_P  = (u16)(PID_YAW_Rate.P*1000);
	PIDflash.YAW_Rate_I  = (u16)(PID_YAW_Rate.I*1000);
	PIDflash.YAW_Rate_D  = (u16)(PID_YAW_Rate.D*1000);
	
	//随机获取的NRF地址最低字节
	PIDflash.NRFaddr = NRFAddr;
}

// 2 从Flash中读出的参数赋值给全局变量
void TableToParams(void)
{
	//零偏数据
	ACC_OFFSET_RAW.X = PIDflash.ACC_OFFSET_X;
	ACC_OFFSET_RAW.Y = PIDflash.ACC_OFFSET_Y;
	ACC_OFFSET_RAW.Z = PIDflash.ACC_OFFSET_Z;
	GYRO_OFFSET_RAW.X = PIDflash.GYRO_OFFSET_X;
	GYRO_OFFSET_RAW.Y = PIDflash.GYRO_OFFSET_Y;
	GYRO_OFFSET_RAW.Z = PIDflash.GYRO_OFFSET_Z;
	
	//角度环数据PID参数
	PID_ROL_Angle.P = PIDflash.ROL_Angle_P/1000.0f;
	PID_ROL_Angle.I = PIDflash.ROL_Angle_I/1000.0f;
	PID_ROL_Angle.D = PIDflash.ROL_Angle_D/1000.0f;
	PID_PIT_Angle.P = PIDflash.PIT_Angle_P/1000.0f;
	PID_PIT_Angle.I = PIDflash.PIT_Angle_I/1000.0f;
	PID_PIT_Angle.D = PIDflash.PIT_Angle_D/1000.0f;
	PID_YAW_Angle.P = PIDflash.YAW_Angle_P/1000.0f;
	PID_YAW_Angle.I = PIDflash.YAW_Angle_I/1000.0f;
	PID_YAW_Angle.D = PIDflash.YAW_Angle_D/1000.0f;
		
	//角速度环PID参数	
	PID_ROL_Rate.P = PIDflash.ROL_Rate_P/1000.0f;
	PID_ROL_Rate.I = PIDflash.ROL_Rate_I/1000.0f;
	PID_ROL_Rate.D = PIDflash.ROL_Rate_D/1000.0f;
	PID_PIT_Rate.P = PIDflash.PIT_Rate_P/1000.0f;
	PID_PIT_Rate.I = PIDflash.PIT_Rate_I/1000.0f;
	PID_PIT_Rate.D = PIDflash.PIT_Rate_D/1000.0f;
	PID_YAW_Rate.P = PIDflash.YAW_Rate_P/1000.0f;
	PID_YAW_Rate.I = PIDflash.YAW_Rate_I/1000.0f;
	PID_YAW_Rate.D = PIDflash.YAW_Rate_D/1000.0f;
	
	//随机获取的NRF地址最低字节
	NRFAddr = PIDflash.NRFaddr;
}

// 3 默认参数
void DefaultParams(void) //若误操作将Flash中的数据都擦除了，可用此函数重新初始化或写入flash
{          
	//零偏数据
	PIDflash.ACC_OFFSET_X  = 0;
	PIDflash.ACC_OFFSET_Y  = 0;
	PIDflash.ACC_OFFSET_Z  = 0;
	PIDflash.GYRO_OFFSET_X = 0;
	PIDflash.GYRO_OFFSET_Y = 0;
	PIDflash.GYRO_OFFSET_Z = 0;
	
	//角度环PID参数
	PIDflash.ROL_Angle_P = 1800; //1800
	PIDflash.ROL_Angle_I = 10;	 //8
	PIDflash.ROL_Angle_D = 50;	 //50
	PIDflash.PIT_Angle_P = 1800; //1800
	PIDflash.PIT_Angle_I = 10;	 //8
	PIDflash.PIT_Angle_D = 50; 	 //50
	PIDflash.YAW_Angle_P = 3500; //3500
	PIDflash.YAW_Angle_I = 50;	 //0
	PIDflash.YAW_Angle_D = 1000; //1000
	
	//角速度环PID参数
	PIDflash.ROL_Rate_P  = 1000;  //930
	PIDflash.ROL_Rate_I  = 15;	 //5
	PIDflash.ROL_Rate_D  = 900;	 //860
	PIDflash.PIT_Rate_P  = 1000;  //930
	PIDflash.PIT_Rate_I  = 15;    //5
	PIDflash.PIT_Rate_D  = 900;  //860
	PIDflash.YAW_Rate_P  = 2000; //2000
	PIDflash.YAW_Rate_I  = 50;   //50
	PIDflash.YAW_Rate_D  = 1000; //1000
}

// 4 从flash存储结构的数据全部清零
void ParamsClearAll(void)
{
	//零偏数据
	PIDflash.ACC_OFFSET_X = 0;
	PIDflash.ACC_OFFSET_Y = 0;
	PIDflash.ACC_OFFSET_Z = 0;
	PIDflash.GYRO_OFFSET_X = 0;
	PIDflash.GYRO_OFFSET_Y = 0;
	PIDflash.GYRO_OFFSET_Z = 0;
	
	//角度环数据PID参数
	PIDflash.ROL_Angle_P = 0;
	PIDflash.ROL_Angle_I = 0;
	PIDflash.ROL_Angle_D = 0;
	PIDflash.PIT_Angle_P = 0;
	PIDflash.PIT_Angle_I = 0;
	PIDflash.PIT_Angle_D = 0;
	PIDflash.YAW_Angle_P = 0;
	PIDflash.YAW_Angle_I = 0;
	PIDflash.YAW_Angle_D = 0;

	//角速度环PID参数	
	PIDflash.ROL_Rate_P = 0;
	PIDflash.ROL_Rate_I = 0; 
	PIDflash.ROL_Rate_D = 0;
	PIDflash.PIT_Rate_P = 0; 
	PIDflash.PIT_Rate_I = 0;
	PIDflash.PIT_Rate_D = 0; 
	PIDflash.YAW_Rate_P = 0;
	PIDflash.YAW_Rate_I = 0; 
	PIDflash.YAW_Rate_D = 0; 
	
	//随机获取的NRF地址最低字节
	PIDflash.NRFaddr = 0;
}

// 5 将flash中的存储参数单元数据都清零
void PID_ClearFlash(void)
{
	u8 size,len;
	ParamsClearAll(); //数据清除
	len = sizeof(PIDflash);
	size = len/(4+(len%4)?1:0); //保存的数据长度
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
}

// 6 将重要的参数写入flash
void PID_WriteFlash(void)
{
	u8 size,len;
	ParamsToTable(); //浮点数转换成整数
	len = sizeof(PIDflash);
	size = len/(4+(len%4)?1:0); //保存的数据长度
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
}

// 7 从flash读取参数
void PID_ReadFlash(void)
{
	u8 size,len;
	len =  sizeof(PIDflash);
	size = len/(4+(len%4)?1:0);//保存的数据长度
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
	TableToParams();//将整数转换成浮点数
	//读取参数失败
	if(PIDflash.PIT_Rate_P ==0xFFFF && PIDflash.ROL_Rate_P==0xFFFF && PIDflash.YAW_Rate_P==0xFFFF)
	{
		DefaultParams_WriteFlash();
//		printf("Flash Read Error!!!\r\n");
	}
	else
	{
//		printf("Flash Read OK!!!\r\n");
	}
	/*
	printf("\r***************PID Angle loop parameter****************\r\n");
	printf("\rPID_ROL_Angle.P = %0.2f\r\n",PID_ROL_Angle.P);
	printf("\rPID_ROL_Angle.I = %0.2f\r\n",PID_ROL_Angle.I);
	printf("\rPID_ROL_Angle.D = %0.2f\r\n",PID_ROL_Angle.D);
	printf("\rPID_PIT_Angle.P = %0.2f\r\n",PID_PIT_Angle.P);
	printf("\rPID_PIT_Angle.I = %0.2f\r\n",PID_PIT_Angle.I);
	printf("\rPID_PIT_Angle.D = %0.2f\r\n",PID_PIT_Angle.D);
	printf("\rPID_YAW_Angle.P = %0.2f\r\n",PID_YAW_Angle.P);
	printf("\rPID_YAW_Angle.I = %0.2f\r\n",PID_YAW_Angle.I);
	printf("\rPID_YAW_Angle.D = %0.2f\r\n",PID_YAW_Angle.D);
	printf("\r***************PID AngleRate loop parameter**************\r\n");
	printf("\rPID_ROL_Rate.P = %0.2f\r\n",PID_ROL_Rate.P );
	printf("\rPID_ROL_Rate.I = %0.2f\r\n",PID_ROL_Rate.I );
	printf("\rPID_ROL_Rate.D = %0.2f\r\n",PID_ROL_Rate.D );
	printf("\rPID_PIT_Rate.P = %0.2f\r\n",PID_PIT_Rate.P );
	printf("\rPID_PIT_Rate.I = %0.2f\r\n",PID_PIT_Rate.I );
	printf("\rPID_PIT_Rate.D = %0.2f\r\n",PID_PIT_Rate.D );
	printf("\rPID_YAW_Rate.P = %0.2f\r\n",PID_YAW_Rate.P );
	printf("\rPID_YAW_Rate.I = %0.2f\r\n",PID_YAW_Rate.I );
	printf("\rPID_YAW_Rate.D = %0.2f\r\n",PID_YAW_Rate.D );
//	printf("\rNRFaddr = 0x%x\r\n", NRFAddr);
	*/
}

// 8 将默认PID参数写入flash
void DefaultParams_WriteFlash(void)
{
	u8 size,len;
	DefaultParams();//初始化默认参数
	len =  sizeof(PIDflash);
	size = len/(4+(len%4)?1:0); //保存的数据长度
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)(&PIDflash),size);
}
