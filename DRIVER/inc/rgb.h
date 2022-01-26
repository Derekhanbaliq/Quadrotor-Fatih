#ifndef RGB_H
#define RGB_H

#include "stm32f4xx.h"

void RGB_Init(void);

void RGB_Write0(void);
void RGB_Write1(void);
void RGB_Reset(void);
void RGB_Write_Byte(u8 data);
void RGB_Set_Color(u8 green, u8 red, u8 blue);

void RGB_Red(void);
void RGB_Green(void);
void RGB_Blue(void);
void RGB_Cyan(void);
void RGB_Yellow(void);
void RGB_Magnet(void);
void RGB_Orange(void);
void RGB_White(void);
void RGB_Rand(void);
void RGB_Off(void);

void RGB_Fly(void);
void RGB_GYRO_Calib(void);
void RGB_ACC_Calib(void);
void RGB_BARO_Calib(void);
void RGB_WIFI_Switch(void);
void RGB_BAT_Alarm(void);
void RGB_Unlock(uint8_t N, uint8_t flag);
void OneNET_LED(uint8_t color[], uint8_t num);

#endif
