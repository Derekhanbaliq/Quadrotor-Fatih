#ifndef POWER_H
#define POWER_H

#include "stm32f4xx.h"

void BAT_Init(void);
u16 BAT_GetAdc(u8 ch);
void BAT_GetVoltage(void);
void BAT_LowVoltageAlarm(void);
void WiFi_Switch(u8 flag);
void NRF24L01_Switch(u8 flag);

#endif
