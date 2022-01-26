#ifndef DELAY_H
#define DELAY_H

#include "stm32f4xx.h"

void delay_Init(void);
void delay_ms(u32 nTime);
void delay_us(u32 nTime);
void delay(u32 timers);

#endif
