#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"

void USART1_Init(u32 Baudrate);
void USART2_Init(u32 Baudrate);
void USART_Send(u8 *data, u8 length);

#endif
