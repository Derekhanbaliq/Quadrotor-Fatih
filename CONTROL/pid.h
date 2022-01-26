#ifndef PID_H
#define PID_H

#include "stm32f4xx.h"
#include "structconfig.h"

void PidParameter_init(void);
void PID_Postion_Cal(PID_TYPE *PID, float target, float measure);

#endif
