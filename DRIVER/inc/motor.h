#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx.h"

void MOTOR_Init(void);
void Motor_Pwm(s16 MOTOR1_PWM, s16 MOTOR2_PWM, s16 MOTOR3_PWM, s16 MOTOR4_PWM);

#endif
