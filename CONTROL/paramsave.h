#ifndef PARAMSAVE_H
#define PARAMSAVE_H

#include "stm32f4xx.h"

void ParamsToTable(void);
void TableToParams(void);
void DefaultParams(void);
void ParamsClearAll(void);
void PID_ClearFlash(void);
void PID_WriteFlash(void);
void PID_ReadFlash(void);
void DefaultParams_WriteFlash(void);

#endif
