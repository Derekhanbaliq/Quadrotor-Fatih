#ifndef FILTER_H
#define FILTER_H

#include "stm32f4xx.h"
#include "structconfig.h"

void SortAver_Filter(float value, float *filter, u8 n);
void SortAver_FilterXYZ(INT16_XYZ *acc, FLOAT_XYZ *Acc_filt, u8 n);
void Aver_Filter(float data, float *filt_data, u8 n);
void Aver_FilterXYZ(INT16_XYZ *acc, FLOAT_XYZ *Acc_filt, u8 n);

#endif
