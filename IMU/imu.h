#ifndef IMU_H
#define IMU_H

#include "stm32f4xx.h"
#include "structconfig.h"

void Prepare_Data(void);
void IMUupdate(FLOAT_XYZ *Gyr_rad, FLOAT_XYZ *Acc_filt, FLOAT_ANGLE *Att_Angle);

#endif
