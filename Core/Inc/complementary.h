#ifndef _COMPLEMENTARY_H
#define _COMPLEMENTARY_H

#include "mpu6050.h"

#define COMP_ALPHA 1.0f

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105f
#endif

void MPU6050_Update_Angle(MPU6050_t *dev);

#endif
