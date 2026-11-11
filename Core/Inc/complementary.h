#ifndef _COMPLEMENTARY_H
#define _COMPLEMENTARY_H

#include "mpu6050.h"

#define COMP_ALPHA 0.98f
#define RAD_TO_DEG 57.295779513082320876798154814105

void MPU6050_Update_Angle(MPU6050_t *dev);

#endif
