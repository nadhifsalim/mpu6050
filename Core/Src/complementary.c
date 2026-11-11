#include "complementary.h"

void MPU6050_Update_Angle(MPU6050_t *dev)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - dev->last_tick) / 1000.0f;
    dev->last_tick = now;

    /* Sudut dari accelerometer */
    float acc_roll  = atan2f(dev->ay, dev->az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-dev->ax, sqrtf(dev->ay * dev->ay + dev->az * dev->az)) * RAD_TO_DEG;

    if (!dev->angle_init) {
        dev->roll  = acc_roll;
        dev->pitch = acc_pitch;
        dev->angle_init = 1;
        return;
    }

    /* Complementary filter: gyro (jangka pendek) + accel (jangka panjang) */
    dev->roll  = COMP_ALPHA * (dev->roll  + dev->gx * dt) + (1.0f - COMP_ALPHA) * acc_roll;
    dev->pitch = COMP_ALPHA * (dev->pitch + dev->gy * dt) + (1.0f - COMP_ALPHA) * acc_pitch;
}