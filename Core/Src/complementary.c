#include "complementary.h"
#include <math.h>

static uint32_t last_tick = 0;

void MPU6050_Update_Angle(MPU6050_t *dev)
{
    if (dev == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) / 1000.0f;
    last_tick = now;

    /* Proteksi delta time pertama kali atau jika ada delay besar */
    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.05f;
    }

    /* Sudut dari accelerometer */
    float acc_roll  = atan2f(dev->accel_y, dev->accel_z) * (float)RAD_TO_DEG;
    float acc_pitch = atan2f(-dev->accel_x, sqrtf(dev->accel_y * dev->accel_y + dev->accel_z * dev->accel_z)) * (float)RAD_TO_DEG;

    if (!dev->angle_init) {
        dev->roll  = acc_roll;
        dev->pitch = acc_pitch;
        dev->angle_init = 1;
        return;
    }

    /* Complementary filter: gyro (jangka pendek) + accel (jangka panjang) */
    dev->roll  = COMP_ALPHA * (dev->roll  + dev->gyro_x * dt) + (1.0f - COMP_ALPHA) * acc_roll;
    dev->pitch = COMP_ALPHA * (dev->pitch + dev->gyro_y * dt) + (1.0f - COMP_ALPHA) * acc_pitch;
}
