#include "complementary.h"
#include <math.h>

static uint32_t last_tick = 0;

HAL_StatusTypeDef MPU6050_Calibrate_Gyro(MPU6050_t *dev)
{
    if (dev == NULL || dev->hi2c == NULL) {
        return HAL_ERROR;
    }

    /* Reset offset sebelum kalibrasi */
    dev->gyro_offset_x = 0.0f;
    dev->gyro_offset_y = 0.0f;
    dev->gyro_offset_z = 0.0f;

    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;

    for (uint16_t i = 0; i < COMP_CALIB_SAMPLES; i++) {
        if (MPU6050_ReadAll(dev) != HAL_OK) {
            return HAL_ERROR;
        }

        sum_gx += dev->gyro_x;
        sum_gy += dev->gyro_y;
        sum_gz += dev->gyro_z;

        HAL_Delay(2);
    }

    dev->gyro_offset_x = sum_gx / (float)COMP_CALIB_SAMPLES;
    dev->gyro_offset_y = sum_gy / (float)COMP_CALIB_SAMPLES;
    dev->gyro_offset_z = sum_gz / (float)COMP_CALIB_SAMPLES;

    return HAL_OK;
}

void MPU6050_Update_Angle(MPU6050_t *dev)
{
    if (dev == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_tick) / 1000.0f;
    last_tick = now;

    /* Raw bias-corrected gyro rates in MPU6050 sensor frame */
    float raw_gx = dev->gyro_x - dev->gyro_offset_x;
    float raw_gy = dev->gyro_y - dev->gyro_offset_y;
    float raw_gz = dev->gyro_z - dev->gyro_offset_z;

    /*
     * Map MPU6050 frame (+X forward, +Y left, +Z up)
     * -> Aerospace NED body frame (+X nose, +Y right wing, +Z down).
     *
     * NOTE: If your MPU6050 silkscreen +X points toward the TAIL instead of the NOSE,
     * negate ax, ay, gyro_rate_x, and gyro_rate_y (keep gyro_rate_z = -raw_gz).
     */
    float ax =  dev->accel_x;
    float ay = -dev->accel_y;
    float az =  dev->accel_z;

    float gyro_rate_x =  raw_gx;   /* +Roll  = right wing down */
    float gyro_rate_y = -raw_gy;   /* +Pitch = nose up         */
    float gyro_rate_z = -raw_gz;   /* +Yaw   = nose right (CW) */

    /* Reference roll and pitch from accelerometer in NED frame */
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    /* Lock initial angles on first call */
    if (!dev->angle_init) {
        dev->roll  = acc_roll;
        dev->pitch = acc_pitch;
        dev->yaw   = 0.0f;
        dev->angle_init = 1;
        return;
    }

    /* Protect against abnormal dt */
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.05f;
    }

    /* Adaptive alpha to reject linear acceleration */
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    float alpha = COMP_ALPHA;
    if (fabsf(accel_mag - 1.0f) > COMP_ACCEL_TOLERANCE) {
        alpha = 1.0f;
    }

    /* Complementary Filter for Roll */
    if ((acc_roll < -90.0f && dev->roll > 90.0f) || (acc_roll > 90.0f && dev->roll < -90.0f)) {
        dev->roll = acc_roll;
    } else {
        dev->roll = alpha * (dev->roll + gyro_rate_x * dt) + (1.0f - alpha) * acc_roll;
    }

    /* Complementary Filter for Pitch */
    dev->pitch = alpha * (dev->pitch + gyro_rate_y * dt) + (1.0f - alpha) * acc_pitch;

    /* Stabilized Z-Axis Yaw Tracking */
    if (fabsf(gyro_rate_z) < COMP_YAW_DEADBAND) {
        gyro_rate_z = 0.0f;
    }

    dev->yaw += gyro_rate_z * dt;

    while (dev->yaw > 180.0f) {
        dev->yaw -= 360.0f;
    }
    while (dev->yaw < -180.0f) {
        dev->yaw += 360.0f;
    }
}