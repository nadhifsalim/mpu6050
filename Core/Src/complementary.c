/**
  ******************************************************************************
  * @file    complementary.c
  * @brief   Implementasi Complementary Filter untuk sensor MPU6050 (Roll, Pitch, Yaw).
  ******************************************************************************
  */

#include "complementary.h"
#include <math.h>

static uint32_t last_tick = 0;

/**
  * @brief  Kalibrasi startup gyroscope saat sensor dalam posisi diam.
  *         Mengambil 500 sampel dengan delay 2 ms untuk menghitung offset sumbu X, Y, Z.
  * @param  dev: Pointer ke struct MPU6050_t
  * @retval HAL_StatusTypeDef (HAL_OK jika sukses, HAL_ERROR jika gagal)
  */
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

/**
  * @brief  Update estimasi sudut orientasi (Roll, Pitch, Yaw) menggunakan Complementary Filter.
  *         - Mengurangi offset kalibrasi gyro sebelum integrasi.
  *         - Menerapkan Adaptive Alpha untuk menolak noise akselerasi linier pada Roll & Pitch.
  *         - Menerapkan deadband filter dan integrasi delta time (dt) dengan wrapping [-180, +180] deg untuk Yaw.
  * @param  dev: Pointer ke struct MPU6050_t
  */
void MPU6050_Update_Angle(MPU6050_t *dev)
{
    if (dev == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_tick) / 1000.0f;
    last_tick = now;

    /* 1. Kurangi offset kalibrasi gyro dari pembacaan saat ini sebelum integrasi */
    float gyro_rate_x = dev->gyro_x - dev->gyro_offset_x;
    float gyro_rate_y = dev->gyro_y - dev->gyro_offset_y;
    float gyro_rate_z = dev->gyro_z - dev->gyro_offset_z;

    /* 2. Hitung sudut roll dan pitch referensi dari accelerometer */
    float acc_roll  = atan2f(dev->accel_y, dev->accel_z) * RAD_TO_DEG;
    float acc_pitch = atan2f(-dev->accel_x, sqrtf(dev->accel_y * dev->accel_y + dev->accel_z * dev->accel_z)) * RAD_TO_DEG;

    /* Inisialisasi awal pada pemanggilan pertama: kunci sudut langsung ke orientasi awal */
    if (!dev->angle_init) {
        dev->roll  = acc_roll;
        dev->pitch = acc_pitch;
        dev->yaw   = 0.0f;
        dev->angle_init = 1;
        return;
    }

    /* Proteksi delta time jika terjadi anomali delay atau overflow */
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.05f; /* Default fallback ke 20Hz (~50ms) */
    }

    /* 3. Adaptive Alpha untuk Roll & Pitch:
          Hitung magnitudo total vektor akselerometer: sqrt(ax^2 + ay^2 + az^2).
          Jika deviasi dari 1.0g melebihi 0.2g (terindikasi akselerasi linier / goncangan),
          set alpha = 1.0f untuk sementara waktu hanya percaya pada gyro.
          Jika stabil (sekitar 1.0g), gunakan nilai default COMP_ALPHA (0.98f). */
    float accel_mag = sqrtf(dev->accel_x * dev->accel_x +
                            dev->accel_y * dev->accel_y +
                            dev->accel_z * dev->accel_z);

    float alpha = COMP_ALPHA;
    if (fabsf(accel_mag - 1.0f) > COMP_ACCEL_TOLERANCE) {
        alpha = 1.0f;
    }

    /* Complementary Filter untuk Roll (dengan proteksi transisi batas -180 / +180 derajat) */
    if ((acc_roll < -90.0f && dev->roll > 90.0f) || (acc_roll > 90.0f && dev->roll < -90.0f)) {
        dev->roll = acc_roll;
    } else {
        dev->roll = alpha * (dev->roll + gyro_rate_x * dt) + (1.0f - alpha) * acc_roll;
    }

    /* Complementary Filter untuk Pitch */
    dev->pitch = alpha * (dev->pitch + gyro_rate_y * dt) + (1.0f - alpha) * acc_pitch;

    /* 4. Stabilized Z-Axis Yaw Tracking:
          - Terapkan deadband filter: jika laju gyro Z absolut < 0.5 deg/s,
            paksa ke 0.0f untuk memutus drift akumulasi akibat noise sensor saat diam.
          - Integrasikan laju Z terhadap delta time (dt).
          - Lakukan angle wrapping agar sudut Yaw selalu berada dalam batas -180.0° s/d +180.0°. */
    if (fabsf(gyro_rate_z) < COMP_YAW_DEADBAND) {
        gyro_rate_z = 0.0f;
    }

    dev->yaw += gyro_rate_z * dt;

    /* Wrapping batas sudut Yaw [-180.0, +180.0] derajat */
    while (dev->yaw > 180.0f) {
        dev->yaw -= 360.0f;
    }
    while (dev->yaw < -180.0f) {
        dev->yaw += 360.0f;
    }
}
