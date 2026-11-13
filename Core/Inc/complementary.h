/**
  ******************************************************************************
  * @file    complementary.h
  * @brief   Complementary Filter driver for MPU6050 (Roll, Pitch, and Yaw).
  *          Features adaptive accelerometer weighting and stabilized Z-axis yaw.
  ******************************************************************************
  */

#ifndef _COMPLEMENTARY_H
#define _COMPLEMENTARY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu6050.h"

/* Parameter Tuning Complementary Filter */
#define COMP_ALPHA              0.98f   /* Bobot filter gyro default (0.0f - 1.0f) */
#define COMP_ACCEL_TOLERANCE    0.20f   /* Toleransi deviasi gravitasi (1.0g +/- 0.2g) */
#define COMP_YAW_DEADBAND       0.50f   /* Deadband threshold laju gyro Z (deg/s) */
#define COMP_CALIB_SAMPLES      500     /* Jumlah sampel kalibrasi gyro statis */

#ifndef RAD_TO_DEG
#define RAD_TO_DEG              57.295779513082320876798154814105f
#endif

/**
  * @brief  Kalibrasi startup gyroscope saat sensor dalam posisi diam.
  *         Mengambil 500 sampel dengan delay 2 ms untuk menghitung offset sumbu X, Y, Z.
  * @param  dev: Pointer ke struct MPU6050_t
  * @retval HAL_StatusTypeDef (HAL_OK jika sukses, HAL_ERROR jika gagal)
  */
HAL_StatusTypeDef MPU6050_Calibrate_Gyro(MPU6050_t *dev);

/**
  * @brief  Update estimasi sudut orientasi (Roll, Pitch, Yaw) menggunakan Complementary Filter.
  *         - Mengurangi offset kalibrasi gyro sebelum integrasi.
  *         - Menerapkan Adaptive Alpha untuk menolak noise akselerasi linier pada Roll & Pitch.
  *         - Menerapkan deadband filter dan integrasi delta time (dt) dengan wrapping [-180, +180] deg untuk Yaw.
  * @param  dev: Pointer ke struct MPU6050_t
  */
void MPU6050_Update_Angle(MPU6050_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* _COMPLEMENTARY_H */
