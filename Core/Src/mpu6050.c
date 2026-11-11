/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   Implementasi driver sensor MPU6050 berbasis STM32 HAL (I2C).
  ******************************************************************************
  */

#include "mpu6050.h"

HAL_StatusTypeDef MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c)
{
    if (mpu == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    mpu->hi2c = hi2c;
    mpu->dev_addr = MPU6050_I2C_ADDR;

    /* Inisialisasi nilai pembacaan dan offset ke 0 */
    mpu->accel_x_raw = 0;
    mpu->accel_y_raw = 0;
    mpu->accel_z_raw = 0;
    mpu->temp_raw    = 0;
    mpu->gyro_x_raw  = 0;
    mpu->gyro_y_raw  = 0;
    mpu->gyro_z_raw  = 0;

    mpu->accel_x     = 0.0f;
    mpu->accel_y     = 0.0f;
    mpu->accel_z     = 0.0f;
    mpu->temperature = 0.0f;
    mpu->gyro_x      = 0.0f;
    mpu->gyro_y      = 0.0f;
    mpu->gyro_z      = 0.0f;

    mpu->accel_offset_x = 0.0f;
    mpu->accel_offset_y = 0.0f;
    mpu->accel_offset_z = 0.0f;
    mpu->gyro_offset_x  = 0.0f;
    mpu->gyro_offset_y  = 0.0f;
    mpu->gyro_offset_z  = 0.0f;

    mpu->angle_init     = 0;
    mpu->kalman_init    = 0;
    mpu->roll           = 0.0f;
    mpu->pitch          = 0.0f;
    mpu->roll_k         = 0.0;
    mpu->pitch_k        = 0.0;

    /* 1. Periksa register WHO_AM_I (0x75) */
    uint8_t who_am_i = 0;
    if (HAL_I2C_Mem_Read(mpu->hi2c, mpu->dev_addr, MPU6050_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VAL) {
        return HAL_ERROR; /* Sensor tidak cocok atau tidak terdeteksi */
    }

    /* 2. Bangunkan sensor dengan menulis 0x00 ke PWR_MGMT_1 (0x6B) */
    uint8_t reg_val = 0x00;
    if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->dev_addr, MPU6050_REG_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &reg_val, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 3. Atur range akselerometer ke ±2g (ACCEL_CONFIG 0x1C) */
    reg_val = MPU6050_ACCEL_FS_2G;
    if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->dev_addr, MPU6050_REG_ACCEL_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &reg_val, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 4. Atur range gyroscope ke ±250 °/s (GYRO_CONFIG 0x1B) */
    reg_val = MPU6050_GYRO_FS_250;
    if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->dev_addr, MPU6050_REG_GYRO_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &reg_val, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_t *mpu)
{
    if (mpu == NULL || mpu->hi2c == NULL) {
        return HAL_ERROR;
    }

    /* Burst read 14 byte mulai dari ACCEL_XOUT_H (0x3B) */
    uint8_t buffer[14];
    if (HAL_I2C_Mem_Read(mpu->hi2c, mpu->dev_addr, MPU6050_REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, buffer, 14, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Gabungkan data mentah 16-bit (Big Endian) */
    mpu->accel_x_raw = (int16_t)((buffer[0]  << 8) | buffer[1]);
    mpu->accel_y_raw = (int16_t)((buffer[2]  << 8) | buffer[3]);
    mpu->accel_z_raw = (int16_t)((buffer[4]  << 8) | buffer[5]);
    mpu->temp_raw    = (int16_t)((buffer[6]  << 8) | buffer[7]);
    mpu->gyro_x_raw  = (int16_t)((buffer[8]  << 8) | buffer[9]);
    mpu->gyro_y_raw  = (int16_t)((buffer[10] << 8) | buffer[11]);
    mpu->gyro_z_raw  = (int16_t)((buffer[12] << 8) | buffer[13]);

    /* Konversi ke satuan fisik dan kurangi offset kalibrasi */
    mpu->accel_x     = ((float)mpu->accel_x_raw / MPU6050_ACCEL_SENS_2G) - mpu->accel_offset_x;
    mpu->accel_y     = ((float)mpu->accel_y_raw / MPU6050_ACCEL_SENS_2G) - mpu->accel_offset_y;
    mpu->accel_z     = ((float)mpu->accel_z_raw / MPU6050_ACCEL_SENS_2G) - mpu->accel_offset_z;

    mpu->temperature = ((float)mpu->temp_raw / 340.0f) + 36.53f;

    mpu->gyro_x      = ((float)mpu->gyro_x_raw / MPU6050_GYRO_SENS_250) - mpu->gyro_offset_x;
    mpu->gyro_y      = ((float)mpu->gyro_y_raw / MPU6050_GYRO_SENS_250) - mpu->gyro_offset_y;
    mpu->gyro_z      = ((float)mpu->gyro_z_raw / MPU6050_GYRO_SENS_250) - mpu->gyro_offset_z;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Calibrate(MPU6050_t *mpu, uint16_t num_samples)
{
    if (mpu == NULL || mpu->hi2c == NULL || num_samples == 0) {
        return HAL_ERROR;
    }

    double sum_ax = 0.0;
    double sum_ay = 0.0;
    double sum_az = 0.0;
    double sum_gx = 0.0;
    double sum_gy = 0.0;
    double sum_gz = 0.0;

    for (uint16_t i = 0; i < num_samples; i++) {
        if (MPU6050_ReadAll(mpu) != HAL_OK) {
            return HAL_ERROR;
        }

        /* Akumulasikan data fisik mentah sebelum offset kalibrasi */
        sum_ax += (double)mpu->accel_x_raw / MPU6050_ACCEL_SENS_2G;
        sum_ay += (double)mpu->accel_y_raw / MPU6050_ACCEL_SENS_2G;
        sum_az += (double)mpu->accel_z_raw / MPU6050_ACCEL_SENS_2G;

        sum_gx += (double)mpu->gyro_x_raw / MPU6050_GYRO_SENS_250;
        sum_gy += (double)mpu->gyro_y_raw / MPU6050_GYRO_SENS_250;
        sum_gz += (double)mpu->gyro_z_raw / MPU6050_GYRO_SENS_250;

        HAL_Delay(2);
    }

    /* Hitung rata-rata statis (static averaging) */
    mpu->accel_offset_x = (float)(sum_ax / (double)num_samples);
    mpu->accel_offset_y = (float)(sum_ay / (double)num_samples);
    /* Sumbu Z mengukur gravitasi bumi (+1.0g) pada posisi datar diam menghadap atas */
    mpu->accel_offset_z = (float)((sum_az / (double)num_samples) - 1.0);

    mpu->gyro_offset_x  = (float)(sum_gx / (double)num_samples);
    mpu->gyro_offset_y  = (float)(sum_gy / (double)num_samples);
    mpu->gyro_offset_z  = (float)(sum_gz / (double)num_samples);

    return HAL_OK;
}

