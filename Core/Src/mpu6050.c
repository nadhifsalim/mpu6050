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

    /* Konversi ke satuan fisik */
    mpu->accel_x     = (float)mpu->accel_x_raw / MPU6050_ACCEL_SENS_2G;
    mpu->accel_y     = (float)mpu->accel_y_raw / MPU6050_ACCEL_SENS_2G;
    mpu->accel_z     = (float)mpu->accel_z_raw / MPU6050_ACCEL_SENS_2G;

    mpu->temperature = ((float)mpu->temp_raw / 340.0f) + 36.53f;

    mpu->gyro_x      = (float)mpu->gyro_x_raw / MPU6050_GYRO_SENS_250;
    mpu->gyro_y      = (float)mpu->gyro_y_raw / MPU6050_GYRO_SENS_250;
    mpu->gyro_z      = (float)mpu->gyro_z_raw / MPU6050_GYRO_SENS_250;

    return HAL_OK;
}




