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

    mpu->gyro_offset_x  = 0.0f;
    mpu->gyro_offset_y  = 0.0f;
    mpu->gyro_offset_z  = 0.0f;

    mpu->angle_init     = 0;
    mpu->kalman_init    = 0;
    mpu->roll           = 0.0f;
    mpu->pitch          = 0.0f;
    mpu->yaw            = 0.0f;
    mpu->roll_k         = 0.0f;
    mpu->pitch_k        = 0.0f;
    mpu->yaw_k          = 0.0f;

    /* Periksa register WHO_AM_I (0x75) */
    uint8_t who_am_i = 0;
    if (HAL_I2C_Mem_Read(mpu->hi2c, mpu->dev_addr, MPU6050_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VAL) {
        return HAL_ERROR; /* Sensor tidak cocok atau tidak terdeteksi */
    }

    /* Reset sensor untuk mengembalikan semua register ke kondisi default */
    uint8_t reg_val = MPU6050_RESET;
    if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->dev_addr, MPU6050_REG_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &reg_val, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(50);

    /* Bangunkan sensor dan atur clock source ke PLL gyro sumbu X (lebih stabil dari RC internal) */
    reg_val = MPU6050_CLK_PLL_XGYRO;
    if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->dev_addr, MPU6050_REG_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &reg_val, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Atur Digital Low Pass Filter (DLPF) ~42Hz untuk meredam noise getaran mekanik */
    reg_val = MPU6050_DLPF_42HZ;
    if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->dev_addr, MPU6050_REG_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &reg_val, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Atur Sample Rate Divider (SMPLRT_DIV = 0 -> output rate = 1kHz) */
    reg_val = 0x00;
    if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->dev_addr, MPU6050_REG_SMPLRT_DIV,
                          I2C_MEMADD_SIZE_8BIT, &reg_val, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Atur range akselerometer ke ±2g (ACCEL_CONFIG 0x1C) */
    reg_val = MPU6050_ACCEL_FS_2G;
    if (HAL_I2C_Mem_Write(mpu->hi2c, mpu->dev_addr, MPU6050_REG_ACCEL_CONFIG,
                          I2C_MEMADD_SIZE_8BIT, &reg_val, 1, MPU6050_I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Atur range gyroscope ke ±250 °/s (GYRO_CONFIG 0x1B) */
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

    /* Konversi ke satuan fisik (scaled) */
    mpu->accel_x     = (float)mpu->accel_x_raw / MPU6050_ACCEL_SENS_2G;
    mpu->accel_y     = (float)mpu->accel_y_raw / MPU6050_ACCEL_SENS_2G;
    mpu->accel_z     = (float)mpu->accel_z_raw / MPU6050_ACCEL_SENS_2G;

    mpu->temperature = ((float)mpu->temp_raw / 340.0f) + 36.53f;

    mpu->gyro_x      = (float)mpu->gyro_x_raw / MPU6050_GYRO_SENS_250;
    mpu->gyro_y      = (float)mpu->gyro_y_raw / MPU6050_GYRO_SENS_250;
    mpu->gyro_z      = (float)mpu->gyro_z_raw / MPU6050_GYRO_SENS_250;

    return HAL_OK;
}

