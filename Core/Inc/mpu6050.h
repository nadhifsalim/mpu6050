/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   Driver sensor MPU6050 berbasis STM32 HAL (I2C blocking mode).
  ******************************************************************************
  */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Ganti header HAL di bawah sesuai seri chip STM32 yang digunakan (misal: stm32f4xx_hal.h) */
#include "stm32f1xx_hal.h"

/* Alamat I2C MPU6050 (AD0 = LOW -> 0x68, digeser 1 bit untuk STM32 HAL) */
#define MPU6050_I2C_ADDR            (0x68 << 1)

/* Register MPU6050 */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

/* Nilai identitas sensor WHO_AM_I */
#define MPU6050_WHO_AM_I_VAL        0x68

/* Konfigurasi Range Default (Accel: ±2g, Gyro: ±250 deg/s) */
#define MPU6050_ACCEL_FS_2G         0x00
#define MPU6050_GYRO_FS_250         0x00

/* Faktor Sensitivitas (LSB) */
#define MPU6050_ACCEL_SENS_2G       16384.0f
#define MPU6050_GYRO_SENS_250       131.0f

/* Timeout komunikasi I2C (ms) */
#define MPU6050_I2C_TIMEOUT         100

/**
  * @brief Struct penyimpan instance dan data sensor MPU6050.
  */
typedef struct {
    I2C_HandleTypeDef *hi2c;    /* Handle I2C peripheral STM32 */
    uint8_t dev_addr;           /* Alamat I2C sensor (shifted) */

    /* Data mentah (raw ADC 16-bit) */
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t temp_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    /* Data hasil konversi satuan fisik (setelah dikurangi offset) */
    float accel_x;              /* Akselerasi sumbu X (g) */
    float accel_y;              /* Akselerasi sumbu Y (g) */
    float accel_z;              /* Akselerasi sumbu Z (g) */
    float temperature;          /* Suhu internal (°C) */
    float gyro_x;               /* Kecepatan sudut sumbu X (°/s) */
    float gyro_y;               /* Kecepatan sudut sumbu Y (°/s) */
    float gyro_z;               /* Kecepatan sudut sumbu Z (°/s) */

    /* Offset kalibrasi */
    float accel_offset_x;       /* Offset akselerasi sumbu X (g) */
    float accel_offset_y;       /* Offset akselerasi sumbu Y (g) */
    float accel_offset_z;       /* Offset akselerasi sumbu Z (g) */
    float gyro_offset_x;        /* Offset kecepatan sudut sumbu X (°/s) */
    float gyro_offset_y;        /* Offset kecepatan sudut sumbu Y (°/s) */
    float gyro_offset_z;        /* Offset kecepatan sudut sumbu Z (°/s) */
} MPU6050_t;

/**
  * @brief  Inisialisasi sensor MPU6050: cek WHO_AM_I, bangunkan sensor, dan atur range.
  * @param  mpu: Pointer ke struct MPU6050_t
  * @param  hi2c: Pointer ke handle I2C yang sudah terinisialisasi
  * @retval HAL status (HAL_OK jika sukses)
  */
HAL_StatusTypeDef MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *hi2c);

/**
  * @brief  Membaca seluruh data (accel, temperatur, gyro) dengan burst read 14 byte
  *         dan menerapkan offset kalibrasi pada data satuan fisik.
  * @param  mpu: Pointer ke struct MPU6050_t
  * @retval HAL status (HAL_OK jika sukses)
  */
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_t *mpu);

/**
  * @brief  Kalibrasi akselerometer dan gyroscope menggunakan metode static averaging.
  * @note   Sensor harus diletakkan dalam kondisi diam dan mendatar (horizontal, Z = +1.0g).
  * @param  mpu: Pointer ke struct MPU6050_t
  * @param  num_samples: Jumlah sampel data untuk rata-rata statis (misal: 500, 1000)
  * @retval HAL status (HAL_OK jika sukses)
  */
HAL_StatusTypeDef MPU6050_Calibrate(MPU6050_t *mpu, uint16_t num_samples);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
