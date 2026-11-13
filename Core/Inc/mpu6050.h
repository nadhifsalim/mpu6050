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

/* Konfigurasi Clock Source & Power Management (Register PWR_MGMT_1 0x6B) */
#define MPU6050_RESET               0x80    /* Device Reset bit */
#define MPU6050_CLK_INTERNAL        0x00    /* Internal 8MHz oscillator */
#define MPU6050_CLK_PLL_XGYRO       0x01    /* PLL dengan referensi gyro sumbu X (direkomendasikan) */
#define MPU6050_CLK_PLL_YGYRO       0x02    /* PLL dengan referensi gyro sumbu Y */
#define MPU6050_CLK_PLL_ZGYRO       0x03    /* PLL dengan referensi gyro sumbu Z */

/* Konfigurasi Digital Low Pass Filter (Register CONFIG 0x1A, bits [2:0]) */
#define MPU6050_DLPF_260HZ          0x00    /* Accel 260Hz, Gyro 256Hz, Fs 8kHz */
#define MPU6050_DLPF_184HZ          0x01    /* Accel 184Hz, Gyro 188Hz, Fs 1kHz */
#define MPU6050_DLPF_94HZ           0x02    /* Accel 94Hz,  Gyro 98Hz,  Fs 1kHz */
#define MPU6050_DLPF_42HZ           0x03    /* Accel 44Hz,  Gyro 42Hz,  Fs 1kHz */
#define MPU6050_DLPF_20HZ           0x04    /* Accel 21Hz,  Gyro 20Hz,  Fs 1kHz */
#define MPU6050_DLPF_10HZ           0x05    /* Accel 10Hz,  Gyro 10Hz,  Fs 1kHz */
#define MPU6050_DLPF_5HZ            0x06    /* Accel 5Hz,   Gyro 5Hz,   Fs 1kHz */

/* Konfigurasi Accelerometer Full-Scale Range (Register ACCEL_CONFIG 0x1C, bits [4:3]) */
#define MPU6050_ACCEL_FS_2G         0x00    /* ±2g   (AFS_SEL = 0) */
#define MPU6050_ACCEL_FS_4G         0x08    /* ±4g   (AFS_SEL = 1) */
#define MPU6050_ACCEL_FS_8G         0x10    /* ±8g   (AFS_SEL = 2) */
#define MPU6050_ACCEL_FS_16G        0x18    /* ±16g  (AFS_SEL = 3) */

/* Konfigurasi Gyroscope Full-Scale Range (Register GYRO_CONFIG 0x1B, bits [4:3]) */
#define MPU6050_GYRO_FS_250         0x00    /* ±250 °/s  (FS_SEL = 0) */
#define MPU6050_GYRO_FS_500         0x08    /* ±500 °/s  (FS_SEL = 1) */
#define MPU6050_GYRO_FS_1000        0x10    /* ±1000 °/s (FS_SEL = 2) */
#define MPU6050_GYRO_FS_2000        0x18    /* ±2000 °/s (FS_SEL = 3) */

/* Faktor Sensitivitas Accelerometer (LSB/g) */
#define MPU6050_ACCEL_SENS_2G       16384.0f
#define MPU6050_ACCEL_SENS_4G       8192.0f
#define MPU6050_ACCEL_SENS_8G       4096.0f
#define MPU6050_ACCEL_SENS_16G      2048.0f

/* Faktor Sensitivitas Gyroscope (LSB/(°/s)) */
#define MPU6050_GYRO_SENS_250       131.0f
#define MPU6050_GYRO_SENS_500       65.5f
#define MPU6050_GYRO_SENS_1000      32.8f
#define MPU6050_GYRO_SENS_2000      16.4f

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

    /* Data hasil konversi satuan fisik (scaled) */
    float accel_x;              /* Akselerasi sumbu X (g) */
    float accel_y;              /* Akselerasi sumbu Y (g) */
    float accel_z;              /* Akselerasi sumbu Z (g) */
    float temperature;          /* Suhu internal (°C) */
    float gyro_x;               /* Kecepatan sudut sumbu X (°/s) */
    float gyro_y;               /* Kecepatan sudut sumbu Y (°/s) */
    float gyro_z;               /* Kecepatan sudut sumbu Z (°/s) */

    /* Offset kalibrasi gyroscope */
    float gyro_offset_x;        /* Offset kecepatan sudut sumbu X (°/s) */
    float gyro_offset_y;        /* Offset kecepatan sudut sumbu Y (°/s) */
    float gyro_offset_z;        /* Offset kecepatan sudut sumbu Z (°/s) */

    uint8_t angle_init;
    uint8_t kalman_init;

    float roll_k;               /* Estimasi sudut Roll hasil Kalman Filter (°) */
    float pitch_k;              /* Estimasi sudut Pitch hasil Kalman Filter (°) */
    float yaw_k;                /* Estimasi sudut Yaw hasil Kalman Filter (°) */

    float roll;                 /* Estimasi sudut Roll hasil Complementary Filter (°) */
    float pitch;                /* Estimasi sudut Pitch hasil Complementary Filter (°) */
    float yaw;                  /* Estimasi sudut Yaw hasil Complementary Filter (°) */
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
  *         dan mengonversinya ke satuan fisik.
  * @param  mpu: Pointer ke struct MPU6050_t
  * @retval HAL status (HAL_OK jika sukses)
  */
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_t *mpu);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
