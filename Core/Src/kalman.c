/*
*************************
*                       *
* İbrahim Cahit Özdemir *
*                       *
*     October 2021      *
*                       *
*************************
*/

#include <math.h>
#include "kalman.h"

Kalman_t KalmanX = {
    .Q_angle = 0.0001f,
    .Q_bias = 0.003f,
    .R_measure = 0.01f,
    .angle = 0.0f,
    .bias = 0.0f,
    .P = {{0.0f, 0.0f}, {0.0f, 0.0f}}
};

Kalman_t KalmanY = {
    .Q_angle = 0.0001f,
    .Q_bias = 0.003f,
    .R_measure = 0.01f,
    .angle = 0.0f,
    .bias = 0.0f,
    .P = {{0.0f, 0.0f}, {0.0f, 0.0f}}
};

Kalman_t KalmanZ = {
    .Q_angle = 0.0001f,
    .Q_bias = 0.003f,
    .R_measure = 0.01f,
    .angle = 0.0f,
    .bias = 0.0f,
    .P = {{0.0f, 0.0f}, {0.0f, 0.0f}}
};

void Kalman_Init(Kalman_t *Kalman)
{
    Kalman->Q_angle = 0.0001f;
    Kalman->Q_bias = 0.003f;
    Kalman->R_measure = 0.01f;
    Kalman->angle = 0.0f;
    Kalman->bias = 0.0f;
    Kalman->P[0][0] = 0.0f;
    Kalman->P[0][1] = 0.0f;
    Kalman->P[1][0] = 0.0f;
    Kalman->P[1][1] = 0.0f;
}

void Kalman_SetAngle(Kalman_t *Kalman, float angle)
{
    Kalman->angle = angle;
}

float Kalman_Predict(Kalman_t *Kalman, float newRate, float dt)
{
    float rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    return Kalman->angle;
}

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt)
{
    Kalman_Predict(Kalman, newRate, dt);

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

static uint32_t kalman_last_tick = 0;

void MPU6050_Update_Kalman(MPU6050_t *dev)
{
    if (dev == NULL) {
        return;
    }

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - kalman_last_tick) / 1000.0f;
    kalman_last_tick = now;

    /* Proteksi delta time pertama kali atau jika terdapat delay besar */
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.05f;
    }

    /* Hitung sudut roll dan pitch dari data accelerometer */
    float acc_roll  = atan2f(dev->accel_y, dev->accel_z) * RAD_TO_DEG;
    float acc_pitch = atan2f(-dev->accel_x, sqrtf(dev->accel_y * dev->accel_y + dev->accel_z * dev->accel_z)) * RAD_TO_DEG;

    /* Inisialisasi awal agar filter langsung mengunci sudut saat startup */
    if (!dev->kalman_init) {
        Kalman_SetAngle(&KalmanX, acc_roll);
        Kalman_SetAngle(&KalmanY, acc_pitch);
        Kalman_SetAngle(&KalmanZ, 0.0f);
        dev->roll_k  = acc_roll;
        dev->pitch_k = acc_pitch;
        dev->yaw_k   = 0.0f;
        dev->kalman_init = 1;
        return;
    }

    /* Proteksi transisi sudut Roll ketika melompati batas -180 / +180 derajat */
    if ((acc_roll < -90.0f && KalmanX.angle > 90.0f) || (acc_roll > 90.0f && KalmanX.angle < -90.0f)) {
        Kalman_SetAngle(&KalmanX, acc_roll);
        dev->roll_k = acc_roll;
    } else {
        dev->roll_k = Kalman_getAngle(&KalmanX, acc_roll, dev->gyro_x - dev->gyro_offset_x, dt);
    }

    /* Update filter Kalman untuk sumbu Pitch */
    dev->pitch_k = Kalman_getAngle(&KalmanY, acc_pitch, dev->gyro_y - dev->gyro_offset_y, dt);

    /* Update filter Kalman untuk sumbu Yaw (prediksi gyro Z + deadband + wrapping [-180, +180]) */
    float gyro_rate_z = dev->gyro_z - dev->gyro_offset_z;
    if (fabsf(gyro_rate_z) < KALMAN_YAW_DEADBAND) {
        gyro_rate_z = 0.0f;
    }

    float yaw_k = Kalman_Predict(&KalmanZ, gyro_rate_z, dt);
    while (yaw_k > 180.0f) {
        yaw_k -= 360.0f;
    }
    while (yaw_k < -180.0f) {
        yaw_k += 360.0f;
    }
    Kalman_SetAngle(&KalmanZ, yaw_k);
    dev->yaw_k = yaw_k;
}
