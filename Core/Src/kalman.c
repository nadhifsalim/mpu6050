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
    .Q_angle = 0.0001,
    .Q_bias = 0.003,
    .R_measure = 0.01,  
    .angle = 0.0,
    .bias = 0.0,
    .P = {{0.0, 0.0}, {0.0, 0.0}}
};

Kalman_t KalmanY = {
    .Q_angle = 0.0001,
    .Q_bias = 0.003,
    .R_measure = 0.01,
    .angle = 0.0,
    .bias = 0.0,
    .P = {{0.0, 0.0}, {0.0, 0.0}}
};

void Kalman_Init(Kalman_t *Kalman)
{
    Kalman->Q_angle = 0.0001;
    Kalman->Q_bias = 0.003;
    Kalman->R_measure = 0.01;
    Kalman->angle = 0.0;
    Kalman->bias = 0.0;
    Kalman->P[0][0] = 0.0;
    Kalman->P[0][1] = 0.0;
    Kalman->P[1][0] = 0.0;
    Kalman->P[1][1] = 0.0;
}

void Kalman_SetAngle(Kalman_t *Kalman, double angle)
{
    Kalman->angle = angle;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

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
    double dt = (double)(now - kalman_last_tick) / 1000.0;
    kalman_last_tick = now;

    /* Proteksi delta time pertama kali atau jika terdapat delay besar */
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.05;
    }

    /* Hitung sudut roll dan pitch dari data accelerometer */
    double acc_roll  = atan2((double)dev->accel_y, (double)dev->accel_z) * RAD_TO_DEG;
    double acc_pitch = atan2(-(double)dev->accel_x, sqrt((double)(dev->accel_y * dev->accel_y + dev->accel_z * dev->accel_z))) * RAD_TO_DEG;

    /* Inisialisasi awal agar filter langsung mengunci sudut saat startup */
    if (!dev->kalman_init) {
        Kalman_SetAngle(&KalmanX, acc_roll);
        Kalman_SetAngle(&KalmanY, acc_pitch);
        dev->roll_k  = acc_roll;
        dev->pitch_k = acc_pitch;
        dev->kalman_init = 1;
        return;
    }

    /* Proteksi transisi sudut Roll ketika melompati batas -180 / +180 derajat */
    if ((acc_roll < -90.0 && KalmanX.angle > 90.0) || (acc_roll > 90.0 && KalmanX.angle < -90.0)) {
        Kalman_SetAngle(&KalmanX, acc_roll);
        dev->roll_k = acc_roll;
    } else {
        dev->roll_k = Kalman_getAngle(&KalmanX, acc_roll, (double)dev->gyro_x, dt);
    }

    /* Update filter Kalman untuk sumbu Pitch */
    dev->pitch_k = Kalman_getAngle(&KalmanY, acc_pitch, (double)dev->gyro_y, dt);
}
