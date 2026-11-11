#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdint.h>
#include "stm32f1xx.h"
#include "mpu6050.h"

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

typedef struct {
    double Q_angle;     /* Varians proses untuk akselerometer */
    double Q_bias;      /* Varians proses untuk bias giroskop */
    double R_measure;   /* Varians noise pengukuran */
    double angle;       /* Estimasi sudut (°/s) */
    double bias;        /* Estimasi bias giroskop (°/s) */
    double P[2][2];     /* Matriks kovariansi error */
} Kalman_t;

extern Kalman_t KalmanX;
extern Kalman_t KalmanY;

void Kalman_Init(Kalman_t *Kalman);
void Kalman_SetAngle(Kalman_t *Kalman, double angle);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

/**
 * @brief Update estimasi orientasi (roll & pitch) menggunakan Kalman filter
 * @param dev Pointer ke struct MPU6050_t
 */
void MPU6050_Update_Kalman(MPU6050_t *dev);

#endif /* INC_KALMAN_H_ */
