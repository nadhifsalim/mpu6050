#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdint.h>
#include "stm32f1xx.h"
#include "mpu6050.h"

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105f
#endif

#ifndef KALMAN_YAW_DEADBAND
#define KALMAN_YAW_DEADBAND 0.50f /* Deadband threshold laju gyro Z (deg/s) */
#endif

typedef struct {
    float Q_angle;      /* Varians proses untuk akselerometer */
    float Q_bias;       /* Varians proses untuk bias giroskop */
    float R_measure;    /* Varians noise pengukuran */
    float angle;        /* Estimasi sudut (°) */
    float bias;         /* Estimasi bias giroskop (°/s) */
    float P[2][2];      /* Matriks kovariansi error */
} Kalman_t;

extern Kalman_t KalmanX;
extern Kalman_t KalmanY;
extern Kalman_t KalmanZ;

void Kalman_Init(Kalman_t *Kalman);
void Kalman_SetAngle(Kalman_t *Kalman, float angle);
float Kalman_Predict(Kalman_t *Kalman, float newRate, float dt);
float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);

void MPU6050_Update_Kalman(MPU6050_t *dev);

#endif 
