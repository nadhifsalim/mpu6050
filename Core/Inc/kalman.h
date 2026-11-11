#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdint.h>
#include "stm32f1xx.h"

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif /* INC_KALMAN_H_ */
