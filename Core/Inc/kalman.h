/*
*************************
*                       *
* İbrahim Cahit Özdemir *
*                       *
*     October 2021      *
*                       *
*************************
*/

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "stm32f1xx.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

