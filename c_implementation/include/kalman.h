#ifndef KALMAN_H
#define KALMAN_H

#include "param.h"

typedef struct {
    double U_kalman[4];
    double X_hat[15];
    double A_u;
    double B_u;
} KalmanState;

void kalman_init(KalmanState* state, Param* param);
void kalman_outputs(KalmanState* state, double* X_hat);
void kalman_update(double* u, double* Y_meas, double* acc_meas, KalmanState* state, Param* param);

#endif // KALMAN_H