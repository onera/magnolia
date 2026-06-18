#ifndef MAHONY_WRAPPER_H
#define MAHONY_WRAPPER_H

#include "param.h"

typedef struct {
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
    double gyro_smoothed_prev[3];
    double eta_hat_prev[3];
} MahonyState;


void mahony_wrapper_init(MahonyState* state);

void mahony_wrapper_outputs(double* gyro_meas, double* acc_meas, double* mag_meas, double* p_ddot_hat, 
                            double* eta_hat, double* omega_hat, MahonyState* state, Param* param);

#endif // MAHONY_WRAPPER_H