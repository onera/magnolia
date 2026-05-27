#include "../include/lqi.h"

void lqi_init(LQIState* state) {
    for (int i = 0; i < 3; i++) {
        state->epsilon[i] = 0.0;
        state->error_old[i] = 0.0;
    }
}

void lqi_update(double* X, double* eta_ref, double* tau, LQIState* state, Param* param) {

    double dt = 1.0 / param->f_lqi;
    double error_now[3];

    error_now[0] = X[3] - eta_ref[0]; 
    error_now[1] = X[4] - eta_ref[1]; 
    error_now[2] = X[5] - eta_ref[2]; 

    for (int i = 0; i < 3; i++) {
        state->epsilon[i] += (dt / 2.0) * (error_now[i] + state->error_old[i]);
        state->error_old[i] = error_now[i];
    }

    double X_aug[9];
    X_aug[0] = X[3];   
    X_aug[1] = X[4];   
    X_aug[2] = X[5]; 
    X_aug[3] = X[9];   
    X_aug[4] = X[10];  
    X_aug[5] = X[11];  
    X_aug[6] = state->epsilon[0];
    X_aug[7] = state->epsilon[1];
    X_aug[8] = state->epsilon[2];

    for (int i = 0; i < 3; i++) {
        tau[i] = 0.0;
        for (int j = 0; j < 9; j++) {
            tau[i] -= param->K_lqi[i][j] * X_aug[j];
        }
    }
}