#include "lqi.h"

void lqi_init(LQIState* state) {
    memset(state->epsilon, 0, sizeof(state->epsilon));
}

void lqi_outputs(double* eta_hat, double* omega_hat, double* eta_ref, double* tau, LQIState* state, Param* param) {
    double X_aug[9];
    for (int i = 0; i < 3; i++) {
        X_aug[i] = eta_hat[i] - eta_ref[i];
        X_aug[i+3] = omega_hat[i];
        X_aug[i+6] = state->epsilon[i];
    }

    for (int i = 0; i < 3; i++) {
        tau[i] = 0.0;
        for (int j = 0; j < 9; j++) {
            tau[i] -= param->K_lqi[i][j] * X_aug[j];
        }
    }
}

void lqi_update(double* eta_hat, double* eta_ref, LQIState* state, Param* param) {

    double dt = 1.0 / param->f_lqi;
    double error[3];

    for (int i = 0; i < 3; i++) {
        error[i] = eta_ref[i] - eta_hat[i];
        state->epsilon[i] += dt * error[i];
    }
}