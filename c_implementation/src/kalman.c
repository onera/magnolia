#include "kalman.h"
#include <math.h>

void kalman_init(KalmanState* state, Param* param) {
    state->U_kalman[0] = param->m_tilde * param->g;
    state->U_kalman[1] = 0.0;
    state->U_kalman[2] = 0.0;
    state->U_kalman[3] = 0.0;

    for (int i = 0; i < 15; i++) {
        state->X_hat[i] = 0.0;
    }

    state->A_u = exp(-1.0 / (param->f_kalman * param->tau_m));
    state->B_u = 1.0 - state->A_u;
}

void kalman_update(double* u, double* Y_meas, double* acc_meas, double* X_hat, KalmanState* state, Param* param) {
    int i, j;

    for (i = 0; i < 15; i++) {
        X_hat[i] = state->X_hat[i];
    }

    double u_adapted[4];
    u_adapted[0] = u[0] - (param->m_tilde * param->g);
    u_adapted[1] = u[1];
    u_adapted[2] = u[2];
    u_adapted[3] = u[3];

    double acc_kalman[3];
    acc_kalman[0] = acc_meas[0];
    acc_kalman[1] = acc_meas[1];
    acc_kalman[2] = acc_meas[2] - param->g;

    double Y[12];
    for (i = 0; i < 9; i++)  Y[i] = Y_meas[i];
    for (i = 0; i < 3; i++)  Y[i + 9] = acc_kalman[i];

    double Y_hat[12] = {0.0};
    for (i = 0; i < 3; i++) {
        Y_hat[i] = state->X_hat[i];
        Y_hat[i+3] = state->X_hat[i+3];
        Y_hat[i+6] = state->X_hat[i+9];
    }
    Y_hat[9]  = state->X_hat[12] - param->g * state->X_hat[4];
    Y_hat[10] = state->X_hat[13] + param->g * state->X_hat[3];
    Y_hat[11] = state->X_hat[14];

    double innovation[12];
    for (i = 0; i < 12; i++) {
        innovation[i] = Y[i] - Y_hat[i];
    }

    double x_next[15] = {0.0};
    for (i = 0; i < 15; i++) {
        for (j = 0; j < 15; j++) {
            x_next[i] += param->A_kalman[i][j] * state->X_hat[j];
        }
        for (j = 0; j < 4; j++) {
            x_next[i] += param->B_kalman[i][j] * state->U_kalman[j];
        }
        for (j = 0; j < 12; j++) {
            x_next[i] += param->K_kalman[i][j] * innovation[j];
        }
    }

    for (i = 0; i < 4; i++) {
        state->U_kalman[i] = state->A_u * state->U_kalman[i] + state->B_u * u_adapted[i];
    }

    for (i = 0; i < 15; i++) {
        state->X_hat[i] = x_next[i];
    }
}