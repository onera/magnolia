#include "lqi.h"

void lqi_init(LQIState* state) {
    for (int i = 0; i < 3; i++) {
        state->epsilon[i] = 0.0;
    }
    for (int i = 0; i < 6; i++) {
        state->prev_state[i] = 0.0;
    }
}

void lqi_outputs(double* tau, LQIState* state, Param* param) {
    double X_aug[9];
    X_aug[0] = state->prev_state[0];   
    X_aug[1] = state->prev_state[1];   
    X_aug[2] = state->prev_state[2]; 
    X_aug[3] = state->prev_state[3];   
    X_aug[4] = state->prev_state[4];  
    X_aug[5] = state->prev_state[5];  
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

void lqi_update(double* X, double* eta_ref, LQIState* state, Param* param) {

    double dt = 1.0 / param->f_lqi;
    double error[3];

    error[0] = eta_ref[0] - X[3]; 
    error[1] = eta_ref[1] - X[4]; 
    error[2] = eta_ref[2] - X[5]; 

    for (int i = 0; i < 3; i++) {
        state->epsilon[i] += dt * error[i];
    }

    for (int i = 0; i < 3; i++) {
        state->prev_state[i] = X[i+3];
        state->prev_state[i+3] = X[i+9];
    }
}