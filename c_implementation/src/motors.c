#include "../include/motors.h"
#include <math.h>

void motors_init(MotorsState* state, Param* param) {
    state->A = exp(-1/(param->f_plant*param->tau_m));
    state->B = 1 - state->A;
    for (int i = 0; i < 4; i++) {
        state->Ti_v[i] = 0;
    }
}

void motors_update(double* u, double* v, MotorsState* state, Param* param) {
    // u = [T_cmd tau_phi_cmd tau_theta_cmd tau_psi_cmd]
    // v = [T tau_phi tau_theta tau_psi]

    double Ti_u[4];

    for (int i = 0; i < 4; i++) {
        Ti_u[i] = 0;
        for (int j = 0; j < 4; j++) {
            Ti_u[i] += param->M_u_T[i][j] * u[j];
        }
    }

    for (int i = 0; i < 4; i++) {
        state->Ti_v[i] = state->A * state->Ti_v[i] + state->B * Ti_u[i];
    }

    double Ti_v_sat[4];
    for (int i = 0; i < 4; i++) {
        Ti_v_sat[i] = state->Ti_v[i];
        if (Ti_v_sat[i] < param->Ti_min) {
            Ti_v_sat[i] = param->Ti_min;
        } else if (Ti_v_sat[i] > param->Ti_max) {
            Ti_v_sat[i] = param->Ti_max;
        }
    }

    for (int i = 0; i < 4; i++) {
        v[i] = 0;
        for (int j = 0; j < 4; j++) {
            v[i] += param->M_T_u[i][j] * Ti_v_sat[j];
        }
    }
}