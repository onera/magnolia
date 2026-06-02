#include "motors.h"
#include <math.h>

void motors_init(MotorsState* state, Param* param) {
    state->A = exp(-1/(param->f_plant*param->tau_m));
    state->B = 1 - state->A;
    for (int i = 0; i < 4; i++) {
        state->Ti_v[i] = (param->m * param->g) / 4.0;
    }
}

void motors_outputs(double* v, MotorsState* state, Param* param) {
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

void motors_update(double* u, MotorsState* state, Param* param) {
    double Ti_u[4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Ti_u[i] += param->M_u_T[i][j] * u[j];
        }
    }

    for (int i = 0; i < 4; i++) {
        state->Ti_v[i] = state->A * state->Ti_v[i] + state->B * Ti_u[i];
    }
}