#include "motors.h"
#include <math.h>

void motors_init(MotorsState* state, Param* param) {
    state->A = exp(-1.0 /param->f_plant/param->tau_m);
    state->B = 1.0 - state->A;

    double init_val = (param->m * param->g) / 4.0;
    for (int i = 0; i < 4; i++) {
        state->Ti_v[i] = init_val;
    }
}

void motors_outputs(double* v, MotorsState* state, Param* param) {
    double Ti_v_sat[4];
    
    for (int i = 0; i < 4; i++) {
        double val = state->Ti_v[i];
        if (val < (double)param->Ti_min) {
            Ti_v_sat[i] = (double)param->Ti_min;
        } else if (val > (double)param->Ti_max) {
            Ti_v_sat[i] = (double)param->Ti_max;
        } else {
            Ti_v_sat[i] = val;
        }
    }

    for (int i = 0; i < 4; i++) {
        double temp = 0.0;
        for (int j = 0; j < 4; j++) {
            temp += (double)param->M_T_u[i][j] * Ti_v_sat[j];
        }
        v[i] = temp;
    }
}

void motors_update(double* u, MotorsState* state, Param* param) {
    double Ti_u[4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Ti_u[i] += (double)param->M_u_T[i][j] * u[j];
        }
    }

    for (int i = 0; i < 4; i++) {
        state->Ti_v[i] = state->A * state->Ti_v[i] + state->B * Ti_u[i];
    }
}