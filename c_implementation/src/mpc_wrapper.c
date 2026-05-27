#include "../include/mpc_wrapper.h"
#include "../lib/osqp_c_code/include/osqp.h" 
#include <string.h>
#include <math.h>

extern OSQPWorkspace workspace;

void mpc_wrapper_init(MPCState* state) {
    memset(state->error_integral, 0, sizeof(state->error_integral));
    memset(state->prev_error, 0, sizeof(state->prev_error));
    memset(state->q_new, 0, sizeof(state->q_new));
    memset(state->l_new, 0, sizeof(state->l_new));
    memset(state->u_new, 0, sizeof(state->u_new));
}

void mpc_wrapper_step(double* X_est, double* X_ref_horizon, double* U_optimal_out, 
                      MPCState* state, Param* param) {
    
    double Ts = 1.0 / param->f_mpc;
    for (int i = 0; i < 3; i++) {
        double current_error = X_ref_horizon[i] - X_est[i];
        state->error_integral[i] += (Ts / 2.0) * (current_error + state->prev_error[i]);
        state->prev_error[i] = current_error;
    }

    c_float X_aug[9];
    for (int i = 0; i < 6; i++) {
        X_aug[i] = (c_float)X_est[i];
    }
    X_aug[6] = (c_float)state->error_integral[0];
    X_aug[7] = (c_float)state->error_integral[1];
    X_aug[8] = (c_float)state->error_integral[2];

    int idx = 0;

    for (int j = 0; j < param->Np; j++) {
        for (int i = 0; i < param->nx; i++) {
            double val_ref = X_ref_horizon[j * param->nx + i];
            state->q_new[idx++] = (c_float)(-param->Q_mpc[i] * val_ref);
        }
    }

    for (int i = 0; i < param->nx; i++) {
        double double_sum = 0.0;
        for (int k = 0; k < param->nx; k++) {
            double_sum += param->Q_Terminal[i][k] * X_ref_horizon[param->Np * param->nx + k];
        }
        state->q_new[idx++] = (c_float)(-double_sum);
    }

    for (int i = 0; i < (param->nu * param->Nc); i++) {
        state->q_new[idx++] = 0.0f;
    }

    int idx = 0;

    for (int i = 0; i < param->nx; i++) {
        state->l_new[idx] = X_aug[i];
        state->u_new[idx] = X_aug[i];
        idx++;
    }

    for (int i = 0; i < (param->nx * param->Np); i++) {
        state->l_new[idx] = 0.0f;
        state->u_new[idx] = 0.0f;
        idx++;
    }

    for (int j = 0; j <= param->Np; j++) {
        for (int i = 0; i < param->nx; i++) {
            state->l_new[idx] = (c_float)param->X_min[i];
            state->u_new[idx] = (c_float)param->X_max[i];
            idx++;
        }
    }

    for (int j = 0; j < param->Nc; j++) {
        for (int i = 0; i < param->nu; i++) {
            state->l_new[idx] = (c_float)param->U_min[i];
            state->u_new[idx] = (c_float)param->U_max[i];
            idx++;
        }
    }

    osqp_update_lin_cost(&workspace, state->q_new);
    osqp_update_bounds(&workspace, state->l_new, state->u_new);
    osqp_solve(&workspace);

    int cmd_start_idx = param->nx * (param->Np + 1);
    for (int i = 0; i < param->nu; i++) {
        U_optimal_out[i] = (double)workspace.solution->x[cmd_start_idx + i];
    }
}