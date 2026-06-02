#include "../include/mpc_wrapper.h"
#include "../lib/osqp_c_code/include/osqp.h" 
#include <string.h>
#include <math.h>

extern OSQPWorkspace workspace;

void mpc_wrapper_init(MPCState* state) {
    memset(state->epsilon, 0, sizeof(state->epsilon));
    memset(state->u_prev, 0, sizeof(state->u_prev));
    memset(state->q_new, 0, sizeof(state->q_new));
    memset(state->l_new, 0, sizeof(state->l_new));
    memset(state->u_new, 0, sizeof(state->u_new));
}

void mpc_wrapper_outputs(double* u, MPCState* state) {
    for (int i = 0; i < 4; i++) {
        u[i] = state->u_prev[i];
    }
}

void mpc_wrapper_update(double* X_hat, double* X_ref, MPCState* state, Param* param) {
    int n = param->nx;       
    int m = param->nu;     
    int Np = param->Np;    
    int Nc = param->Nc;     
    int stride = Np + 1;    

    c_float X_aug[9];
    X_aug[0] = (c_float)X_hat[0]; 
    X_aug[1] = (c_float)X_hat[1];
    X_aug[2] = (c_float)X_hat[2]; 
    X_aug[3] = (c_float)X_hat[3]; 
    X_aug[4] = (c_float)X_hat[4];
    X_aug[5] = (c_float)X_hat[5]; 
    X_aug[6] = (c_float)state->epsilon[0]; 
    X_aug[7] = (c_float)state->epsilon[1]; 
    X_aug[8] = (c_float)state->epsilon[2]; 

    int idx_q = 0;

    for (int j = 0; j < Np; j++) {
        double rx = X_ref[0 * stride + j];
        double ry = X_ref[1 * stride + j];
        double rz = X_ref[2 * stride + j];

        state->q_new[idx_q++] = (c_float)(-param->Q_mpc[0] * rx); 
        state->q_new[idx_q++] = (c_float)(-param->Q_mpc[1] * ry); 
        state->q_new[idx_q++] = (c_float)(-param->Q_mpc[2] * rz); 
        state->q_new[idx_q++] = 0.0f;                             
        state->q_new[idx_q++] = 0.0f;                             
        state->q_new[idx_q++] = 0.0f;                            
        state->q_new[idx_q++] = 0.0f;                             
        state->q_new[idx_q++] = 0.0f;                             
        state->q_new[idx_q++] = 0.0f;                             
    }

    double X_ref_terminal[9] = {0.0};
    X_ref_terminal[0] = X_ref[0 * stride + Np]; 
    X_ref_terminal[1] = X_ref[1 * stride + Np]; 
    X_ref_terminal[2] = X_ref[2 * stride + Np]; 

    for (int i = 0; i < n; i++) {
        double sum = 0.0;
        for (int k = 0; k < n; k++) {
            sum += param->Q_Terminal[i][k] * X_ref_terminal[k];
        }
        state->q_new[idx_q++] = (c_float)(-sum);
    }

    for (int i = 0; i < (m * Nc); i++) {
        state->q_new[idx_q++] = 0.0f;
    }

    int idx_b = 0;

    for (int i = 0; i < n; i++) {
        state->l_new[idx_b] = X_aug[i];
        state->u_new[idx_b] = X_aug[i];
        idx_b++;
    }

    for (int i = 0; i < (n * Np); i++) {
        state->l_new[idx_b] = 0.0f;
        state->u_new[idx_b] = 0.0f;
        idx_b++;
    }

    for (int j = 0; j < (Np + 1); j++) {
        for (int i = 0; i < n; i++) {
            state->l_new[idx_b] = (c_float)param->X_min[i];
            state->u_new[idx_b] = (c_float)param->X_max[i];
            idx_b++;
        }
    }

    for (int j = 0; j < Nc; j++) {
        for (int i = 0; i < m; i++) {
            state->l_new[idx_b] = (c_float)param->U_min[i];
            state->u_new[idx_b] = (c_float)param->U_max[i];
            idx_b++;
        }
    }

    osqp_update_lin_cost(&workspace, state->q_new);
    osqp_update_bounds(&workspace, state->l_new, state->u_new);
    osqp_solve(&workspace);

    int cmd_start_idx = n * (Np + 1);
    for (int i = 0; i < m; i++) {
        state->u_prev[i] = (double)workspace.solution->x[cmd_start_idx + i];
    }

    double Ts_mpc = 1.0 / param->f_mpc;
    state->epsilon[0] += Ts_mpc * (X_ref[0 * stride] - X_hat[0]); 
    state->epsilon[1] += Ts_mpc * (X_ref[1 * stride] - X_hat[1]); 
    state->epsilon[2] += Ts_mpc * (X_ref[2 * stride] - X_hat[2]); 
}