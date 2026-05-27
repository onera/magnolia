#ifndef MPC_WRAPPER_H
#define MPC_WRAPPER_H

#include "param.h"
#include "../lib/osqp_c_code/include/osqp.h" 

typedef struct {
    double error_integral[3];
    double prev_error[3];
    
    c_float q_new[309];  
    c_float l_new[639];   
    c_float u_new[639];   
} MPCState;

void mpc_wrapper_init(MPCState* state);

void mpc_wrapper_step(double* X_est, double* X_ref_horizon, double* U_optimal_out, 
                      MPCState* state, Param* param);

#endif