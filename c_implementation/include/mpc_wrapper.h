#ifndef MPC_WRAPPER_H
#define MPC_WRAPPER_H

#include "param.h"
#include "gains.h"
#include "../lib/osqp_c_code/include/osqp.h" 

typedef struct {
    double epsilon[3];
    
    c_float q_new[PARAM_NX * (PARAM_NP + 1) + PARAM_NU * PARAM_NC]; 
    c_float l_new[2*PARAM_NX*(PARAM_NP + 1) + PARAM_NU * PARAM_NC];   
    c_float u_new[2*PARAM_NX*(PARAM_NP + 1) + PARAM_NU * PARAM_NC];   
} MPCState;

void mpc_wrapper_init(MPCState* state);
void mpc_wrapper_outputs(double* X_hat, double* X_ref, double* u, MPCState* state, Param* param);
void mpc_wrapper_update(double* X_hat, double* X_ref, MPCState* state, Param* param);

#endif // MPC_WRAPPER_H