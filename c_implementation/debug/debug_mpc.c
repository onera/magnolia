#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "param.h"
#include "mpc_wrapper.h"
#include "../lib/osqp_c_code/include/osqp.h"

extern OSQPWorkspace workspace;

int main() {
    Param param;
    load_param(&param);

    MPCState mpc_state;
    mpc_wrapper_init(&mpc_state);

    double X_est[9] = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

    double X_ref_horizon[279] = {0.0};
    for (int j = 0; j <= 30; j++) {
        X_ref_horizon[j * 9 + 0] = 2.0;
        X_ref_horizon[j * 9 + 1] = 2.0; 
        X_ref_horizon[j * 9 + 2] = 4.0; 
    }

    double U_optimal_out[3] = {0.0, 0.0, 0.0};

    mpc_wrapper_step(X_est, X_ref_horizon, U_optimal_out, &mpc_state, &param);

    double expected_integral = (1.0 / param.f_mpc) / 2.0 * 1.0;
    assert(fabs(mpc_state.epsilon[0] - expected_integral) < 1e-6);
    assert(fabs(mpc_state.q_new[0] - (-2.0f)) < 1e-4);
    assert(fabs(mpc_state.l_new[0] - 1.0f) < 1e-4);
    assert(fabs(mpc_state.u_new[0] - 1.0f) < 1e-4);

    printf(">>> ALL TESTS PASSED <<<\n");
    return 0;
}