#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "param.h"
#include "mahony_wrapper.h"

int main() {
    Param param;
    load_param(&param);

    MahonyState state;
    mahony_wrapper_init(&state);

    double gyro_meas[3] = {0.0, 0.0, 0.0};
    double acc_meas[3]  = {0.0, 0.0, 9.81}; 
    double mag_meas[3]  = {20.0, 0.0, 40.0};
    double p_ddot_hat[3] = {0.0, 0.0, 0.0};
    
    double eta_hat[3]   = {0.0, 0.0, 0.0};
    double omega_hat[3] = {0.0, 0.0, 0.0};

    mahony_wrapper_step(gyro_meas, acc_meas, mag_meas, p_ddot_hat, eta_hat, omega_hat, &state, &param);
    assert(fabs(eta_hat[0]) < 1e-3);
    assert(fabs(eta_hat[1]) < 1e-3);
    assert(state.q0 == 1.0f || state.q0 < 1.0f);

    printf(">>> ALL TESTS PASSED <<<\n");
    return 0;
}