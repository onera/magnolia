#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "../include/param.h"
#include "../include/kalman.h"

static int double_equal(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

int main() {
    Param param;
    load_param(&param);

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            param.A_kalman[i][j] = (i == j) ? 0.5 : 0.0;
        }
        for (int j = 0; j < 4; j++) {
            param.B_kalman[i][j] = (i == j) ? 1.0 : 0.0;
        }
        for (int j = 0; j < 12; j++) {
            param.K_kalman[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 15; j++) {
            param.C_kalman[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    KalmanState state;
    kalman_init(&state, &param);

    double u[4] = {param.m_tilde * param.g, 0.0, 0.0, 0.0}; 
    double Y_meas[9] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}; 
    double acc_meas[3] = {0.0, 0.0, param.g}; 
    double X_hat[15];

    kalman_update(u, Y_meas, acc_meas, X_hat, &state, &param);

    for (int i = 0; i < 15; i++) {
        assert(double_equal(X_hat[i], 0.0, 1e-7));
    }

    assert(double_equal(state.X_hat[0], param.m_tilde * param.g, 1e-6));

    assert(double_equal(state.X_hat[4], 1.0, 1e-6));

    double A_u = exp(-1.0 / (param.f_kalman * param.tau_m));
    assert(double_equal(state.U_kalman[0], A_u * param.m_tilde * param.g, 1e-6));

     printf(">>> ALL TESTS PASSED <<<\n");
    return 0;
}