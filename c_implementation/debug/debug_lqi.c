#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "param.h"
#include "lqi.h"

static int double_equal(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

int main() {
    Param param;
    load_param(&param);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 9; j++) {
            param.K_lqi[i][j] = 0.0;
        }
        param.K_lqi[i][i]     = 1.0;
        param.K_lqi[i][i + 3] = 2.0; 
        param.K_lqi[i][i + 6] = 3.0; 
    }

    LQIState state;
    lqi_init(&state);

    double X[12] = {0.0, 0.0, 0.0,  0.2, 0.0, 0.0,  0.0, 0.0, 0.0,  0.5, 0.0, 0.0};
    double targets[3] = {0.0, 0.0, 0.0};
    double u_lqi[3] = {0.0, 0.0, 0.0};

    lqi_update(X, targets, u_lqi, &state, &param);
    assert(double_equal(state.epsilon[0], 0.001, 1e-7));
    assert(double_equal(state.epsilon[1], 0.0, 1e-7));
    assert(double_equal(u_lqi[0], -1.203, 1e-7));

    lqi_update(X, targets, u_lqi, &state, &param);
    assert(double_equal(state.epsilon[0], 0.003, 1e-7));
    assert(double_equal(u_lqi[0], -1.209, 1e-7));

    printf(">>> ALL TESTS PASSED <<<\n");
    return 0;
}