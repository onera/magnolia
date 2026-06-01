#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "param.h"
#include "plant.h"

static int double_equal(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

int main() {
    Param param;
    load_param(&param);

    double dt = 1.0 / param.f_plant;
    
    double X[12] = {0.0, 0.0, 10.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0};
    double X_dot[12] = {0};
    double v_hover[4] = {param.m * param.g, 0.0, 0.0, 0.0};
    plant_update(X, v_hover, X_dot, &param);
    assert(double_equal(X_dot[8], 0.0, 1e-7));
    assert(double_equal(X[8], 0.0, 1e-7));
    assert(double_equal(X[2], 10.0, 1e-7));
    
    double X_chute[12] = {0.0, 0.0, 10.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0};
    double v_cut[4] = {0.0, 0.0, 0.0, 0.0};
    plant_update(X_chute, v_cut, X_dot, &param);
    assert(double_equal(X_dot[8], -param.g, 1e-7));
    assert(double_equal(X_chute[8], -param.g * dt, 1e-7));
    
    double X_roll[12] = {0.0, 0.0, 10.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0};
    double v_roll[4] = {param.m * param.g, 0.01, 0.0, 0.0};
    plant_update(X_roll, v_roll, X_dot, &param);
    assert(double_equal(X_dot[9], 0.01 / param.Ix, 1e-7));
    assert(double_equal(X_roll[9], 0.01 / param.Ix * dt, 1e-7));

    printf(">>> ALL TESTS PASSED <<<\n");
    return 0;
}