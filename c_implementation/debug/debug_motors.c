#include <stdio.h>
#include <math.h>
#include <assert.h> 
#include "param.h"
#include "motors.h"

static int double_equal(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

int main() {
    Param param;
    load_param(&param);
    
    MotorsState state;
    motors_init(&state, &param);
    
    double u[4] = {1.0, 0.0, 0.0, 0.0};
    double v[4] = {0.0, 0.0, 0.0, 0.0};

    for (int i = 1; i <= 6; i++) {
        motors_update(u, v, &state, &param);
    }
    assert(double_equal(v[0], 0.6321, 0.0001));

    for (int i = 1; i <= 12; i++) {
        motors_update(u, v, &state, &param);
    }
    assert(double_equal(v[0], 0.9502, 0.0001));
    
    for (int i = 1; i <= 12; i++) {
        motors_update(u, v, &state, &param);
    }
    assert(double_equal(v[0], 0.9932, 0.0001));

    double u_sat[4] = {param.T_max * 2, 0.0, 0.0, 0.0};
    
    for (int i = 0; i < 20; i++) {
        motors_update(u_sat, v, &state, &param);
    }
    assert(double_equal(v[0], param.Ti_max * 4, 0.0001));

    printf(">>> ALL TESTS PASSED <<<\n");
    return 0;
}