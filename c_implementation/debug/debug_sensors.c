#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "param.h"
#include "../include/sensors.h"

static int double_equal(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

int main() {
    Param param;
    load_param(&param);

    param.sigma_pos = 0.0; 
    param.sigma_acc = 0.0;
    param.sigma_omega = 0.0;
    param.sigma_mag = 0.0;

    SensorsState state;
    sensors_init(&state, &param);

    double mocap_meas[3];

    double X_1[12] = {0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0};
    sensors_mocap(X_1, mocap_meas, &state, &param);

    assert(double_equal(mocap_meas[0], 0.0, 1e-6));

    double X_2[12] = {5.0, -2.0, 10.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0};
    sensors_mocap(X_2, mocap_meas, &state, &param);
    assert(double_equal(mocap_meas[0], 0.0, 1e-6));
    assert(double_equal(mocap_meas[1], 0.0, 1e-6));
    assert(double_equal(mocap_meas[2], 0.0, 1e-6));

    sensors_mocap(X_1, mocap_meas, &state, &param);
    assert(double_equal(mocap_meas[0], 5.0, 1e-6));
    assert(double_equal(mocap_meas[1], -2.0, 1e-6));
    assert(double_equal(mocap_meas[2], 10.0, 1e-6));

    printf(">>> ALL TESTS PASSED <<<\n");
    return 0;
}