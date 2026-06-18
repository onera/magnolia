#ifndef LQI_H
#define LQI_H

#include "param.h"
#include "gains.h"

typedef struct {
    double epsilon[3];
} LQIState;

void lqi_init(LQIState* state);
void lqi_outputs(double* eta_hat, double* omega_hat, double* eta_ref, double* tau, LQIState* state, Param* param);
void lqi_update(double* eta_hat, double* eta_ref, LQIState* state, Param* param);

#endif // LQI_H