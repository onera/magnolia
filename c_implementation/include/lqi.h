#ifndef LQI_H
#define LQI_H

#include "param.h"

typedef struct {
    double epsilon[3];
    double error_old[3];
} LQIState;

void lqi_init(LQIState* state);
void lqi_update(double* X, double* eta_ref, double* tau, LQIState* state, Param* param);

#endif