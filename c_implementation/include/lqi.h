#ifndef LQI_H
#define LQI_H

#include "param.h"

typedef struct {
    double epsilon[3];
    double prev_state[6];
} LQIState;

void lqi_init(LQIState* state);
void lqi_outputs(double* tau, LQIState* state, Param* param);
void lqi_update(double* X, double* eta_ref, LQIState* state, Param* param);

#endif // LQI_H