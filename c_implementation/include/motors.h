#ifndef MOTORS_H
#define MOTORS_H

#include "param.h"

typedef struct {
    double A;
    double B;
    double Ti_v[4];
} MotorsState;

void motors_init(MotorsState* state, Param* param);
void motors_outputs(double* v, MotorsState* state, Param* param);
void motors_update(double* u, MotorsState* state, Param* param);

#endif // MOTORS_H