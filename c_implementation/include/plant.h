#ifndef PLANT_H
#define PLANT_H

#include "param.h"
#include <math.h>

typedef struct {
    double X[12];
} PlantState;

void plant_init(PlantState* state, Param* param);
void plant_outputs(double* v, double* X, double* X_dot, Param* param, PlantState* state);
void plant_update(double* X_dot, Param* param, PlantState* state);

#endif // PLANT_H