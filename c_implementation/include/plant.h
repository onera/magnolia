#ifndef PLANT_H
#define PLANT_H

#include "param.h"
#include <math.h>

typedef struct {
    double X[12];
    double X_dot[12];
} PlantState;

void plant_outputs(double* X, double* X_dot, PlantState* state);
void plant_update(double* v, Param* param, PlantState* state);

#endif // PLANT_H