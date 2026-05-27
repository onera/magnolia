#ifndef PLANT_H
#define PLANT_H

#include "param.h"
#include <math.h>

void plant_update(double* X, double* v, double* X_dot, Param* param);

#endif // PLANT_H