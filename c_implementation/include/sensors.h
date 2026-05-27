#ifndef SENSORS_H
#define SENSORS_H

#include "param.h"
#include "utils.h"

typedef struct {
    double p_buffer[2][3]; 
    int write_index;
} SensorsState;

void sensors_init(SensorsState* state, Param* param);

void sensors_accelerometer(double* X_dot, double* X, double* acc_meas, Param* param);

void sensors_gyrometer(double* X, double* gyro_meas, Param* param);

void sensors_magnetometer(double* X, double* mag_meas, Param* param);

void sensors_mocap(double* X, double* mocap_meas, SensorsState* state, Param* param);

#endif // SENSORS_H