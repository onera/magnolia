#ifndef SENSORS_H
#define SENSORS_H

#include "param.h"
#include "utils.h"

typedef struct {
    double acc_meas[3];
    double gyro_meas[3];
    double mag_meas[3];
    double mocap_meas[3];
    double p_buffer[2][3]; 
    int write_index;
} SensorsState;

void sensors_init(SensorsState* state, Param* param);

void accelerometer_outputs(double* acc_meas, Param* param, SensorsState* state);
void accelerometer_update(double* X_dot, double* X, Param* param, SensorsState* state);

void gyrometer_outputs(double* gyro_meas, Param* param, SensorsState* state);
void gyrometer_update(double* X, Param* param, SensorsState* state);

void magnetometer_outputs(double* mag_meas, Param* param, SensorsState* state);
void magnetometer_update(double* X, Param* param, SensorsState* state);

void mocap_outputs(double* mocap_meas, Param* param, SensorsState* state);
void mocap_update(double* X, double* mocap_meas, Param* param, SensorsState* state);

#endif // SENSORS_H