#include "../include/sensors.h"
#include <math.h>

void sensors_init(SensorsState* state, Param* param) {
    state->write_index = 0;
    for (int i = 0; i < 2; i++) {
        state->p_buffer[i][0] = 0.0;
        state->p_buffer[i][1] = 0.0;
        state->p_buffer[i][2] = 0.0;
    }
}

void sensors_accelerometer(double* X_dot, double* X, double* acc_meas, Param* param) {
    double phi = X[3];   double theta = X[4]; double psi = X[5];

    double cphi = cos(phi);   double sphi = sin(phi);
    double ctheta = cos(theta); double stheta = sin(theta);
    double cpsi = cos(psi);   double spsi = sin(psi);

    double R_B[3][3] = {
        {ctheta*cpsi, sphi*stheta*cpsi-cphi*spsi, cphi*stheta*cpsi+sphi*spsi},
        {ctheta*spsi, sphi*stheta*spsi+cphi*cpsi, cphi*stheta*spsi-sphi*cpsi},
        {-stheta,     sphi*ctheta,                cphi*ctheta}
    };

    for (int i = 0; i < 3; i++) {
        acc_meas[i] = R_B[0][i] * X_dot[6] 
                    + R_B[1][i] * X_dot[7] 
                    + R_B[2][i] * (X_dot[8] - param->g);
                    
        acc_meas[i] += param->b_acc + generate_gaussian_noise(param->sigma_acc, param->f_acc);
    }
}


void sensors_gyrometer(double* X, double* gyro_meas, Param* param) {
    gyro_meas[0] = X[9]  + param->b_gyro + generate_gaussian_noise(param->sigma_omega, param->f_gyro);
    gyro_meas[1] = X[10] + param->b_gyro + generate_gaussian_noise(param->sigma_omega, param->f_gyro);
    gyro_meas[2] = X[11] + param->b_gyro + generate_gaussian_noise(param->sigma_omega, param->f_gyro);
}

void sensors_magnetometer(double* X, double* mag_meas, Param* param) {
    double phi = X[3];   double theta = X[4]; double psi = X[5];

    double cphi = cos(phi);   double sphi = sin(phi);
    double ctheta = cos(theta); double stheta = sin(theta);
    double cpsi = cos(psi);   double spsi = sin(psi);

    double R_B[3][3] = {
        {ctheta*cpsi, sphi*stheta*cpsi-cphi*spsi, cphi*stheta*cpsi+sphi*spsi},
        {ctheta*spsi, sphi*stheta*spsi+cphi*cpsi, cphi*stheta*spsi-sphi*cpsi},
        {-stheta,     sphi*ctheta,                cphi*ctheta}
    };

    for (int i = 0; i < 3; i++) {
        mag_meas[i] = R_B[0][i] * param->m_N + R_B[1][i] * param->m_E + R_B[2][i] * param->m_D;
    }

    double mx = mag_meas[0]; double my = mag_meas[1]; double mz = mag_meas[2];
    
    for (int i = 0; i < 3; i++) {
        mag_meas[i] = param->A_mag[i][0] * mx + param->A_mag[i][1] * my + param->A_mag[i][2] * mz
                    + param->B_mag[i]
                    + generate_gaussian_noise(param->sigma_mag, param->f_mag);
    }
}

void sensors_mocap(double* X, double* mocap_meas, SensorsState* state, Param* param) {
    state->p_buffer[state->write_index][0] = X[0];
    state->p_buffer[state->write_index][1] = X[1];
    state->p_buffer[state->write_index][2] = X[2];

    int read_index = (state->write_index + 1) % 2;

    mocap_meas[0] = state->p_buffer[read_index][0] + generate_gaussian_noise(param->sigma_pos, param->f_MoCap);
    mocap_meas[1] = state->p_buffer[read_index][1] + generate_gaussian_noise(param->sigma_pos, param->f_MoCap);
    mocap_meas[2] = state->p_buffer[read_index][2] + generate_gaussian_noise(param->sigma_pos, param->f_MoCap);

    state->write_index = (state->write_index + 1) % 2;
}