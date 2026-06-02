#include "plant.h"

void plant_outputs(double* X, double* X_dot, PlantState* state) {
    for (int i = 0; i < 12; i++) {
        X[i] = state->X[i];
        X_dot[i] = state->X_dot[i];
    }
}

void plant_update(double* v, Param* param, PlantState* state) {
    // X = [x y z phi theta psi x_dot y_dot z_dot p q r]
    // v = [T tau_phi tau_theta tau_psi]

    double eta[3] = {state->X[3], state->X[4], state->X[5]};
    double p_dot[3] = {state->X[6], state->X[7], state->X[8]};
    double omega[3] = {state->X[9], state->X[10], state->X[11]};
    double phi = eta[0];
    double theta = eta[1];
    double psi = eta[2];

    double cphi = cos(phi); double sphi = sin(phi);
    double ctheta = cos(theta); double stheta = sin(theta); double ttheta = tan(theta);
    double cpsi = cos(psi); double spsi = sin(psi); double sec_theta = 1/cos(theta);

    double T_eta[3][3] = {{1, sphi*ttheta, cphi*ttheta},
                          {0, cphi, -sphi},
                          {0, sphi*sec_theta, cphi*sec_theta}};

    double eta_dot[3];
    for (int i = 0; i < 3; i++) {
        eta_dot[i] = 0;
        for (int j = 0; j < 3; j++) {
            eta_dot[i] += T_eta[i][j] * omega[j];
        }
    }

    double R_B[3][3] = {{ctheta*cpsi, sphi*stheta*cpsi-cphi*spsi, cphi*stheta*cpsi+sphi*spsi},
                        {ctheta*spsi, sphi*stheta*spsi+cphi*cpsi, cphi*stheta*spsi-sphi*cpsi},
                        {-stheta, sphi*ctheta, cphi*ctheta}};
    
    double a3[3] = {0, 0, 1};

    double A[12] = {0};
    for (int i = 0; i < 3; i++) {
        A[i] = p_dot[i];
        A[i+3] = eta_dot[i];
    }
    A[8] = -param->g * a3[2];
    A[9] = (param->Iy - param->Iz) / param->Ix * omega[1] * omega[2];
    A[10] = (param->Iz - param->Ix) / param->Iy * omega[0] * omega[2];
    A[11] = (param->Ix - param->Iy) / param->Iz * omega[0] * omega[1];

    double B[12][4] = {{0}};
    for (int i = 0; i < 3; i++) {
        B[i+6][0] = R_B[i][2] / param->m;
    }
    B[9][1] = 1 / param->Ix;
    B[10][2] = 1 / param->Iy;
    B[11][3] = 1 / param->Iz;

    for (int i = 0; i < 12; i++) {
        state->X[i] += state->X_dot[i] / param->f_plant;
    }

    for (int i = 0; i < 12; i++) {
        state->X_dot[i] = A[i];
        for (int j = 0; j < 4; j++) {
            state->X_dot[i] += B[i][j] * v[j];
        }
    }
}