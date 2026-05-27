#include "../include/param.h"

void load_param(Param* param) {
    const double pi = 3.141592653589793;

    // Physical parameters
    param->g = 9.81;
    param->Ix = 2.24e-3; param->Iy = 2.9e-3; param->Iz = 5.3e-3;
    param->m = 0.460;
    param->L = 0.127;
    param->Ti_min = 0; param->Ti_max = 2.5;
    param->f_plant = 200;

    // Model parameters
    param->Ix_tilde = param->Ix; param->Iy_tilde = param->Iy; param->Iz_tilde = param->Iz;
    param->m_tilde = param->m;
    param->L_tilde = param->L;
    param->ci = 0.015;
    param->T_min = 4*param->Ti_min - param->m_tilde*param->g; 
    param->T_max = 4*param->Ti_max - param->m_tilde*param->g;

    // Kalman filter parameters
    param->f_kalman = 100;
    // TODO Fill with actual values
    /*
    param->A_kalman[15][15] = {0};
    param->B_kalman[15][4] = {0}; 
    param->C_kalman[12][15] = {0};
    param->K_kalman[15][12] = {0}; 
    */

    // LQI controller parameters
    param->f_lqi = 100;
    // TODO Fill with actual values
    /*
    param->K_lqi[3][9] = {0}; 
    */

    // MPC parameters
    param->f_mpc = 10;
    param->Np = 30; param->Nc = 10;
    param->nx = 9; param->nu = 3;
    param->X_max[1] = 1e3; param->X_max[2] = 1e3; param->X_max[3] = 1e3; param->X_max[4] = 1e3; param->X_max[5] = 1e3; param->X_max[6] = 1e3; param->X_max[7] = 1e3; param->X_max[8] = 1e3;
    param->X_min[0] = -1e3; param->X_min[1] = -1e3; param->X_min[2] = -1e3; param->X_min[3] = -1e3; param->X_min[4] = -1e3; param->X_min[5] = -1e3; param->X_min[6] = -1e3; param->X_min[7] = -1e3; param->X_min[8] = -1e3;
    param->U_max[0] = param->T_max; param->U_max[1] = pi/9; param->U_max[2] = pi/9;
    param->U_min[0] = param->T_min; param->U_min[1] = -pi/9; param->U_min[2] = -pi/9;
    // TODO Fill with actual values
    /*
    param->Q_mpc[9] = {0}; 
    param->Q_Terminal[9][9] = {0};
    */
    param->Wterminal = 1;

    // Mocap parameters
    param->sigma_pos = 0.001;
    param->latency_MoCap = 0.01;
    param->f_MoCap = 100;

    // Gyrometer parameters
    param->sigma_omega = 0.01;
    param->b_gyro = 0.02 * pi/180;
    param->f_c_gyro = 30;
    param->f_gyro = 200;

    // Accelerometer parameters
    param->sigma_acc = 0.1;
    param->b_acc = 0.15;
    param->f_acc = 200;

    // Magnetometer parameters
    param->m_N = 22.3; param->m_E = 0; param->m_D = 41.94;
    param->B_mag[0] = 0.15;
    param->B_mag[1] = -0.25;
    param->B_mag[2] = 0.10;
    param->A_mag[0][0] = 1.005; param->A_mag[0][1] = 0.002; param->A_mag[0][2] = 0.001;
    param->A_mag[1][0] = 0.002; param->A_mag[1][1] = 0.998; param->A_mag[1][2] = 0.003;
    param->A_mag[2][0] = 0.001; param->A_mag[2][1] = 0.003; param->A_mag[2][2] = 1.002;
    param->sigma_mag = 0.6;
    param->f_mag = 50;

    // Mahony filter parameters
    param->f_mahony = 200;
    param->Kp_mahony = 0.5;
    param->Ki_mahony = 0.01;

    // Motor parameters
    param->tau_m = 0.03;
    param->M_T_u[0][0] = 1; param->M_T_u[0][1] = 1; param->M_T_u[0][2] = 1; param->M_T_u[0][3] = 1;
    param->M_T_u[1][0] = param->L; param->M_T_u[1][1] = -param->L; param->M_T_u[1][2] = -param->L; param->M_T_u[1][3] = param->L;
    param->M_T_u[2][0] = -param->L; param->M_T_u[2][1] = -param->L; param->M_T_u[2][2] = param->L; param->M_T_u[2][3] = param->L;
    param->M_T_u[3][0] = param->ci; param->M_T_u[3][1] = -param->ci; param->M_T_u[3][2] = param->ci; param->M_T_u[3][3] = -param->ci;

    param->M_u_T[0][0] = 0.25; param->M_u_T[0][1] =  0.25/param->L; param->M_u_T[0][2] = -0.25/param->L; param->M_u_T[0][3] =  0.25/param->ci;
    param->M_u_T[1][0] = 0.25; param->M_u_T[1][1] = -0.25/param->L; param->M_u_T[1][2] = -0.25/param->L; param->M_u_T[1][3] = -0.25/param->ci;
    param->M_u_T[2][0] = 0.25; param->M_u_T[2][1] = -0.25/param->L; param->M_u_T[2][2] =  0.25/param->L; param->M_u_T[2][3] =  0.25/param->ci;
    param->M_u_T[3][0] = 0.25; param->M_u_T[3][1] =  0.25/param->L; param->M_u_T[3][2] =  0.25/param->L; param->M_u_T[3][3] = -0.25/param->ci;
}