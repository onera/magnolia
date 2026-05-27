#ifndef PARAM_H
#define PARAM_H

#include <string.h>

typedef struct {
    // Physical parameters
    double g;
    double Ix; double Iy; double Iz;
    double m;
    double L;
    double Ti_min; double Ti_max;
    double f_plant;

    // Model parameters
    double Ix_tilde; double Iy_tilde; double Iz_tilde;
    double m_tilde;
    double L_tilde;
    double ci;
    double T_min; double T_max;

    // Kalman filter parameters
    double f_kalman;
    double A_kalman[15][15];
    double B_kalman[15][4];
    double C_kalman[12][15];
    double K_kalman[15][12];

    // LQI controller parameters
    double f_lqi;
    double K_lqi[3][9];

    // MPC parameters
    double f_mpc;
    int Np; int Nc;
    int nx; int nu;
    double X_max[9]; double X_min[9];
    double U_max[3]; double U_min[3];
    double Q_mpc[9];
    double Q_Terminal[9][9];
    double Wterminal;

    // Mocap parameters
    double sigma_pos;
    double latency_MoCap;
    double f_MoCap;

    // Gyrometer parameters
    double sigma_omega;
    double b_gyro;
    double f_c_gyro;
    double f_gyro;

    // Accelerometer parameters
    double sigma_acc;
    double b_acc;
    double f_acc;

    // Magnetometer parameters
    double m_N; double m_E; double m_D;
    double B_mag[3];
    double A_mag[3][3];
    double sigma_mag;
    double f_mag;

    // Mahony filter parameters
    double f_mahony;
    double Kp_mahony;
    double Ki_mahony;

    // Motor parameters
    double tau_m;
    double M_T_u[4][4];
    double M_u_T[4][4];
} Param;

void load_param(Param* param);

#endif // PARAM_H