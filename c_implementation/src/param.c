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
    param->Q_mpc[0] = 1; param->Q_mpc[1] = 1; param->Q_mpc[2] = 1; param->Q_mpc[3] = 5e3; param->Q_mpc[4] = 5e3; param->Q_mpc[5] = 1e3; param->Q_mpc[6] = 5e3; param->Q_mpc[7] = 5e3; param->Q_mpc[8] = 100;
    // TODO Fill with actual values
    double Q_T_data[9][9] = {
        {2.587978389381442e+05,  1.300837244564020e-08,  1.759289229976043e-09,  1.706376869216877e+05,  2.206620987728035e-09,  2.049414645825843e-10, -1.398705470861162e+05, -9.370132654504791e-09, -2.146099270809591e-09},
        {1.300837244564020e-08,  2.587978389381748e+05, -1.440552439354272e-09,  1.095673800264782e-08,  1.706376869217023e+05, -1.276694823525542e-10, -5.274688821028189e-09, -1.398705470861342e+05,  1.547296674362151e-09},
        {1.759289229976043e-09, -1.440552439354272e-09,  8.976922864068971e+03,  7.813617554635597e-10,  2.422298678020990e-09,  1.297017704135240e+03, -1.024139422433787e-09,  1.526976924255056e-09, -3.548811126035567e+03},
        {1.706376869216877e+05,  1.095673800264782e-08,  7.813617554635597e-10,  2.042591894904616e+05,  3.510379542449001e-09,  1.195087534502225e-10, -7.212354340983467e+04, -7.432887732418586e-09, -1.123464508318517e-09},
        {2.206620987728035e-09,  1.706376869217023e+05,  2.422298678020990e-09,  3.510379542449001e-09,  2.042591894904675e+05,  4.098822911874020e-10,  5.330177387144029e-10, -7.212354340984252e+04, -3.473116223594268e-10},
        {2.049414645825843e-10, -1.276694823525542e-10,  1.297017704135240e+03,  1.195087534502225e-10,  4.098822911874020e-10,  2.226196597319318e+03, -7.938540575671655e-11,  1.745487152331077e-10, -4.864154602806899e+02},
        {-1.398705470861162e+05, -5.274688821028189e-09, -1.024139422433787e-09, -7.212354340983467e+04,  5.330177387144029e-10, -7.938540575671655e-11,  1.207954128806866e+05,  4.313878613920840e-09,  1.294242788865521e-09},
        {-9.370132654504791e-09, -1.398705470861342e+05,  1.526976924255056e-09, -7.432887732418586e-09, -7.212354340984252e+04,  1.745487152331077e-10,  4.313878613920840e-09,  1.207954128806990e+05, -1.458461076869963e-09},
        {-2.146099270809591e-09,  1.547296674362151e-09, -3.548811126035567e+03, -1.123464508318517e-09, -3.473116223594268e-10, -4.864154602806899e+02,  1.294242788865521e-09, -1.458461076869963e-09,  2.716481249150540e+03}
    };
    memcpy(param->Q_Terminal, Q_T_data, sizeof(Q_T_data));
    /*
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