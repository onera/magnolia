#include "param.h"
#include "gains.h"

void load_param(Param* param) {
    param->g = PARAM_G;
    param->Ix = PARAM_IX; 
    param->Iy = PARAM_IY; 
    param->Iz = PARAM_IZ;
    param->m = PARAM_M;
    param->L = PARAM_L;
    param->Ti_min = PARAM_TI_MIN; 
    param->Ti_max = PARAM_TI_MAX;
    param->f_plant = PARAM_F_PLANT;

    param->Ix_tilde = PARAM_IX_TILDE; 
    param->Iy_tilde = PARAM_IY_TILDE; 
    param->Iz_tilde = PARAM_IZ_TILDE;
    param->m_tilde = PARAM_M_TILDE;
    param->L_tilde = PARAM_L_TILDE;
    param->ci = PARAM_CI;
    param->T_min = PARAM_T_MIN; 
    param->T_max = PARAM_T_MAX;

    param->f_kalman = PARAM_F_KALMAN;
    memcpy(param->A_kalman, export_A_kalman, sizeof(export_A_kalman));
    memcpy(param->B_kalman, export_B_kalman, sizeof(export_B_kalman));
    memcpy(param->C_kalman, export_C_kalman, sizeof(export_C_kalman));
    memcpy(param->K_kalman, export_K_kalman, sizeof(export_K_kalman));

    param->f_lqi = PARAM_F_LQI;
    memcpy(param->K_lqi, export_K_lqi, sizeof(export_K_lqi));

    param->f_mpc = PARAM_F_MPC;
    param->Np = PARAM_NP; 
    param->Nc = PARAM_NC;
    param->nx = PARAM_NX; 
    param->nu = PARAM_NU;
    
    memcpy(param->X_max, export_X_max, sizeof(export_X_max));
    memcpy(param->X_min, export_X_min, sizeof(export_X_min));
    memcpy(param->U_max, export_U_max, sizeof(export_U_max));
    memcpy(param->U_min, export_U_min, sizeof(export_U_min));
    memcpy(param->Q_mpc, export_Q_mpc, sizeof(export_Q_mpc));
    memcpy(param->Q_Terminal, export_Q_Terminal, sizeof(export_Q_Terminal));
    param->Wterminal = 1.0;

    param->sigma_pos = PARAM_SIGMA_POS;
    param->latency_MoCap = PARAM_LATENCY_MOCAP;
    param->f_MoCap = PARAM_F_MOCAP;

    param->sigma_omega = PARAM_SIGMA_OMEGA;
    param->b_gyro = PARAM_B_GYRO;
    param->f_c_gyro = PARAM_F_C_GYRO;
    param->f_gyro = PARAM_F_GYRO;

    param->sigma_acc = PARAM_SIGMA_ACC;
    param->b_acc = PARAM_B_ACC;
    param->f_acc = PARAM_F_ACC;

    param->m_N = PARAM_M_N; 
    param->m_E = PARAM_M_E; 
    param->m_D = PARAM_M_D;
    param->sigma_mag = PARAM_SIGMA_MAG;
    param->f_mag = PARAM_F_MAG;
    
    for(int i = 0; i < 3; i++) {
        param->B_mag[i] = export_B_mag[i][0];
        for(int j = 0; j < 3; j++) {
            param->A_mag[i][j] = export_A_mag[i][j];
        }
    }

    param->f_mahony = PARAM_F_MAHONY;
    param->Kp_mahony = PARAM_KP_MAHONY;
    param->Ki_mahony = PARAM_KI_MAHONY;

    param->tau_m = PARAM_TAU_M;
    memcpy(param->M_T_u, export_M_T_u, sizeof(export_M_T_u));
    memcpy(param->M_u_T, export_M_u_T, sizeof(export_M_u_T));
}