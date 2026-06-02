#include "mahony_wrapper.h"
#include "../lib/MahonyAHRS/MahonyAHRS.h" 
#include <math.h> 

void mahony_wrapper_init(MahonyState* state) {
    state->q0 = 1.0f; 
    state->q1 = 0.0f; 
    state->q2 = 0.0f; 
    state->q3 = 0.0f; 
    state->integralFBx = 0.0f; 
    state->integralFBy = 0.0f; 
    state->integralFBz = 0.0f; 

    for (int i = 0; i < 3; i++) { 
        state->gyro_smoothed_prev[i] = 0.0;
    }

    for (int i = 0; i < 3; i++) { 
        state->eta_hat[i] = 0.0; 
    }
}

void mahony_wrapper_outputs(double* eta_hat, double* omega_hat, MahonyState* state) {
    for (int i = 0; i < 3; i++) { 
        eta_hat[i] = state->eta_hat[i]; 
        omega_hat[i] = state->gyro_smoothed_prev[i]; 
    }
}

void mahony_wrapper_step(double* gyro_meas, double* acc_meas, double* mag_meas, 
                          double* p_ddot_hat, MahonyState* state, Param* param) {
    double pi = 3.141592653589793; 
    
    double phi   = state->eta_hat[0]; 
    double theta = state->eta_hat[1]; 
    double psi   = state->eta_hat[2]; 

    double cphi = cos(phi);   double sphi = sin(phi); 
    double ctheta = cos(theta); double stheta = sin(theta); 
    double cpsi = cos(psi);   double spsi = sin(psi); 

    double R_B[3][3] = {
        {ctheta*cpsi,  sphi*stheta*cpsi - cphi*spsi,  cphi*stheta*cpsi + sphi*spsi},
        {ctheta*spsi,  sphi*stheta*spsi + cphi*cpsi,  cphi*stheta*spsi - sphi*cpsi},
        {-stheta,      sphi*ctheta,                   cphi*ctheta}
    };

    double acc_hat_B[3] = {0.0}; 
    for(int i = 0; i < 3; i++) { 
        acc_hat_B[i] = R_B[0][i]*p_ddot_hat[0] + R_B[1][i]*p_ddot_hat[1] + R_B[2][i]*p_ddot_hat[2]; 
    }

    float ax_pure = (float)(acc_meas[0] - acc_hat_B[0]); 
    float ay_pure = (float)(acc_meas[1] - acc_hat_B[1]); 
    float az_pure = (float)(acc_meas[2] - acc_hat_B[2]); 


    q0 = state->q0; q1 = state->q1; q2 = state->q2; q3 = state->q3; 
    integralFBx = state->integralFBx;
    integralFBy = state->integralFBy; 
    integralFBz = state->integralFBz; 

    twoKp = 2.0f * (float)param->Kp_mahony; 
    twoKi = 2.0f * (float)param->Ki_mahony; 
    sampleFreq = (float)param->f_gyro; 

    MahonyAHRSupdate((float)gyro_meas[0], (float)gyro_meas[1], (float)gyro_meas[2], 
                     ax_pure, ay_pure, az_pure, 
                     (float)mag_meas[0], (float)mag_meas[1], (float)mag_meas[2]); 

    float roll, pitch, yaw; 
    getEulerAngles(&roll, &pitch, &yaw); 
    state->eta_hat[0] = (double)roll; 
    state->eta_hat[1] = (double)pitch; 
    state->eta_hat[2] = (double)yaw; 

    float bx, by, bz; 
    MahonyAHRSgetBias(&bx, &by, &bz); 

    state->q0 = q0; state->q1 = q1; state->q2 = q2; state->q3 = q3; 
    state->integralFBx = integralFBx; 
    state->integralFBy = integralFBy; 
    state->integralFBz = integralFBz; 

    double tau = 1.0 / (2.0 * pi * param->f_c_gyro); 
    double Ts = 1.0 / param->f_gyro; 
    double alpha = Ts / (tau + Ts); 

    double gyro_unbiased[3]; 
    gyro_unbiased[0] = gyro_meas[0] + (double)bx; 
    gyro_unbiased[1] = gyro_meas[1] + (double)by; 
    gyro_unbiased[2] = gyro_meas[2] + (double)bz; 

    double omega_hat[3];
    for (int i = 0; i < 3; i++) { 
        omega_hat[i] = alpha * gyro_unbiased[i] + (1.0 - alpha) * state->gyro_smoothed_prev[i]; 
        state->gyro_smoothed_prev[i] = omega_hat[i]; 
    } 
}
