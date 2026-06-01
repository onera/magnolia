#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "param.h"
#include "motors.h"
#include "plant.h"
#include "lqi.h"
#include "mpc_wrapper.h"
#include "../include/sensors.h" 
#include "mahony_wrapper.h"
#include "kalman.h"

extern float q0, q1, q2, q3;
extern float integralFBx, integralFBy, integralFBz;
extern float twoKp, twoKi, sampleFreq;

int main(void) {
    Param param;
    load_param(&param);

    MotorsState motors_state;
    motors_init(&motors_state, &param);

    LQIState lqi_state;
    lqi_init(&lqi_state);

    MPCState mpc_state;
    mpc_wrapper_init(&mpc_state);

    MahonyState mahony_state;
    mahony_wrapper_init(&mahony_state);

    KalmanState kalman_state;
    kalman_init(&kalman_state, &param);

    SensorsState sensors_state;
    sensors_init(&sensors_state, &param);

    double X[12] = {0.0};        
    double X_dot[12] = {0.0};    
    double v[4] = {0.0};        
    double u[4] = {0.0};          
    double eta_ref[3] = {0.0};    

    double X_ref_horizon[3 * (30 + 1)] = {0.0};    
    double U_mpc_optimal[3] = {0.0};         

    double gyro_meas[3] = {0.0};
    double acc_meas[3]  = {0.0};
    double mag_meas[3]  = {0.0};
    double mocap_meas[3] = {0.0};

    double p_ddot_hat[3] = {0.0};
    double eta_hat[3]    = {0.0}; 
    double omega_hat[3]  = {0.0}; 

    double dt_base = 1.0 / param.f_plant; 
    double sim_time = 0.0;
    double max_sim_time = 10.0; 
    unsigned long loop_counter = 0;

    FILE* log_file = fopen("debug/sim_log.csv", "w");
    if (log_file == NULL) {
        printf("Error : Could not create sim_log.csv\n");
        return -1;
    }
    fprintf(log_file, "time,ref_x,ref_y,ref_z,x,y,z,est_x,est_y,est_z,phi,theta,psi,est_phi,est_theta,est_psi,dx,dy,dz,est_dx,est_dy,est_dz,p,q,r,est_p,est_q,est_r,ref_phi,ref_theta,ref_psi,cmd_T,cmd_tau_phi,cmd_tau_theta,cmd_tau_psi,real_T,real_tau_phi,real_tau_theta,real_tau_psi\n");

    u[0] = param.m_tilde * param.g; 
    u[1] = 0.0;
    u[2] = 0.0;
    u[3] = 0.0;

    memset(kalman_state.X_hat, 0, sizeof(kalman_state.X_hat));

    printf(">>> Starting Simulation (Strict User Order) <<<\n");

    while (sim_time < max_sim_time) {

        motors_update(u, v, &motors_state, &param);

        plant_update(X, v, X_dot, &param);

        if (loop_counter % (unsigned int)(param.f_plant / param.f_mpc) == 0) {
            double X_est_mpc[6] = {
                kalman_state.X_hat[0], kalman_state.X_hat[1], kalman_state.X_hat[2],
                kalman_state.X_hat[6], kalman_state.X_hat[7], kalman_state.X_hat[8]  
            };
            int stride = param.Np + 1;
            for (int k = 0; k <= param.Np; k++) {
                X_ref_horizon[0 * stride + k] = 3.0;
                X_ref_horizon[1 * stride + k] = 0.0;
                X_ref_horizon[2 * stride + k] = 0.0;
            }
            mpc_wrapper_step(X_est_mpc, X_ref_horizon, U_mpc_optimal, &mpc_state, &param);
        }

        sensors_gyrometer(X, gyro_meas, &param);

        sensors_accelerometer(X_dot, X, acc_meas, &param);

        p_ddot_hat[0] = kalman_state.X_hat[12]; 
        p_ddot_hat[1] = kalman_state.X_hat[13]; 
        p_ddot_hat[2] = kalman_state.X_hat[14]; 
        mahony_wrapper_step(gyro_meas, acc_meas, mag_meas, p_ddot_hat, eta_hat, omega_hat, &mahony_state, &param);

        if (loop_counter % (unsigned int)(param.f_plant / param.f_lqi) == 0) {
            u[0] = U_mpc_optimal[0] + (param.m * param.g); 
            eta_ref[0] = U_mpc_optimal[1]; 
            eta_ref[1] = U_mpc_optimal[2]; 
            eta_ref[2] = 0.0;              

            double X_for_lqi[12] = {
                kalman_state.X_hat[0],  kalman_state.X_hat[1],  kalman_state.X_hat[2],  
                eta_hat[0],             eta_hat[1],             eta_hat[2],             
                kalman_state.X_hat[6],  kalman_state.X_hat[7],  kalman_state.X_hat[8],  
                omega_hat[0],           omega_hat[1],           omega_hat[2]            
            };
            double tau_control[3] = {0.0};
            lqi_update(X_for_lqi, eta_ref, tau_control, &lqi_state, &param);

            u[1] = tau_control[0]; 
            u[2] = tau_control[1]; 
            u[3] = tau_control[2]; 
        }

        if (loop_counter % (unsigned int)(param.f_plant / param.f_kalman) == 0) {
            sensors_mocap(X, mocap_meas, &sensors_state, &param);
        }

        if (loop_counter % (unsigned int)(param.f_plant / param.f_mag) == 0) {
            sensors_magnetometer(X, mag_meas, &param);
        }

        if (loop_counter % (unsigned int)(param.f_plant / param.f_kalman) == 0) {
            double Y_meas[9] = {
                mocap_meas[0], mocap_meas[1], mocap_meas[2],
                eta_hat[0],    eta_hat[1],    eta_hat[2],
                omega_hat[0],  omega_hat[1],  omega_hat[2]
            };
            kalman_update(u, Y_meas, acc_meas, kalman_state.X_hat, &kalman_state, &param);
        }

        int stride = param.Np + 1;
        fprintf(log_file, "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                sim_time, 
                X_ref_horizon[0], X_ref_horizon[stride], X_ref_horizon[2 * stride],
                X[0], X[1], X[2],                             
                kalman_state.X_hat[0], kalman_state.X_hat[1], kalman_state.X_hat[2], 
                X[3], X[4], X[5],                             
                eta_hat[0], eta_hat[1], eta_hat[2],           
                X[6], X[7], X[8],                             
                kalman_state.X_hat[6], kalman_state.X_hat[7], kalman_state.X_hat[8], 
                X[9], X[10], X[11],                           
                omega_hat[0], omega_hat[1], omega_hat[2],     
                eta_ref[0], eta_ref[1], eta_ref[2],           
                u[0], u[1], u[2], u[3],                       
                v[0], v[1], v[2], v[3]);                      
        
        loop_counter++;
        sim_time += dt_base;
    }

    fclose(log_file);
    printf(">>> Simulation finished <<<\n");
    return 0;
}