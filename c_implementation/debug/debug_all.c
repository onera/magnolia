#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "param.h"
#include "motors.h"
#include "plant.h"
#include "lqi.h"
#include "gains.h"
#include "mpc_wrapper.h"
#include "../include/sensors.h" 
#include "mahony_wrapper.h"
#include "kalman.h"

int main(void) {
    Param param;
    load_param(&param);

    PlantState plant_state;
    plant_init(&plant_state, &param);

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
              
    double u[4] = {param.m * param.g, 0.0, 0.0, 0.0};          
    double eta_ref[3] = {0.0};    

    double X_ref[PARAM_NX * (PARAM_NP + 1)] = {0.0}; 
    int stride = param.Np + 1;   
    for (int k = 0; k <= param.Np; k++) {
        X_ref[0 * stride + k] = 3.0;
        X_ref[1 * stride + k] = 3.0; 
        X_ref[2 * stride + k] = 3.0; 
    }

    double gyro_meas[3] = {0.0};
    double acc_meas[3]  = {0.0};
    double mag_meas[3]  = {0.0};
    double mocap_meas[3] = {0.0};

    double X[12] = {0.0};
    double X_dot[12] = {0.0};
    double v[4] = {0.0};
    double kalman_X_hat[15] = {0.0};
    double mahony_eta_hat[3] = {0.0};
    double mahony_omega_hat[3] = {0.0};
    double lqi_tau[3] = {0.0};
    double mpc_u[4] = {0.0};
    double p_ddot_hat[3] = {0.0};
    double Y_meas[9] = {0.0};

    double buffer_u[4] = {param.m * param.g, 0.0, 0.0, 0.0};
    double buffer_eta_ref[3] = {0.0};
    double buffer_mag[3] = {0.0};
    double buffer_p_ddot_hat[3] = {0.0};

    double dt = 1.0 / param.f_plant; 
    double sim_time = 0.0;
    double max_sim_time = 10.0; 
    unsigned long loop_counter = 0;

    FILE* log_file = fopen("debug/log_c.csv", "w");
    if (log_file == NULL) {
        printf("Error : Could not create log_c.csv\n");
        return -1;
    }
    fprintf(log_file, "time,ref_x,ref_y,ref_z,x,y,z,est_x,est_y,est_z,phi,theta,psi,est_phi,est_theta,est_psi,dx,dy,dz,est_dx,est_dy,est_dz,p,q,r,est_p,est_q,est_r,ref_phi,ref_theta,ref_psi,cmd_T,cmd_tau_phi,cmd_tau_theta,cmd_tau_psi,real_T,real_tau_phi,real_tau_theta,real_tau_psi,acc_x,acc_y,acc_z,meas_acc_x,meas_acc_y,meas_acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,mocap_x,mocap_y,mocap_z,p_ddot_hat_x,p_ddot_hat_y,p_ddot_hat_z\n");

    printf(">>> Starting Simulation <<<\n");

    while (sim_time <= max_sim_time + dt) {

        motors_outputs(v, &motors_state, &param);

        plant_outputs(v, X, X_dot, &param, &plant_state);

        if (loop_counter % (unsigned int)(param.f_plant / param.f_kalman) == 0) {
            kalman_outputs(kalman_X_hat, &kalman_state);

            p_ddot_hat[0] = buffer_p_ddot_hat[0];
            p_ddot_hat[1] = buffer_p_ddot_hat[1];
            p_ddot_hat[2] = buffer_p_ddot_hat[2];
            buffer_p_ddot_hat[0] = kalman_X_hat[12];
            buffer_p_ddot_hat[1] = kalman_X_hat[13];
            buffer_p_ddot_hat[2] = kalman_X_hat[14];
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_mpc) == 0) {
            double X_hat_mpc[6] = {
                kalman_X_hat[0], kalman_X_hat[1], kalman_X_hat[2],
                kalman_X_hat[6], kalman_X_hat[7], kalman_X_hat[8]  
            };
            mpc_wrapper_outputs(X_hat_mpc, X_ref, mpc_u, &mpc_state, &param);

            eta_ref[0] = buffer_eta_ref[0];
            eta_ref[1] = buffer_eta_ref[1];
            eta_ref[2] = buffer_eta_ref[2];
            buffer_eta_ref[0] = mpc_u[1];
            buffer_eta_ref[1] = mpc_u[2];
            buffer_eta_ref[2] = 0.0; 
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_gyro) == 0) {
            gyrometer_outputs(X, gyro_meas, &param, &sensors_state);
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_acc) == 0) {
            accelerometer_outputs(X, X_dot, acc_meas, &param, &sensors_state);
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_mag) == 0) {
            mag_meas[0] = buffer_mag[0];
            mag_meas[1] = buffer_mag[1];
            mag_meas[2] = buffer_mag[2];
            magnetometer_outputs(X, buffer_mag, &param, &sensors_state);
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_MoCap) == 0) {
            mocap_outputs(mocap_meas, &param, &sensors_state);
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_mahony) == 0) {
            mahony_wrapper_outputs(gyro_meas, acc_meas, mag_meas, p_ddot_hat, mahony_eta_hat, mahony_omega_hat, &mahony_state, &param);
            for (int i = 0; i < 3; i++) {
                Y_meas[i] = mocap_meas[i];
                Y_meas[i+3] = mahony_eta_hat[i];
                Y_meas[i+6] = mahony_omega_hat[i];
            }
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_lqi) == 0) {
            lqi_outputs(mahony_eta_hat, mahony_omega_hat, eta_ref, lqi_tau, &lqi_state, &param);
            u[0] = buffer_u[0];
            u[1] = buffer_u[1];
            u[2] = buffer_u[2];
            u[3] = buffer_u[3];
            buffer_u[0] = mpc_u[0] + (param.m_tilde * param.g);
            buffer_u[1] = lqi_tau[0]; 
            buffer_u[2] = lqi_tau[1]; 
            buffer_u[3] = lqi_tau[2]; 
        }

        fprintf(log_file, "%.4f,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a\n",
                sim_time,
                X_ref[0], X_ref[stride], X_ref[2 * stride],
                X[0], X[1], X[2],                             
                kalman_X_hat[0], kalman_X_hat[1], kalman_X_hat[2], 
                X[3], X[4], X[5],                             
                mahony_eta_hat[0], mahony_eta_hat[1], mahony_eta_hat[2],           
                X[6], X[7], X[8],                             
                kalman_X_hat[6], kalman_X_hat[7], kalman_X_hat[8], 
                X[9], X[10], X[11],                           
                mahony_omega_hat[0], mahony_omega_hat[1], mahony_omega_hat[2],     
                buffer_eta_ref[0], buffer_eta_ref[1], buffer_eta_ref[2],           
                buffer_u[0], buffer_u[1], buffer_u[2], buffer_u[3],                       
                v[0], v[1], v[2], v[3],
                X_dot[6], X_dot[7], X_dot[8], 
                acc_meas[0], acc_meas[1], acc_meas[2], 
                gyro_meas[0], gyro_meas[1], gyro_meas[2], 
                buffer_mag[0], buffer_mag[1], buffer_mag[2], 
                mocap_meas[0], mocap_meas[1], mocap_meas[2], 
                buffer_p_ddot_hat[0], buffer_p_ddot_hat[1], buffer_p_ddot_hat[2]);
                
        if (loop_counter % (unsigned int)(param.f_plant / param.f_kalman) == 0) {
            kalman_update(buffer_u, Y_meas, acc_meas, &kalman_state, &param);
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_mpc) == 0) {
            double X_hat_mpc[6] = {
                kalman_X_hat[0], kalman_X_hat[1], kalman_X_hat[2],
                kalman_X_hat[6], kalman_X_hat[7], kalman_X_hat[8]  
            };
            mpc_wrapper_update(X_hat_mpc, X_ref, &mpc_state, &param);
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_lqi) == 0) {             
            double eta_hat[3] = { 
                mahony_eta_hat[0], mahony_eta_hat[1], mahony_eta_hat[2],                    
            };
            lqi_update(eta_hat, eta_ref, &lqi_state, &param);
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_MoCap) == 0) {
            mocap_update(X, &param, &sensors_state);
        }

        motors_update(u, &motors_state, &param);

        plant_update(X_dot, &param, &plant_state);

        loop_counter++;
        sim_time += dt;
    }

    fclose(log_file);
    printf(">>> Simulation finished <<<\n");
    return 0;
}