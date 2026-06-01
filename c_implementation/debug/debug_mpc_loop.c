#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "param.h"
#include "motors.h"
#include "plant.h"
#include "lqi.h"
#include "mpc_wrapper.h"

int main(void) {
    Param param;
    load_param(&param);

    MotorsState  motors_state;
    motors_init(&motors_state, &param);

    LQIState lqi_state;
    lqi_init(&lqi_state);

    MPCState mpc_state;
    mpc_wrapper_init(&mpc_state);

    double X[12] = {0.0};       
    double X_dot[12] = {0.0};   
    double v[4] = {0.0};       
    double u[4] = {0.0};       
    double eta_ref[3] = {0.0}; 
    
    double X_ref_horizon[3 * (30 + 1)] = {0.0};    
    double U_mpc_optimal[3] = {0.0};         

    double dt_base = 1.0 / param.f_plant; 
    double sim_time = 0.0;
    double max_sim_time = 10.0; 
    unsigned long loop_counter = 0;

    FILE* log_file = fopen("debug/sim_log.csv", "w");
    if (log_file == NULL) {
        printf("Error : Could not create sim_log.csv\n");
        return -1;
    }
    
    fprintf(log_file, "time,ref_x,ref_y,ref_z,x,y,z,phi,theta,psi,x_dot,y_dot,z_dot,p,q,r,cmd_T,cmd_phi,cmd_theta,real_T,real_phi,real_theta,real_psi\n");

    while (sim_time < max_sim_time) {
        
        int stride = param.Np + 1;
        for (int k = 0; k <= param.Np; k++) {
            X_ref_horizon[0 * stride + k] = 3.0;
            X_ref_horizon[1 * stride + k] = 0.0; 
            X_ref_horizon[2 * stride + k] = 0.0; 
        }

        if (loop_counter % (unsigned int)(param.f_plant / param.f_mpc) == 0) {
            double X_est_mpc[6] = {X[0], X[1], X[2], X[6], X[7], X[8]};
            mpc_wrapper_step(X_est_mpc, X_ref_horizon, U_mpc_optimal, &mpc_state, &param);
        }

        if (loop_counter % (unsigned int)(param.f_plant / param.f_lqi) == 0) {
            u[0] = U_mpc_optimal[0] + (param.m * param.g); 
            eta_ref[0] = U_mpc_optimal[1]; 
            eta_ref[1] = U_mpc_optimal[2]; 
            eta_ref[2] = 0.0;              

            double tau_control[3] = {0.0};
            lqi_update(X, eta_ref, tau_control, &lqi_state, &param);

            u[1] = tau_control[0]; 
            u[2] = tau_control[1]; 
            u[3] = tau_control[2]; 
        }

        motors_update(u, v, &motors_state, &param);
        plant_update(X, v, X_dot, &param);

        fprintf(log_file, "%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                sim_time, 
                X_ref_horizon[0], X_ref_horizon[stride], X_ref_horizon[2 * stride], 
                X[0], X[1], X[2],         
                X[3], X[4], X[5],         
                X[6], X[7], X[8],         
                X[9], X[10], X[11],       
                u[0], eta_ref[0], eta_ref[1], 
                v[0], v[1], v[2], v[3]);  

        loop_counter++;
        sim_time += dt_base;
    }

    fclose(log_file);
    printf(">>> Data exported to debug/sim_log.csv !\n");
    return 0;
}