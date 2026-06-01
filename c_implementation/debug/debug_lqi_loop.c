#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "param.h"
#include "utils.h"
#include "motors.h"
#include "plant.h"
#include "lqi.h"

int main(void) {
    Param param;
    load_param(&param);

    MotorsState motors_state;
    motors_init(&motors_state, &param);

    LQIState lqi_state;
    lqi_init(&lqi_state);

    double X[12] = {0.0};       
    double X_dot[12] = {0.0};   
    double v[4] = {0.0};       
    double u[4] = {0.0};       

    double eta_ref[3] = {0.0}; 

    double dt_base = 1.0 / param.f_plant; 
    double sim_time = 0.0;
    double max_sim_time = 3.0; 
    unsigned long loop_counter = 0;

    unsigned int ratio_lqi = (unsigned int)(param.f_plant / param.f_lqi); 

    while (sim_time < max_sim_time) {
        
        if (sim_time >= 1.0) {
            eta_ref[0] = 0.1; 
        } else {
            eta_ref[0] = 0.0;
        }

        if (loop_counter % ratio_lqi == 0) {
            u[0] = param.m * param.g; 
            
            double tau_control[3] = {0.0};
            lqi_update(X, eta_ref, tau_control, &lqi_state, &param);

            u[1] = tau_control[0]; 
            u[2] = tau_control[1]; 
            u[3] = tau_control[2]; 
        }

        motors_update(u, v, &motors_state, &param);
        plant_update(X, v, X_dot, &param);

        loop_counter++;
        sim_time += dt_base;

        if (loop_counter % 20 == 0) {
            printf("Time: %.2fs | Ref Roll: %.2f | Real Roll: %.4f | Cmd Torque Roll: %.4f | Real Torque Roll: %.4f\n", 
                    sim_time, eta_ref[0], X[3], u[1], v[1]);
        }
    }

    return 0;
}