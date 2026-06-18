#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "param.h"
#include "gains.h"
#include "motors.h"
#include "plant.h"
#include "lqi.h"

int main(void) {
    Param param;
    load_param(&param);

    PlantState plant_state;
    plant_init(&plant_state, &param);

    MotorsState motors_state;
    motors_init(&motors_state, &param);

    LQIState lqi_state;
    lqi_init(&lqi_state);

    double u[4] = {0.0};       
    double eta_ref[3] = {0.0}; 

    double X_local[12] = {0.0};
    double X_dot_local[12] = {0.0};
    double v_local[4] = {0.0};
    double lqi_tau_local[3] = {0.0};

    double dt_base = 1.0 / param.f_plant;
    double sim_time = 0.0;
    double max_sim_time = 3.0;
    unsigned long loop_counter = 0;

    unsigned int ratio_lqi = (unsigned int)(param.f_plant / param.f_lqi);

    u[0] = param.m * param.g;
    u[1] = 0.0;
    u[2] = 0.0;
    u[3] = 0.0;

    printf(">>> Starting LQI Loop Test (Advanced Telemetry Enabled) <<<\n");

    while (sim_time < max_sim_time) {

        if (sim_time >= 1.0) {
            eta_ref[0] = 0.1; 
        } else {
            eta_ref[0] = 0.0;
        }

        motors_outputs(v_local, &motors_state, &param);
        plant_update(v_local, &param, &plant_state);
        plant_outputs(X_local, X_dot_local, &plant_state);

        if (loop_counter % ratio_lqi == 0) {
            double X_for_lqi[6] = { 
                X_local[3], X_local[4], X_local[5],             
                X_local[9], X_local[10], X_local[11]            
            };
            
            lqi_update(X_for_lqi, eta_ref, &lqi_state, &param);
        }

        if (loop_counter % ratio_lqi == 0) {
            lqi_outputs(lqi_tau_local, &lqi_state, &param);

            u[0] = param.m * param.g;
            u[1] = lqi_tau_local[0]; 
            u[2] = lqi_tau_local[1];  
            u[3] = lqi_tau_local[2];  
        }

        if (loop_counter % ratio_lqi == 0 && sim_time >= 0.90 && sim_time <= 1.60) {
            printf("[TIME: %.3fs] Ref Roll: % .4f rad | Ref Pitch: % .4f rad | Real Roll: % .4f rad | Real Pitch: % .4f rad | Real Yaw: % .4f rad | p (Roll rate): % .4f rad/s | q (Pitch rate): % .4f rad/s | Err Roll: % .4f | Integrateur lqi_state->epsilon[0]: % .4f | u[0] (Coll): % .4f N | u[1] (Roll): % .4f N.m | u[2] (Pitch): % .4f N.m | Thrust: % .4f N | Tau_X: % .4f N.m | Tau_Y: % .4f N.m | M1: %.3f | M2: %.3f | M3: %.3f | M4: %.3f (N)\n", 
                sim_time, eta_ref[0], eta_ref[1], X_local[3], X_local[4], X_local[5], X_local[9], X_local[10], (eta_ref[0] - X_local[3]), lqi_state.epsilon[0], u[0], u[1], u[2], v_local[0], v_local[1], v_local[2], motors_state.Ti_v[0], motors_state.Ti_v[1], motors_state.Ti_v[2], motors_state.Ti_v[3]);
        }

        motors_update(u, &motors_state, &param);

        loop_counter++;
        sim_time += dt_base;
    }

    printf(">>> LQI Test Finished <<<\n");
    return 0;
}