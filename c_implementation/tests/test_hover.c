#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "param.h"
#include "motors.h"
#include "plant.h"

int main(void) {
    Param param;
    load_param(&param);

    MotorsState motors_state;
    motors_init(&motors_state, &param);

    double X[12] = {0.0};       
    double X_dot[12] = {0.0};   
    double v[4] = {0.0};       
    double u[4] = {0.0};       

    u[0] = param.m * param.g; 
    u[1] = 0.0;               
    u[2] = 0.0;               
    u[3] = 0.0;               

    double dt_base = 1.0 / param.f_plant; 
    double sim_time = 0.0;
    double max_sim_time = 1.0; 
    unsigned long loop_counter = 0;

    printf("====================================================\n");
    printf("        STEP 1: MOTORS & PLANT HOVER TEST           \n");
    printf("====================================================\n");

    while (sim_time < max_sim_time) {
        
        motors_update(u, v, &motors_state, &param);
        
        plant_update(X, v, X_dot, &param);

        loop_counter++;
        sim_time += dt_base;

        printf("Time: %.2fs | Cmd Thrust: %.4f | Real Wrench T: %.4f | Pos Z: %.4f | Roll: %.4f\n", 
                sim_time, u[0], v[0], X[2], X[3]);
    }

    printf("====================================================\n");
    if (fabs(X[2]) < 1e-2 && fabs(X[3]) < 1e-3) {
        printf(">>> HOVER TEST PASSED SUCCESSFULLY <<<\n");
    } else {
        printf(">>> TEST FAILED: DRIFT DETECTED <<<\n");
    }
    printf("====================================================\n");

    return 0;
}