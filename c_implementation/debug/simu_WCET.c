#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h> 

#include "param.h"
#include "motors.h"
#include "plant.h"
#include "lqi.h"
#include "gains.h"
#include "mpc_wrapper.h"
#include "../include/sensors.h" 
#include "mahony_wrapper.h"
#include "kalman.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GET_ELAPSED_US(start, end) (((double)((end) - (start))) / CLOCKS_PER_SEC * 1e6)

typedef struct {
    double x;
    double y;
    double z;
} PosRef;

PosRef generate_reference_at_index(int base_idx, const char* traj_type, double max_sim_time, double shift_seconds, const Param* p) {
    PosRef ref = {0.0, 0.0, 0.0};
    double f_m = p->f_mahony;
    double dt_mahony = 1.0 / f_m;

    int N_ext = (int)round((max_sim_time + (p->Np / p->f_mpc) + shift_seconds) * f_m);
    if (base_idx > N_ext) {
        base_idx = N_ext;
    }
    if (base_idx < 0) {
        base_idx = 0;
    }

    if (strcmp(traj_type, "StepX") == 0) {
        ref.x = 3.0;
    } else if (strcmp(traj_type, "StepY") == 0) {
        ref.y = 3.0;
    } else if (strcmp(traj_type, "StepZ") == 0) {
        ref.z = 3.0;
    } else if (strcmp(traj_type, "Castle") == 0) {
        int idx_30s = (int)round(30.0 * f_m);
        
        if (base_idx >= idx_30s) {
            int idx_traj = base_idx - idx_30s;
            double t_traj = (double)idx_traj / f_m;

            if (idx_traj < (int)round(10.0 * f_m))       ref.z = 0.7 * t_traj / 10.0;
            else if (idx_traj < (int)round(55.0 * f_m))  ref.z = 0.7;
            else if (idx_traj < (int)round(65.0 * f_m))  ref.z = 1.2 * (t_traj - 55.0) / 10.0 + 0.7 * (65.0 - t_traj) / 10.0;
            else if (idx_traj < (int)round(95.0 * f_m))  ref.z = 1.2;
            else if (idx_traj < (int)round(105.0 * f_m)) ref.z = 1.2 * (105.0 - t_traj) / 10.0 + 0.7 * (t_traj - 95.0) / 10.0;
            else if (idx_traj < (int)round(135.0 * f_m)) ref.z = 0.7;
            else if (idx_traj < (int)round(145.0 * f_m)) ref.z = 1.2 * (t_traj - 135.0) / 10.0 + 0.7 * (145.0 - t_traj) / 10.0;
            else if (idx_traj < (int)round(175.0 * f_m)) ref.z = 1.2;
            else if (idx_traj < (int)round(185.0 * f_m)) ref.z = 1.2 * (185.0 - t_traj) / 10.0 + 0.7 * (t_traj - 175.0) / 10.0;
            else                                         ref.z = 0.7;

            if (idx_traj < (int)round(30.0 * f_m))      { ref.x = 0.0; ref.y = 0.0; }
            else if (idx_traj < (int)round(50.0 * f_m)) { ref.x = -(t_traj - 30.0) / 20.0; ref.y = 0.0; }
            else if (idx_traj < (int)round(70.0 * f_m)) { ref.x = -1.0; ref.y = 0.0; }
            else if (idx_traj < (int)round(90.0 * f_m)) {
                double theta_c = (t_traj - 70.0) * (M_PI / 20.0);
                ref.x = -1.0 - 0.5 * sin(theta_c); 
                ref.y = -0.5 + 0.5 * cos(theta_c);
            }
            else if (idx_traj < (int)round(110.0 * f_m)) { ref.x = -1.0; ref.y = -1.0; }
            else if (idx_traj < (int)round(130.0 * f_m)) { ref.x = -(130.0 - t_traj) / 20.0; ref.y = -1.0; }
            else if (idx_traj < (int)round(150.0 * f_m)) { ref.x = 0.0; ref.y = -1.0; }
            else if (idx_traj < (int)round(170.0 * f_m)) {
                double theta_c = (t_traj - 150.0) * (M_PI / 20.0);
                ref.x = 0.5 * sin(theta_c); 
                ref.y = -0.5 - 0.5 * cos(theta_c);
            }
            else                                          { ref.x = 0.0; ref.y = 0.0; }
        }
    } else if (strcmp(traj_type, "Lemniscate") == 0) {
        int idx_30s = (int)round(30.0 * f_m);
        if (base_idx >= idx_30s) {
            double t_traj = (base_idx - idx_30s) * dt_mahony;
            ref.x = 2.0 * sin((M_PI / 30.0) * t_traj);
            ref.y = (2.0 / 2.0) * sin((M_PI / 30.0) * 2.0 * t_traj);
        }
    } else if (strcmp(traj_type, "Helix") == 0) {
        int idx_30s = (int)round(30.0 * f_m);
        if (base_idx >= idx_30s) {
            double t_traj = (base_idx - idx_30s) * dt_mahony;
            ref.x = 2.0 * (1.0 - cos((M_PI / 30.0) * t_traj));
            ref.y = 2.0 * sin((M_PI / 30.0) * t_traj);
            ref.z = 5.0 * t_traj / 120.0;
        }
    }
    return ref;
}

void get_mpc_prediction_horizon(unsigned long loop_counter, double shift_seconds, const char* traj_type, const Param* p, double* X_ref, double max_sim_time) {
    int stride = p->Np + 1;
    double dt_mahony = 1.0 / p->f_mahony;

    int dt_ratio = (int)round(p->f_mahony / p->f_mpc); 
    int shift_idx = (int)round(shift_seconds * p->f_mahony);

    int current_k_minus_1 = (int)loop_counter + 1;

    int N_ext = (int)round((max_sim_time + ((double)p->Np / p->f_mpc) + shift_seconds) * p->f_mahony) + 1; 

    for (int step = 0; step <= p->Np; step++) {
        int base_idx = current_k_minus_1 + step * dt_ratio + shift_idx;
        int matlab_idx = base_idx + 1; 

        PosRef p_curr = generate_reference_at_index(base_idx, traj_type, max_sim_time, shift_seconds, p);

        X_ref[0 * stride + step] = p_curr.x;
        X_ref[1 * stride + step] = p_curr.y;
        X_ref[2 * stride + step] = p_curr.z;

        if (matlab_idx > 1 && matlab_idx < N_ext) {
            PosRef p_next = generate_reference_at_index(base_idx + 1, traj_type, max_sim_time, shift_seconds, p);
            PosRef p_prev = generate_reference_at_index(base_idx - 1, traj_type, max_sim_time, shift_seconds, p);

            X_ref[3 * stride + step] = (p_next.x - p_prev.x) / (2.0 * dt_mahony);
            X_ref[4 * stride + step] = (p_next.y - p_prev.y) / (2.0 * dt_mahony);
            X_ref[5 * stride + step] = (p_next.z - p_prev.z) / (2.0 * dt_mahony);
        } 
        else if (matlab_idx == 1) {
            PosRef p_next = generate_reference_at_index(base_idx + 1, traj_type, max_sim_time, shift_seconds, p);

            X_ref[3 * stride + step] = (p_next.x - p_curr.x) / dt_mahony;
            X_ref[4 * stride + step] = (p_next.y - p_curr.y) / dt_mahony;
            X_ref[5 * stride + step] = (p_next.z - p_curr.z) / dt_mahony;
        }
        else {
            PosRef p_prev = generate_reference_at_index(base_idx - 1, traj_type, max_sim_time, shift_seconds, p);

            X_ref[3 * stride + step] = (p_curr.x - p_prev.x) / dt_mahony;
            X_ref[4 * stride + step] = (p_curr.y - p_prev.y) / dt_mahony;
            X_ref[5 * stride + step] = (p_curr.z - p_prev.z) / dt_mahony;
        }
    }
}

int run_simulation(const char* traj_type, const char* log_filename, const char* timing_filename, double max_sim_time, double shift_seconds, const char* log_format) {
    Param param;
    load_param(&param);

    PlantState plant_state; plant_init(&plant_state, &param);
    MotorsState motors_state; motors_init(&motors_state, &param);
    LQIState lqi_state; lqi_init(&lqi_state);
    MPCState mpc_state; mpc_wrapper_init(&mpc_state);
    MahonyState mahony_state; mahony_wrapper_init(&mahony_state);
    KalmanState kalman_state; kalman_init(&kalman_state, &param);
    SensorsState sensors_state; sensors_init(&sensors_state, &param);
               
    double u[4] = {param.m * param.g, 0.0, 0.0, 0.0};          
    double eta_ref[3] = {0.0};    
    double X_ref[PARAM_NX * (PARAM_NP + 1)] = {0.0}; 

    double gyro_meas[3] = {0.0}, acc_meas[3] = {0.0}, mag_meas[3] = {0.0}, mocap_meas[3] = {0.0};
    double X[12] = {0.0}, X_dot[12] = {0.0}, v[4] = {0.0}; double X_hat_mpc[6] = {0.0};
    double kalman_X_hat[15] = {0.0}, mahony_eta_hat[3] = {0.0}, mahony_omega_hat[3] = {0.0};
    double lqi_tau[3] = {0.0}, mpc_u[4] = {0.0}, p_ddot_hat[3] = {0.0}, Y_meas[9] = {0.0};

    double buffer_u[4] = {param.m * param.g, 0.0, 0.0, 0.0};
    double buffer_eta_ref[3] = {0.0}, buffer_mag[3] = {0.0}, buffer_p_ddot_hat[3] = {0.0};

    double dt = 1.0 / param.f_plant; 
    double sim_time = 0.0;
    unsigned long loop_counter = 0;

    FILE* log_file = fopen(log_filename, "w");
    FILE* timing_file = fopen(timing_filename, "w");
    if (log_file == NULL || timing_file == NULL) {
        printf("[ERROR] Could not create files %s or %s\n", log_filename, timing_filename);
        if (log_file) fclose(log_file);
        if (timing_file) fclose(timing_file);
        return -1;
    }
    
    fprintf(log_file, "time,ref_x,ref_y,ref_z,ref_dx,ref_dy,ref_dz,x,y,z,est_x,est_y,est_z,phi,theta,psi,est_phi,est_theta,est_psi,dx,dy,dz,est_dx,est_dy,est_dz,p,q,r,est_p,est_q,est_r,ref_phi,ref_theta,ref_psi,cmd_T,cmd_tau_phi,cmd_tau_theta,cmd_tau_psi,real_T,real_tau_phi,real_tau_theta,real_tau_psi,acc_x,acc_y,acc_z,meas_acc_x,meas_acc_y,meas_acc_z,meas_gyro_x,meas_gyro_y,meas_gyro_z,meas_mag_x,meas_mag_y,meas_mag_z,meas_mocap_x,meas_mocap_y,meas_mocap_z,p_ddot_hat_x,p_ddot_hat_y,p_ddot_hat_z\n");
    fprintf(timing_file, "time,out_motors,out_plant,out_kalman,out_mpc,out_sensors,out_mahony,out_lqi,up_kalman,up_mpc,up_lqi,up_mocap,up_motors,up_plant\n");
    
    const char* fmt_hex = "%.4f,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a\n";
    const char* fmt_dec = "%.4f,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g\n";

    const char* row_format = (strcmp(log_format, "HEX") == 0) ? fmt_hex : fmt_dec;

    printf("Executing Scenario [%s] -> Saving data to: %s [%s]...\n", traj_type, log_filename, log_format);
    fflush(stdout);

    clock_t start, end;
    double t_out_motors, t_out_plant, t_out_kalman, t_out_mpc, t_out_sensors, t_out_mahony, t_out_lqi;
    double t_up_kalman, t_up_mpc, t_up_lqi, t_up_mocap, t_up_motors, t_up_plant;

    while (sim_time <= max_sim_time + 1.0e-3) {
        t_out_motors = t_out_plant = t_out_kalman = t_out_mpc = t_out_sensors = t_out_mahony = t_out_lqi = -1.0;
        t_up_kalman = t_up_mpc = t_up_lqi = t_up_mocap = t_up_motors = t_up_plant = -1.0;

        start = clock(); motors_outputs(v, &motors_state, &param); end = clock();
        t_out_motors = GET_ELAPSED_US(start, end);

        start = clock(); plant_outputs(v, X, X_dot, &param, &plant_state); end = clock();
        t_out_plant = GET_ELAPSED_US(start, end);

        if (loop_counter % (unsigned int)(param.f_plant / param.f_kalman) == 0) {
            start = clock();
            kalman_outputs(kalman_X_hat, &kalman_state);
            p_ddot_hat[0] = buffer_p_ddot_hat[0]; p_ddot_hat[1] = buffer_p_ddot_hat[1]; p_ddot_hat[2] = buffer_p_ddot_hat[2];
            buffer_p_ddot_hat[0] = kalman_X_hat[12]; buffer_p_ddot_hat[1] = kalman_X_hat[13]; buffer_p_ddot_hat[2] = kalman_X_hat[14];
            end = clock(); t_out_kalman = GET_ELAPSED_US(start, end);
        }
        
        if (loop_counter % (unsigned int)(param.f_plant / param.f_mpc) == 0) {
            start = clock();
            for (int i = 0; i < 3; i++) {
                X_hat_mpc[i] = kalman_X_hat[i];
                X_hat_mpc[i+3] = kalman_X_hat[i+6];
            }
            get_mpc_prediction_horizon(loop_counter, shift_seconds, traj_type, &param, X_ref, max_sim_time);
            mpc_wrapper_outputs(X_hat_mpc, X_ref, mpc_u, &mpc_state, &param);

            eta_ref[0] = buffer_eta_ref[0]; eta_ref[1] = buffer_eta_ref[1]; eta_ref[2] = buffer_eta_ref[2];
            buffer_eta_ref[0] = mpc_u[1]; buffer_eta_ref[1] = mpc_u[2]; buffer_eta_ref[2] = 0.0; 
            end = clock(); t_out_mpc = GET_ELAPSED_US(start, end);
        }

        start = clock();
        if (loop_counter % (unsigned int)(param.f_plant / param.f_gyro) == 0)   gyrometer_outputs(X, gyro_meas, &param, &sensors_state);
        if (loop_counter % (unsigned int)(param.f_plant / param.f_acc) == 0)    accelerometer_outputs(X, X_dot, acc_meas, &param, &sensors_state);
        if (loop_counter % (unsigned int)(param.f_plant / param.f_mag) == 0) {
            mag_meas[0] = buffer_mag[0]; mag_meas[1] = buffer_mag[1]; mag_meas[2] = buffer_mag[2];
            magnetometer_outputs(X, buffer_mag, &param, &sensors_state);
        }
        if (loop_counter % (unsigned int)(param.f_plant / param.f_MoCap) == 0) {
            mocap_outputs(mocap_meas, &param, &sensors_state);
        }
        end = clock(); t_out_sensors = GET_ELAPSED_US(start, end);

        if (loop_counter % (unsigned int)(param.f_plant / param.f_mahony) == 0) {
            start = clock();
            mahony_wrapper_outputs(gyro_meas, acc_meas, mag_meas, p_ddot_hat, mahony_eta_hat, mahony_omega_hat, &mahony_state, &param);
            for (int i = 0; i < 3; i++) {
                Y_meas[i] = mocap_meas[i]; Y_meas[i+3] = mahony_eta_hat[i]; Y_meas[i+6] = mahony_omega_hat[i];
            }
            end = clock(); t_out_mahony = GET_ELAPSED_US(start, end);
        }

        if (loop_counter % (unsigned int)(param.f_plant / param.f_lqi) == 0) {
            start = clock();
            lqi_outputs(mahony_eta_hat, mahony_omega_hat, eta_ref, lqi_tau, &lqi_state, &param);
            u[0] = buffer_u[0]; u[1] = buffer_u[1]; u[2] = buffer_u[2]; u[3] = buffer_u[3];
            buffer_u[0] = mpc_u[0] + (param.m_tilde * param.g);
            buffer_u[1] = lqi_tau[0]; buffer_u[2] = lqi_tau[1]; buffer_u[3] = lqi_tau[2]; 
            end = clock(); t_out_lqi = GET_ELAPSED_US(start, end);
        }

        double dt_m = 1.0 / param.f_mahony;

        int k_log = (int)loop_counter; 

        PosRef current_true_ref = generate_reference_at_index(k_log, traj_type, max_sim_time, shift_seconds, &param);
        PosRef next_true_ref    = generate_reference_at_index(k_log + 1, traj_type, max_sim_time, shift_seconds, &param);
        PosRef prev_true_ref    = generate_reference_at_index(k_log - 1, traj_type, max_sim_time, shift_seconds, &param);

        double ref_dx = (next_true_ref.x - prev_true_ref.x) / (2.0 * dt_m);
        double ref_dy = (next_true_ref.y - prev_true_ref.y) / (2.0 * dt_m);
        double ref_dz = (next_true_ref.z - prev_true_ref.z) / (2.0 * dt_m);

        fprintf(log_file, row_format,
                sim_time, current_true_ref.x, current_true_ref.y, current_true_ref.z, ref_dx, ref_dy, ref_dz,                                     
                X[0], X[1], X[2], kalman_X_hat[0], kalman_X_hat[1], kalman_X_hat[2], 
                X[3], X[4], X[5], mahony_eta_hat[0], mahony_eta_hat[1], mahony_eta_hat[2],           
                X[6], X[7], X[8], kalman_X_hat[6], kalman_X_hat[7], kalman_X_hat[8], 
                X[9], X[10], X[11], mahony_omega_hat[0], mahony_omega_hat[1], mahony_omega_hat[2],     
                buffer_eta_ref[0], buffer_eta_ref[1], buffer_eta_ref[2],           
                buffer_u[0], buffer_u[1], buffer_u[2], buffer_u[3], v[0], v[1], v[2], v[3],
                X_dot[6], X_dot[7], X_dot[8], acc_meas[0], acc_meas[1], acc_meas[2], 
                gyro_meas[0], gyro_meas[1], gyro_meas[2], buffer_mag[0], buffer_mag[1], buffer_mag[2], 
                mocap_meas[0], mocap_meas[1], mocap_meas[2], buffer_p_ddot_hat[0], buffer_p_ddot_hat[1], buffer_p_ddot_hat[2]);
                
        if (loop_counter % (unsigned int)(param.f_plant / param.f_kalman) == 0) {
            start = clock(); 
            kalman_update(buffer_u, Y_meas, acc_meas, &kalman_state, &param); 
            end = clock(); t_up_kalman = GET_ELAPSED_US(start, end);
        }

        if (loop_counter % (unsigned int)(param.f_plant / param.f_mpc) == 0) {
            start = clock();
            for (int i = 0; i < 3; i++) {
                X_hat_mpc[i] = kalman_X_hat[i];
                X_hat_mpc[i+3] = kalman_X_hat[i+6];
            }
            mpc_wrapper_update(X_hat_mpc, X_ref, &mpc_state, &param);
            end = clock(); t_up_mpc = GET_ELAPSED_US(start, end);
        } 

        if (loop_counter % (unsigned int)(param.f_plant / param.f_lqi) == 0) {            
            start = clock(); 
            double eta_hat[3] = { mahony_eta_hat[0], mahony_eta_hat[1], mahony_eta_hat[2] };
            lqi_update(eta_hat, eta_ref, &lqi_state, &param); 
            end = clock(); t_up_lqi = GET_ELAPSED_US(start, end);
        }

        if (loop_counter % (unsigned int)(param.f_plant / param.f_MoCap) == 0) {
            start = clock(); 
            mocap_update(X, &param, &sensors_state); 
            end = clock(); t_up_mocap = GET_ELAPSED_US(start, end);
        }

        start = clock(); 
        motors_update(u, &motors_state, &param); 
        end = clock(); t_up_motors = GET_ELAPSED_US(start, end);

        start = clock(); 
        plant_update(X_dot, &param, &plant_state); 
        end = clock(); t_up_plant = GET_ELAPSED_US(start, end);

        fprintf(timing_file, "%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                sim_time, t_out_motors, t_out_plant, t_out_kalman, t_out_mpc, t_out_sensors, t_out_mahony, t_out_lqi,
                t_up_kalman, t_up_mpc, t_up_lqi, t_up_mocap, t_up_motors, t_up_plant);

        loop_counter++;
        sim_time = loop_counter * dt; 
    }

    fclose(log_file);
    fclose(timing_file);
    return 0;
}

int main(void) {
    const char* trajectories[] = {"StepX", "Castle"}; // StepX StepY StepZ Castle Lemniscate Helix
    double run_times[] = {20.0, 230.0};
    int num_scenarios = sizeof(trajectories) / sizeof(trajectories[0]);
    
    double SHIFT_SECONDS = 2.5;
    const char* FORMAT_FLAG = "HEX"; // HEX or DEC

    printf("====================================================\n");
    printf("       STARTING EMBEDDED C BATCH SIMULATION         \n");
    printf("====================================================\n");

    for (int i = 0; i < num_scenarios; i++) {
        char out_filename[256];
        char timing_filename[256];
        
        if (strcmp(FORMAT_FLAG, "HEX") == 0) {
            sprintf(out_filename, "../checker/c_%s_hex.csv", trajectories[i]);
        } else {
            sprintf(out_filename, "../checker/c_%s.csv", trajectories[i]);
        }
        
        sprintf(timing_filename, "./debug/timing_%s.csv", trajectories[i]);
        
        run_simulation(trajectories[i], out_filename, timing_filename, run_times[i], SHIFT_SECONDS, FORMAT_FLAG);
    }

    printf("====================================================\n");
    printf("           BATCH SIMULATIONS COMPLETED !            \n");
    printf("====================================================\n");
    
    return 0;
}