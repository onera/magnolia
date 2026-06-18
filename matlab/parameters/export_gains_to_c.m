function export_gains_to_c()
    fprintf('Generating gain.h...\t\t\t\t\t\t');

    p = load_parameters();
    kalman_data = design_Kalman(p);
    lqi_data = design_LQI(p);
    mpc_data = design_MPC(p); 

    output_dir = fullfile('..', 'c_implementation', 'include');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    filepath = fullfile(output_dir, 'gains.h');
    fid = fopen(filepath, 'w');
    if fid == -1
        error('Error : Could not create gains.h');
    end

    fprintf(fid, '#ifndef GAINS_H\n');
    fprintf(fid, '#define GAINS_H\n\n');
    fprintf(fid, '/* ========================================================================= */\n');
    fprintf(fid, '/* AUTOMATICALLY GENERATED FILE FROM MATLAB MAGNOLIA TOOL - DO NOT EDIT      */\n');
    fprintf(fid, '/* ========================================================================= */\n\n');

    fprintf(fid, '#define PARAM_G %.17f\n', p.g);
    fprintf(fid, '#define PARAM_IX %.17e\n', p.Ix);
    fprintf(fid, '#define PARAM_IY %.17e\n', p.Iy);
    fprintf(fid, '#define PARAM_IZ %.17e\n', p.Iz);
    fprintf(fid, '#define PARAM_M %.17f\n', p.M);
    fprintf(fid, '#define PARAM_L %.17f\n', p.L);
    fprintf(fid, '#define PARAM_TI_MIN %.17f\n', p.Ti_min);
    fprintf(fid, '#define PARAM_TI_MAX %.17f\n', p.Ti_max);
    fprintf(fid, '#define PARAM_F_PLANT %.17f\n', p.f_plant);
    fprintf(fid, '#define PARAM_IX_TILDE %.17e\n', p.Ix_tilde);
    fprintf(fid, '#define PARAM_IY_TILDE %.17e\n', p.Iy_tilde);
    fprintf(fid, '#define PARAM_IZ_TILDE %.17e\n', p.Iz_tilde);
    fprintf(fid, '#define PARAM_M_TILDE %.17f\n', p.m_tilde);
    fprintf(fid, '#define PARAM_L_TILDE %.17f\n', p.L_tilde);
    fprintf(fid, '#define PARAM_CI %.17f\n', p.ci);
    fprintf(fid, '#define PARAM_T_MIN %.17f\n', p.T_min);
    fprintf(fid, '#define PARAM_T_MAX %.17f\n\n', p.T_max);

    fprintf(fid, '#define PARAM_F_KALMAN %.17f\n', p.f_kalman);
    fprintf(fid, '#define PARAM_F_LQI %.17f\n', p.f_lqi);
    fprintf(fid, '#define PARAM_F_MPC %.17f\n', p.f_mpc);
    fprintf(fid, '#define PARAM_NP %d\n', p.Np);
    fprintf(fid, '#define PARAM_NC %d\n', p.Nc);
    fprintf(fid, '#define PARAM_NX %d\n', mpc_data.n);
    fprintf(fid, '#define PARAM_NU %d\n\n', mpc_data.m);
    fprintf(fid, '#define PARAM_W_TERMINAL %.17f\n\n', p.Wterminale);

    fprintf(fid, '#define PARAM_SIGMA_POS %.17f\n', p.sigma_p);
    fprintf(fid, '#define PARAM_LATENCY_MOCAP %.17f\n', p.latency_MoCap);
    fprintf(fid, '#define PARAM_F_MOCAP %.17f\n', p.f_MoCap);
    fprintf(fid, '#define PARAM_SIGMA_OMEGA %.17f\n', p.sigma_omega);
    fprintf(fid, '#define PARAM_B_GYRO %.17e\n', p.b_gyro);
    fprintf(fid, '#define PARAM_F_C_GYRO %.17f\n', p.f_c_gyro);
    fprintf(fid, '#define PARAM_F_GYRO %.17f\n', p.f_gyro);
    fprintf(fid, '#define PARAM_SIGMA_ACC %.17f\n', p.sigma_acc);
    fprintf(fid, '#define PARAM_B_ACC %.17f\n', p.b_acc);
    fprintf(fid, '#define PARAM_F_ACC %.17f\n', p.f_acc);
    fprintf(fid, '#define PARAM_M_N %.17f\n', p.m_N);
    fprintf(fid, '#define PARAM_M_E %.17f\n', p.m_E);
    fprintf(fid, '#define PARAM_M_D %.17f\n', p.m_D);
    fprintf(fid, '#define PARAM_SIGMA_MAG %.17f\n', p.sigma_mag);
    fprintf(fid, '#define PARAM_F_MAG %.17f\n', p.f_mag);
    fprintf(fid, '#define PARAM_F_MAHONY %.17f\n', p.f_mahony);
    fprintf(fid, '#define PARAM_KP_MAHONY %.17f\n', p.Kp_mahony);
    fprintf(fid, '#define PARAM_KI_MAHONY %.17f\n', p.Ki_mahony);
    fprintf(fid, '#define PARAM_TAU_M %.17f\n\n', p.tau_m);

    write_vector_c(fid, 'export_X_max', p.X_max);
    write_vector_c(fid, 'export_X_min', p.X_min);
    write_vector_c(fid, 'export_U_max', p.U_max);
    write_vector_c(fid, 'export_U_min', p.U_min);
    write_vector_c(fid, 'export_Q_mpc', p.Q_mpc);
    
    write_matrix_c(fid, 'export_B_mag', p.B_mag);
    write_matrix_c(fid, 'export_A_mag', p.A_mag);
    write_matrix_c(fid, 'export_M_T_u', p.M_T_u);
    write_matrix_c(fid, 'export_M_u_T', p.M_u_T);

    write_matrix_c(fid, 'export_A_kalman', kalman_data.sys_d.A);
    write_matrix_c(fid, 'export_B_kalman', kalman_data.sys_d.B);
    write_matrix_c(fid, 'export_C_kalman', kalman_data.sys_d.C);
    write_matrix_c(fid, 'export_K_kalman', kalman_data.K);
    write_matrix_c(fid, 'export_K_lqi', lqi_data.K);
    write_matrix_c(fid, 'export_Q_Terminal', mpc_data.Q_T);

    fprintf(fid, '#endif // GAINS_H\n');
    fclose(fid);
    fprintf('[done]\n');
end

function write_matrix_c(fid, var_name, matrix)
    [rows, cols] = size(matrix);
    fprintf(fid, 'static const double %s[%d][%d] = {\n', var_name, rows, cols);
    for i = 1:rows
        fprintf(fid, '    {');
        for j = 1:cols
            if j == cols
                fprintf(fid, '%.17e', matrix(i,j));
            else
                fprintf(fid, '%.17e, ', matrix(i,j));
            end
        end
        if i == rows
            fprintf(fid, '}\n');
        else
            fprintf(fid, '},\n');
        end
    end
    fprintf(fid, '};\n\n');
end

function write_vector_c(fid, var_name, vector)
    size_v = length(vector);
    fprintf(fid, 'static const double %s[%d] = {', var_name, size_v);
    for i = 1:size_v
        if i == size_v
            fprintf(fid, '%.17e', vector(i));
        else
            fprintf(fid, '%.17e, ', vector(i));
        end
    end
    fprintf(fid, '};\n\n');
end