function [ref_shift, ref, t, T_SIM] = generate_reference(p, traj_type, shift_seconds)
    if nargin < 3
        shift_seconds = 0;
    end

    dt_ratio = round(p.f_mahony / p.f_mpc);
    dt_mahony = 1 / p.f_mahony;
    shift_idx = round(shift_seconds * p.f_mahony);

    switch traj_type
        case 'StepXYZ'
            T_SIM = 20;
            t = (0 : dt_mahony : T_SIM)';
            N = length(t);
            N_ext = N + p.Np * dt_ratio + shift_idx; 
            X_ext = zeros(N_ext,1); Y_ext = zeros(N_ext,1); Z_ext = zeros(N_ext,1);
            
            STEP_MAGNITUDE = 3;
            X_ext(:) = STEP_MAGNITUDE;
            Y_ext(:) = STEP_MAGNITUDE;
            Z_ext(:) = STEP_MAGNITUDE;

        case {'StepX', 'StepY', 'StepZ'}
            T_SIM = 10;
            t = (0 : dt_mahony : T_SIM)';
            N = length(t);
            N_ext = N + p.Np * dt_ratio + shift_idx; 
            X_ext = zeros(N_ext,1); Y_ext = zeros(N_ext,1); Z_ext = zeros(N_ext,1);
            
            STEP_MAGNITUDE = 3;
            if strcmp(traj_type, 'StepX'),     X_ext(:) = STEP_MAGNITUDE;
            elseif strcmp(traj_type, 'StepY'), Y_ext(:) = STEP_MAGNITUDE;
            else,                              Z_ext(:) = STEP_MAGNITUDE;
            end

        case 'Castle'
            T_SIM = 230;
            t = (0 : dt_mahony : T_SIM)';
            N = length(t);
            t_ext = (0 : dt_mahony : T_SIM + (p.Np/p.f_mpc) + shift_seconds)'; 
            N_ext = length(t_ext);
            
            X_ext = zeros(N_ext,1); Y_ext = zeros(N_ext,1); Z_ext = zeros(N_ext,1);
            for i = 1:N_ext
                ti = t_ext(i);
                if ti >= 30
                    t_traj = ti - 30;
                    if t_traj < 10,       Z_ext(i) = 0.7 * t_traj / 10;
                    elseif t_traj < 55,   Z_ext(i) = 0.7;
                    elseif t_traj < 65,   Z_ext(i) = 1.2*(t_traj-55)/10 + 0.7*(65-t_traj)/10;
                    elseif t_traj < 95,   Z_ext(i) = 1.2;
                    elseif t_traj < 105,  Z_ext(i) = 1.2*(105-t_traj)/10 + 0.7*(t_traj-95)/10;
                    elseif t_traj < 135,  Z_ext(i) = 0.7;
                    elseif t_traj < 145,  Z_ext(i) = 1.2*(t_traj-135)/10 + 0.7*(145-t_traj)/10;
                    elseif t_traj < 175,  Z_ext(i) = 1.2;
                    elseif t_traj < 185,  Z_ext(i) = 1.2*(185-t_traj)/10 + 0.7*(t_traj-175)/10;
                    else,                 Z_ext(i) = 0.7;
                    end
                    if t_traj < 30,       X_ext(i) = 0; Y_ext(i) = 0;
                    elseif t_traj < 50,   X_ext(i) = -(t_traj - 30) / 20; Y_ext(i) = 0;
                    elseif t_traj < 70,   X_ext(i) = -1; Y_ext(i) = 0;
                    elseif t_traj < 90
                        theta_c = (t_traj - 70) * (pi / 20);
                        X_ext(i) = -1 - 0.5 * sin(theta_c); Y_ext(i) = -0.5 + 0.5 * cos(theta_c);
                    elseif t_traj < 110,  X_ext(i) = -1; Y_ext(i) = -1;
                    elseif t_traj < 130,  X_ext(i) = -(130-t_traj) / 20; Y_ext(i) = -1;
                    elseif t_traj < 150,  X_ext(i) = 0; Y_ext(i) = -1;
                    elseif t_traj < 170
                        theta_c = (t_traj - 150) * (pi / 20);
                        X_ext(i) = 0.5 * sin(theta_c); Y_ext(i) = - 0.5 - 0.5 * cos(theta_c);
                    else,                 X_ext(i) = 0; Y_ext(i) = 0;
                    end
                end
            end

        case 'Lemniscate'
            AMPLITUDE = 2; PULSATION = pi/30; T_SIM = 90;
            t = (0 : dt_mahony : T_SIM)'; N = length(t);
            t_ext = (0 : dt_mahony : T_SIM + (p.Np/p.f_mpc) + shift_seconds)'; N_ext = length(t_ext);
            X_ext = zeros(N_ext,1); Y_ext = zeros(N_ext,1); Z_ext = zeros(N_ext,1);
            for i = 1:N_ext
                if t_ext(i) >= 30
                    t_traj = t_ext(i) - 30;
                    X_ext(i) = AMPLITUDE * sin(PULSATION*t_traj);
                    Y_ext(i) = AMPLITUDE/2 * sin(PULSATION*2*t_traj);
                end
            end

        case 'Helix'
            AMPLITUDE = 2; HEIGHT = 5; PULSATION = pi/30; T_SIM = 150; T_HELIX_ONLY = 120;
            t = (0 : dt_mahony : T_SIM)'; N = length(t);
            t_ext = (0 : dt_mahony : T_SIM + (p.Np/p.f_mpc) + shift_seconds)'; N_ext = length(t_ext);
            X_ext = zeros(N_ext,1); Y_ext = zeros(N_ext,1); Z_ext = zeros(N_ext,1);
            for i = 1:N_ext
                if t_ext(i) >= 30
                    t_traj = t_ext(i) - 30;
                    X_ext(i) = AMPLITUDE * (1 - cos(PULSATION*t_traj));
                    Y_ext(i) = AMPLITUDE * sin(PULSATION*t_traj);
                    Z_ext(i) = HEIGHT * t_traj / T_HELIX_ONLY;
                end
            end
        otherwise
            error('Unknowned trajectory ');
    end

    ref_matrix_shift = zeros(6, p.Np + 1, N);
    ref_matrix  = zeros(6, p.Np + 1, N);
    
    for k = 1:N
        for step = 0:p.Np
            idx  = k + step * dt_ratio;
            idx  = min(idx, N_ext);
            
            idx_shift = k + step * dt_ratio + shift_idx;
            idx_shift = min(idx_shift, N_ext); 
            
            ref_matrix(1, step+1, k) = X_ext(idx);
            ref_matrix(2, step+1, k) = Y_ext(idx);
            ref_matrix(3, step+1, k) = Z_ext(idx);
            
            if idx > 1 && idx < N_ext
                ref_matrix(4, step+1, k) = (X_ext(idx+1) - X_ext(idx-1)) / (2 * dt_mahony);
                ref_matrix(5, step+1, k) = (Y_ext(idx+1) - Y_ext(idx-1)) / (2 * dt_mahony);
                ref_matrix(6, step+1, k) = (Z_ext(idx+1) - Z_ext(idx-1)) / (2 * dt_mahony);
            elseif idx == 1
                ref_matrix(4, step+1, k) = (X_ext(2) - X_ext(1)) / dt_mahony;
                ref_matrix(5, step+1, k) = (Y_ext(2) - Y_ext(1)) / dt_mahony;
                ref_matrix(6, step+1, k) = (Z_ext(2) - Z_ext(1)) / dt_mahony;
            else
                ref_matrix(4, step+1, k) = (X_ext(N_ext) - X_ext(N_ext-1)) / dt_mahony;
                ref_matrix(5, step+1, k) = (Y_ext(N_ext) - Y_ext(N_ext-1)) / dt_mahony;
                ref_matrix(6, step+1, k) = (Z_ext(N_ext) - Z_ext(N_ext-1)) / dt_mahony;
            end
            
            ref_matrix_shift(1, step+1, k) = X_ext(idx_shift);
            ref_matrix_shift(2, step+1, k) = Y_ext(idx_shift);
            ref_matrix_shift(3, step+1, k) = Z_ext(idx_shift);
            
            if idx_shift > 1 && idx_shift < N_ext
                ref_matrix_shift(4, step+1, k) = (X_ext(idx_shift+1) - X_ext(idx_shift-1)) / (2 * dt_mahony);
                ref_matrix_shift(5, step+1, k) = (Y_ext(idx_shift+1) - Y_ext(idx_shift-1)) / (2 * dt_mahony);
                ref_matrix_shift(6, step+1, k) = (Z_ext(idx_shift+1) - Z_ext(idx_shift-1)) / (2 * dt_mahony);
            elseif idx_shift == 1
                ref_matrix_shift(4, step+1, k) = (X_ext(2) - X_ext(1)) / dt_mahony;
                ref_matrix_shift(5, step+1, k) = (Y_ext(2) - Y_ext(1)) / dt_mahony;
                ref_matrix_shift(6, step+1, k) = (Z_ext(2) - Z_ext(1)) / dt_mahony;
            else
                ref_matrix_shift(4, step+1, k) = (X_ext(N_ext) - X_ext(N_ext-1)) / dt_mahony;
                ref_matrix_shift(5, step+1, k) = (Y_ext(N_ext) - Y_ext(N_ext-1)) / dt_mahony;
                ref_matrix_shift(6, step+1, k) = (Z_ext(N_ext) - Z_ext(N_ext-1)) / dt_mahony;
            end
        end
    end

    ref_shift  = timeseries(ref_matrix_shift, t);
    ref = timeseries(ref_matrix, t);
end