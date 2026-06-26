function sim_data_to_csv(simOut, full_filepath, varargin)
    p = inputParser;
    addRequired(p, 'simOut');
    addRequired(p, 'full_filepath');
    addParameter(p, 'Hex', false, @islogical);
    addParameter(p, 'Dec', false, @islogical);
    
    modifiedVarargin = varargin;
    i = 1;
    while i <= length(modifiedVarargin)
        if ischar(modifiedVarargin{i}) || isstring(modifiedVarargin{i})
            if i == length(modifiedVarargin) || ~islogical(modifiedVarargin{i+1})
                modifiedVarargin = [modifiedVarargin(1:i), {true}, modifiedVarargin(i+1:end)];
            end
        end
        i = i + 1;
    end
    if isempty(varargin)
        modifiedVarargin = {'Hex', false, 'Dec', true};
    end
    parse(p, simOut, full_filepath, modifiedVarargin{:});
    
    exportHex = p.Results.Hex;
    exportDec = p.Results.Dec;
    
    ts_list = {simOut.X, simOut.eta_hat, simOut.omega_hat, simOut.p_hat, simOut.p_dot_hat, simOut.u, simOut.v, ...
               simOut.eta_ref, simOut.X_dot, simOut.acc_meas, simOut.gyro, simOut.mocap, simOut.mag, simOut.p_ddot_hat, simOut.ref};
    max_points = 0;
    time_target = [];
    for k = 1:length(ts_list)
        if length(ts_list{k}.Time) > max_points
            max_points = length(ts_list{k}.Time);
            time_target = ts_list{k}.Time;
        end
    end
    
    ts_X_resolved         = resample(simOut.X, time_target, 'zoh');
    ts_eta_hat_resolved   = resample(simOut.eta_hat, time_target, 'zoh');
    ts_omega_hat_resolved = resample(simOut.omega_hat, time_target, 'zoh');
    ts_p_hat_resolved     = resample(simOut.p_hat, time_target, 'zoh');
    ts_p_dot_resolved     = resample(simOut.p_dot_hat, time_target, 'zoh');
    ts_u_resolved         = resample(simOut.u, time_target, 'zoh');
    ts_v_resolved         = resample(simOut.v, time_target, 'zoh');
    ts_eta_ref_resolved   = resample(simOut.eta_ref, time_target, 'zoh'); 
    ts_X_dot_resolved     = resample(simOut.X_dot, time_target, 'zoh');
    ts_acc_meas_resolved  = resample(simOut.acc_meas, time_target, 'zoh');
    ts_gyro_resolved      = resample(simOut.gyro, time_target, 'zoh');
    ts_mocap_resolved     = resample(simOut.mocap, time_target, 'zoh');
    ts_mag_resolved       = resample(simOut.mag, time_target, 'zoh');
    ts_p_ddot_hat_resolved= resample(simOut.p_ddot_hat, time_target, 'zoh');
    ts_ref_resolved       = resample(simOut.ref, time_target, 'zoh');
    
    N = length(time_target);
    time_target = time_target(:);
    
    data_X         = squeeze(ts_X_resolved.Data);           if size(data_X, 1) ~= N, data_X = data_X'; end
    data_eta_hat   = squeeze(ts_eta_hat_resolved.Data);     if size(data_eta_hat, 1) ~= N, data_eta_hat = data_eta_hat'; end
    data_omega_hat = squeeze(ts_omega_hat_resolved.Data);   if size(data_omega_hat, 1) ~= N, data_omega_hat = data_omega_hat'; end
    data_p_hat     = squeeze(ts_p_hat_resolved.Data);       if size(data_p_hat, 1) ~= N, data_p_hat = data_p_hat'; end
    data_p_dot_hat = squeeze(ts_p_dot_resolved.Data);       if size(data_p_dot_hat, 1) ~= N, data_p_dot_hat = data_p_dot_hat'; end
    data_u         = squeeze(ts_u_resolved.Data);           if size(data_u, 1) ~= N, data_u = data_u'; end
    data_v         = squeeze(ts_v_resolved.Data);           if size(data_v, 1) ~= N, data_v = data_v'; end
    data_eta_ref   = squeeze(ts_eta_ref_resolved.Data);     if size(data_eta_ref, 1) ~= N, data_eta_ref = data_eta_ref'; end
    data_X_dot      = squeeze(ts_X_dot_resolved.Data);      if size(data_X_dot, 1) ~= N, data_X_dot = data_X_dot'; end
    data_acc_meas   = squeeze(ts_acc_meas_resolved.Data);   if size(data_acc_meas, 1) ~= N, data_acc_meas = data_acc_meas'; end
    data_gyro       = squeeze(ts_gyro_resolved.Data);       if size(data_gyro, 1) ~= N, data_gyro = data_gyro'; end
    data_mocap      = squeeze(ts_mocap_resolved.Data);      if size(data_mocap, 1) ~= N, data_mocap = data_mocap'; end
    data_mag        = squeeze(ts_mag_resolved.Data);        if size(data_mag, 1) ~= N, data_mag = data_mag'; end
    data_p_ddot_hat = squeeze(ts_p_ddot_hat_resolved.Data); if size(data_p_ddot_hat, 1) ~= N, data_p_ddot_hat = data_p_ddot_hat'; end
    data_ref = squeeze(ts_ref_resolved.Data(:, 1, :));      if size(data_ref, 1) ~= N, data_ref = data_ref'; end
    
    ref_x = data_ref(:,1); ref_y = data_ref(:,2); ref_z = data_ref(:,3);
    ref_dx = data_ref(:,4); ref_dy = data_ref(:,5); ref_dz = data_ref(:,6);
    ref_phi = data_eta_ref(:,1); ref_theta = data_eta_ref(:,2); ref_psi = data_eta_ref(:,3); 
    
    data_matrix = [time_target, ref_x, ref_y, ref_z, ref_dx, ref_dy, ref_dz, ...
        data_X(:, 1:3), data_p_hat(:, 1:3), data_X(:, 4:6), data_eta_hat(:, 1:3), ...
        data_X(:, 7:9), data_p_dot_hat(:, 1:3), data_X(:, 10:12), data_omega_hat(:, 1:3), ...
        ref_phi, ref_theta, ref_psi, data_u(:, 1:4), data_v(:, 1:4), data_X_dot(:, 7:9), ...
        data_acc_meas(:, 1:3), data_gyro(:, 1:3), data_mag(:, 1:3), data_mocap(:, 1:3), data_p_ddot_hat(:, 1:3)];
    
    headers = {'time', 'ref_x', 'ref_y', 'ref_z', 'ref_dx', 'ref_dy', 'ref_dz', 'x', 'y', 'z', ...
        'est_x', 'est_y', 'est_z', 'phi', 'theta', 'psi', 'est_phi', 'est_theta', 'est_psi', ...
        'dx', 'dy', 'dz', 'est_dx', 'est_dy', 'est_dz', 'p', 'q', 'r', 'est_p', 'est_q', 'est_r', ...
        'ref_phi', 'ref_theta', 'ref_psi', 'cmd_T', 'cmd_tau_phi', 'cmd_tau_theta', 'cmd_tau_psi', ...
        'real_T', 'real_tau_phi', 'real_tau_theta', 'real_tau_psi', 'acc_x', 'acc_y', 'acc_z', ...
        'meas_acc_x', 'meas_acc_y', 'meas_acc_z', 'meas_gyro_x', 'meas_gyro_y', 'meas_gyro_z', ...
        'meas_mag_x', 'meas_mag_y', 'meas_mag_z', 'meas_mocap_x', 'meas_mocap_y', 'meas_mocap_z', ...
        'p_ddot_hat_x', 'p_ddot_hat_y', 'p_ddot_hat_z'};
    
    if exportDec
        fid = fopen(full_filepath, 'wt'); 
        if fid == -1, error('Error : Could not open CSV file'); end
        fprintf(fid, '%s\n', strjoin(headers, ','));
        if any(isnan(data_matrix), 'all'), data_matrix(isnan(data_matrix)) = 0; end
        for r = 1:size(data_matrix, 1)
            fprintf(fid, '%.4f', data_matrix(r, 1));
            for c = 2:size(data_matrix, 2)
                fprintf(fid, ',%.5g', data_matrix(r, c));
            end
            fprintf(fid, '\n');
        end
        fclose(fid);
    end
    
    if exportHex
        hex_filepath = strrep(full_filepath, '.csv', '_hex.csv');
        fid = fopen(hex_filepath, 'wt'); 
        if fid == -1, error('Error : Could not open Hex CSV file'); end
        fprintf(fid, '%s\n', strjoin(headers, ','));
        if any(isnan(data_matrix), 'all'), data_matrix(isnan(data_matrix)) = 0; end
        for r = 1:size(data_matrix, 1)
            fprintf(fid, '%.4f', data_matrix(r, 1));
            for c = 2:size(data_matrix, 2)
                val = data_matrix(r, c);
                if val == 0, c_style_hex = '0x0p+0';
                elseif isnan(val), c_style_hex = 'nan';
                elseif isinf(val)
                    if val > 0, c_style_hex = 'inf'; else, c_style_hex = '-inf'; end
                else
                    signe_str = ''; if val < 0, signe_str = '-'; val = -val; end
                    [mantissa, exponent] = log2(val); mantissa = mantissa * 2; exponent = exponent - 1;
                    frac = mantissa - 1; hex_frac = '';
                    for h_i = 1:13 
                        frac = frac * 16; digit = floor(frac);
                        hex_frac = [hex_frac, sprintf('%x', digit)]; frac = frac - digit;
                    end
                    hex_frac = regexprep(hex_frac, '0+$', '');
                    if isempty(hex_frac), c_style_hex = sprintf('%s0x1p%+d', signe_str, exponent);
                    else, c_style_hex = sprintf('%s0x1.%sp%+d', signe_str, hex_frac, exponent);
                    end
                end
                fprintf(fid, ',%s', c_style_hex);
            end
            fprintf(fid, '\n');
        end
        fclose(fid);
    end
end