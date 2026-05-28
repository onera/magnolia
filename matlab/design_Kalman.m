function kalman_data = design_Kalman(p)
    % state X = [x y z phi theta psi x_dot y_dot z_dot p q r a_x a_y a_z]
    % input u = [Delta_T tau_phi tau_theta tau_psi]
    
    A_kalman = zeros(15, 15);
    A_kalman(1:6, 7:12) = eye(6);
    A_kalman(7:9, 13:15) = eye(3);

    B_kalman = zeros(15, 4);
    B_kalman(9, 1)  = 1/p.m_tilde;
    B_kalman(10, 2) = 1/p.Ix_tilde;
    B_kalman(11, 3) = 1/p.Iy_tilde;
    B_kalman(12, 4) = 1/p.Iz_tilde;

    C_kalman = zeros(12, 15);
    C_kalman(1:6, 1:6) = eye(6);        % Position & Attitude [x y z phi theta psi]
    C_kalman(7:9, 10:12) = eye(3);      % Vitesses angulaires [p q r]
    C_kalman(10:12, 13:15) = eye(3);    % Accelerations [a_x a_y a_z]
    C_kalman(10, 5) = -p.g; 
    C_kalman(11, 4) = p.g;

    D_kalman = zeros(12, 4);

    % Discretization
    sys_kalman_c = ss(A_kalman, B_kalman, C_kalman, D_kalman);
    sys_kalman_d = c2d(sys_kalman_c, 1/p.f_kalman, 'tustin');

    % Synthesis
    [K, ~, ~] = dlqe(sys_kalman_d.A, eye(15), sys_kalman_d.C, p.Q_kalman, p.R_kalman);

    kalman_data.K = K;
    kalman_data.sys_d = sys_kalman_d;
end