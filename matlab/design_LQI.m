function lqi_data = design_LQI(p)
    % state X = [phi theta psi phi_dot theta_dot psi_dot epsilon_phi epsilon_theta epsilon_psi]
    % input u = [tau_phi tau_theta tau_psi]

    A_lqi = kron([0 1; 0 0], eye(3));
    B_lqi = [zeros(3,3);
             1/p.Ix_tilde, 0, 0;
             0, 1/p.Iy_tilde, 0;
             0, 0, 1/p.Iz_tilde];
    C_lqi = [eye(3), zeros(3)];
    D_lqi = zeros(3,3);

    sys_lqi_c = ss(A_lqi, B_lqi, C_lqi, D_lqi);
    sys_lqi_d = c2d(sys_lqi_c, 1/p.f_lqi, 'zoh');

    [K, ~, ~] = lqi(sys_lqi_d, p.Q_lqi, p.R_lqi);

    lqi_data.K = K;
end