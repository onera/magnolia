function p = load_parameters()
    % -------------------------------------------------------------------------
    % 1. Physical Parameters
    p.g = 9.81; % m/s^2
    p.Ix = 2.24e-3; % kg.m^2
    p.Iy = 2.9e-3; % kg.m^2
    p.Iz = 5.3e-3; % kg.m^2
    p.I = diag([p.Ix, p.Iy, p.Iz]);
    p.M = 0.460; % kg
    p.L = 0.127; % m
    
    p.Ti_min = 0; % N
    p.Ti_max = 2.5; % N

    p.f_plant = 200;

    % -------------------------------------------------------------------------
    % 2. Estimated Model Parameters
    p.Ix_tilde = p.Ix; 
    p.Iy_tilde = p.Iy; 
    p.Iz_tilde = p.Iz; 
    p.m_tilde = p.M; 
    p.L_tilde = p.L; 
    p.ci = 0.015; 

    p.T_min = 4*p.Ti_min - p.m_tilde*p.g; % N
    p.T_max = 4*p.Ti_max - p.m_tilde*p.g; % N

    % -------------------------------------------------------------------------
    % 3. Kalman Filter Parameters
    p.f_kalman = 100; % Hz
    p.Q_kalman = diag([1e-2 1e-2 1e-2 10e-2 10e-4 10e-4 1 1 1 10e-7 10e-7 10e-7 1e5 1e5 1e5]);
    p.R_kalman = diag([1 1 1 1e-3 1e-3 1e-3 1e-4 1e-4 1e-4 1e-5 1e-5 1e-5]);

    % -------------------------------------------------------------------------
    % 4. Inner Control Loop Parameters (LQI)
    p.f_lqi = 100; % Hz
    p.Q_lqi = diag([0.5 0.5 0.5 1e-4 1e-4 1e-4 2e3 2e3 2e3]);
    p.R_lqi = diag([5e2 5e2 5e2]);

    % -------------------------------------------------------------------------
    % 5. Outer Control Loop Parameters (MPC)
    p.f_mpc = 10; % Hz
    p.Np = 30;
    p.Nc = 10;
    
    p.X_max = ones(1, 9) * 1e3; 
    p.X_min = -ones(1, 9) * 1e3;
    
    p.U_max = [p.T_max pi/9 pi/9]; 
    p.U_min = [p.T_min -pi/9 -pi/9];

    p.Q_mpc = [1 1 1 5e3 5e3 1e3 5e3 5e3 100];
    p.R_mpc = [100 1e6 1e6];
    p.W_mpc = [1 1e7 1e7];
    p.Wterminale = 1;

    % -------------------------------------------------------------------------
    % 6. Sensor Parameters

    % Motion Capture Systeme
    p.sigma_p = 0.001; 
    p.latency_MoCap = 0.01; 
    p.f_MoCap = 100; 

    % GNSS
    p.sigma_p_xy = 1; 
    p.sigma_p_z = 2.5; 
    p.latency_GNSS = 0.1; 
    p.f_GNSS = 10; 

    % Gyrometer
    p.sigma_omega = 0.01; 
    p.b_gyro = 0.02 * pi/180; 
    p.f_c_gyro = 30; 
    p.f_gyro = 200; 

    % Accelerometer
    p.sigma_acc = 0.1; 
    p.b_acc = 0.15; 
    p.f_acc = 200; 

    % Magnetometer
    p.m_N = 22.3; 
    p.m_E = 0; 
    p.m_D = 41.94; 
    p.B_mag = [0.15; -0.25; 0.10]; 
    p.A_mag = [1.005 0.002 0.001; 0.002 0.998 0.003; 0.001 0.003 1.002];
    p.sigma_mag = 0.6; 
    p.f_mag = 50; 

    % MAHONY FILTER
    p.f_mahony = 200; 
    p.Kp_mahony = 0.5;
    p.Ki_mahony = 0.01;
    
    % -------------------------------------------------------------------------
    % 7- Motors Parameters
    p.tau_m = 0.03; 
    p.M_T_u = [1 1 1 1; p.L -p.L -p.L p.L; -p.L -p.L p.L p.L; p.ci -p.ci p.ci -p.ci];
    p.M_u_T = inv(p.M_T_u);


    % NO NOISE %

    p.sigma_p   = 0.0; 
    p.sigma_omega = 0.0; 
    p.sigma_acc   = 0.0; 
    p.sigma_mag   = 0.0;
end