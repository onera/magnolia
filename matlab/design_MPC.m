function mpc_data = design_MPC(p)
    % 1. Plant definition (Translation Space)
    A_mpc_red = zeros(6,6);
    A_mpc_red(1:3, 4:6) = eye(3);

    B_mpc_red = zeros(6,3);
    B_mpc_red(4, 3) = p.g;         % x_dot from theta_ref
    B_mpc_red(5, 2) = -p.g;        % y_dot from phi_ref
    B_mpc_red(6, 1) = 1/p.m_tilde; % z_dot from Delta_T

    C_mpc_red = [eye(3) zeros(3,3)];

    % 2. Augmented Model (with integral tracking states)
    A_mpc = [A_mpc_red, zeros(6, 3);
             -C_mpc_red, zeros(3,3)];
             
    B_mpc = [B_mpc_red; zeros(3, 3)];
    
    C_mpc = eye(9); 
    D_mpc = zeros(9, 3);

    % Discretization
    sys_mpc_c = ss(A_mpc, B_mpc, C_mpc, D_mpc);
    sys_mpc_d = c2d(sys_mpc_c, 1/p.f_mpc, 'tustin');
    Phi = sys_mpc_d.A;
    Gamma = sys_mpc_d.B;

    % 3. Optimization Parameters (Cost Matrices)
    Q = diag(p.Q_mpc);
    R = diag(p.R_mpc);
    
    % Terminal cost
    [~, Q_T, ~] = dlqr(Phi, Gamma, Q, R);

    % 4. Build Constant OSQP Matrices (P and A_osqp)
    n = 9; % Number of augmented states
    m = 3; % Number of inputs
    Np = p.Np;
    Nc = p.Nc;

    % Build Quadratic Cost Matrix P
    Px = blkdiag(kron(eye(Np), Q), Q_T * p.Wterminale);
    W = diag(p.W_mpc);
    
    if Nc == 1
        Pu = R + W;
    else
        Pu = kron(eye(Nc), R) - kron(spdiags(ones(Nc, 1), [-1 1], Nc, Nc)*(Nc > 1), W) ...
             + blkdiag(W, kron(eye(max(0,Nc-2)), 2*W), W*(Nc >1));
    end
    P_osqp = sparse(blkdiag(Px, Pu));

    % Build Dynamic Constraint Matrix A_osqp
    Adyn_state = [kron(eye(Np), Phi), zeros(n*Np, n)] + [zeros(n*Np, n), -eye(n*Np)];
    Adyn_input = [kron(eye(Nc), Gamma); kron([zeros(Np-Nc, Nc-1) ones(Np-Nc, 1)], Gamma)];
    
    Adyn = [eye(n), zeros(n, n*Np + m*Nc);
            Adyn_state, Adyn_input];
    Abounds = eye(n*(Np+1) + m*Nc);
    A_osqp = sparse([Adyn; Abounds]);

    % Output structure
    mpc_data.Phi = Phi;
    mpc_data.Gamma = Gamma;
    mpc_data.Q_T = Q_T;
    mpc_data.P_osqp = P_osqp;
    mpc_data.A_osqp = A_osqp;
    mpc_data.n = n;
    mpc_data.m = m;
end