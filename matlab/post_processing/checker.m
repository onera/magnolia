clear checker_py; clc;
py.sys.path().append('../');
checker_py = py.importlib.import_module('checker.checker');
py.importlib.reload(checker_py);

p = load_parameters();
lqi_data    = design_LQI(p);
kalman_data = design_Kalman(p);
mpc_data    = design_MPC(p);
mpc_data = rmfield(mpc_data, {'P_osqp', 'A_osqp'});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STEP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

STEP_MAGNITUDE = 3; % m
T_SIM = 20; % s
SETTLING_THRESHOLD = 5; % %

t = (0 : 1/p.f_mahony : T_SIM)';
N = length(t);

clc;

% --- Step on X ---
X_ref_step = ones(1, N) * STEP_MAGNITUDE;
ref_matrix = zeros(3, p.Np + 1, N);
ref_matrix(1, :, :) = repmat(X_ref_step, p.Np + 1, 1); 

ref = timeseries(ref_matrix, t);
simOutX = sim('modele_V6', 'StopTime', num2str(T_SIM));
fprintf("\nX step response :\n");
checker_py.step_checker(simOutX.X.Data(:,1)', simOutX.X.Time(:)', STEP_MAGNITUDE, SETTLING_THRESHOLD, 'X');

% --- Step on Y ---
Y_ref_step = ones(1, N) * STEP_MAGNITUDE;
ref_matrix = zeros(3, p.Np + 1, N);
ref_matrix(2, :, :) = repmat(Y_ref_step, p.Np + 1, 1); 

ref = timeseries(ref_matrix, t);
simOutY = sim('modele_V6', 'StopTime', num2str(T_SIM));
fprintf("\nY step response :\n");
checker_py.step_checker(simOutY.X.Data(:,2)', simOutY.X.Time(:)', STEP_MAGNITUDE, SETTLING_THRESHOLD, 'Y');

% --- Step on Z ---
Z_ref_step = ones(1, N) * STEP_MAGNITUDE;
ref_matrix = zeros(3, p.Np + 1, N);
ref_matrix(3, :, :) = repmat(Z_ref_step, p.Np + 1, 1); 

ref = timeseries(ref_matrix, t);
simOutZ = sim('modele_V6', 'StopTime', num2str(T_SIM));
fprintf("\nZ step response :\n");
checker_py.step_checker(simOutZ.X.Data(:,3)', simOutZ.X.Time(:)', STEP_MAGNITUDE, SETTLING_THRESHOLD, 'Z');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TRAJECTORY : Castle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T_SIM = 200;
t = (0 : 1/p.f_mahony : T_SIM)';
N = length(t);
t_ext = (0 : 1/p.f_mahony : T_SIM + (p.Np/p.f_mpc))'; 
N_ext = length(t_ext);

clc;

X_ext = zeros(N_ext,1); Y_ext = zeros(N_ext,1); Z_ext = zeros(N_ext,1);

for i = 1:N_ext
    ti = t_ext(i);
    
    if ti < 10,       Z_ext(i) = 0;
    elseif ti < 55,   Z_ext(i) = 0.7;
    elseif ti < 65,   Z_ext(i) = 1.2*(ti-55)/10 + 0.7*(65-ti)/10;
    elseif ti < 95,   Z_ext(i) = 1.2;
    elseif ti < 105,  Z_ext(i) = 1.2*(105-ti)/10 + 0.7*(ti-95)/10;
    elseif ti < 135,  Z_ext(i) = 0.7;
    elseif ti < 145,  Z_ext(i) = 1.2*(ti-135)/10 + 0.7*(145-ti)/10;
    elseif ti < 175,  Z_ext(i) = 1.2;
    elseif ti < 185,  Z_ext(i) = 1.2*(185-ti)/10 + 0.7*(ti-175)/10;
    else,             Z_ext(i) = 0.7;
    end
   
    if ti < 30
        X_ext(i) = 0; Y_ext(i) = 0;
    elseif ti < 50
        X_ext(i) = -(ti - 30) / 20; 
        Y_ext(i) = 0;
    elseif ti < 70
        X_ext(i) = -1; Y_ext(i) = 0;
    elseif ti < 90
        theta_c = (ti - 70) * (pi / 20);
        X_ext(i) = -1 - 0.5 * sin(theta_c);
        Y_ext(i) = -0.5 + 0.5 * cos(theta_c);
    elseif ti < 110
        X_ext(i) = -1; Y_ext(i) = -1;
    elseif ti < 130
        X_ext(i) =  -(130-ti) / 20; Y_ext(i) = -1;
    elseif ti < 150
        X_ext(i) = 0; Y_ext(i) = -1;
    elseif ti < 170
        theta_c = (ti - 150) * (pi / 20);
        X_ext(i) = 0.5 * sin(theta_c);
        Y_ext(i) = - 0.5 - 0.5 * cos(theta_c);
    else
        X_ext(i) = 0; Y_ext(i) = 0;
    end
end

ref_matrix = zeros(3, p.Np + 1, N);
dt_ratio = round(p.f_mahony / p.f_mpc);
for k = 1:N
    for step = 0:p.Np
        idx = k + step * dt_ratio;
        ref_matrix(1, step+1, k) = X_ext(idx);
        ref_matrix(2, step+1, k) = Y_ext(idx);
        ref_matrix(3, step+1, k) = Z_ext(idx);
    end
end

ref = timeseries(ref_matrix, t);
simOut = sim('modele_V6', 'StopTime', num2str(T_SIM));

fprintf("\nX following response to Castle trajectory :\n\n");
checker_py.trajectory_checker(simOut.X.Data(:,1), X_ext(1:N), 'X');
fprintf("\nY following response to Castle trajectory :\n\n");
checker_py.trajectory_checker(simOut.X.Data(:,2), Y_ext(1:N), 'Y');
fprintf("\nZ following response to Castle trajectory :\n\n");
checker_py.trajectory_checker(simOut.X.Data(:,3), Z_ext(1:N), 'Z');
trajectory();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TRAJECTORY : Lemniscate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

AMPLITUDE = 2; % m
PULSATION = pi/30; % rad/s

T_SIM = 60;
t = (0 : 1/p.f_mahony : T_SIM)';
N = length(t);
t_ext = (0 : 1/p.f_mahony : T_SIM + (p.Np/p.f_mpc))'; 
N_ext = length(t_ext);

clc;

X_ext = AMPLITUDE * sin(PULSATION*t_ext);
Y_ext = AMPLITUDE/2 * sin(PULSATION*2*t_ext);
Z_ext = zeros(N_ext,1);

ref_matrix = zeros(3, p.Np + 1, N);
for k = 1:N
    for step = 0:p.Np
        idx = k + step * dt_ratio;
        ref_matrix(1, step+1, k) = X_ext(idx);
        ref_matrix(2, step+1, k) = Y_ext(idx);
        ref_matrix(3, step+1, k) = Z_ext(idx);
    end
end

ref = timeseries(ref_matrix, t);
simOut = sim('modele_V6', 'StopTime', num2str(T_SIM));

fprintf("\nX following response to Lemniscate trajectory :\n\n");
checker_py.trajectory_checker(simOut.X.Data(:,1), X_ext(1:N), 'X');
fprintf("\nY following response to Lemniscate trajectory :\n\n");
checker_py.trajectory_checker(simOut.X.Data(:,2), Y_ext(1:N), 'Y');
fprintf("\nZ following response to Lemniscate trajectory :\n\n");
checker_py.trajectory_checker(simOut.X.Data(:,3), Z_ext(1:N), 'Z');
trajectory();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TRAJECTORY : Helix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

AMPLITUDE = 2; % m
HEIGHT = 5; % m
PULSATION = pi/30; % rad/s

T_SIM = 120;
t = (0 : 1/p.f_mahony : T_SIM)';
N = length(t);
t_ext = (0 : 1/p.f_mahony : T_SIM + (p.Np/p.f_mpc))'; 
N_ext = length(t_ext);

clc;

X_ext = AMPLITUDE * (1-cos(PULSATION*t_ext));
Y_ext = AMPLITUDE * sin(PULSATION*t_ext);
Z_ext = HEIGHT * t_ext/T_SIM;

ref_matrix = zeros(3, p.Np + 1, N);
for k = 1:N
    for step = 0:p.Np
        idx = k + step * dt_ratio;
        ref_matrix(1, step+1, k) = X_ext(idx);
        ref_matrix(2, step+1, k) = Y_ext(idx);
        ref_matrix(3, step+1, k) = Z_ext(idx);
    end
end

ref = timeseries(ref_matrix, t);
simOut = sim('modele_V6', 'StopTime', num2str(T_SIM));

fprintf("\nX following response to Helix trajectory :\n\n");
checker_py.trajectory_checker(simOut.X.Data(:,1), X_ext(1:N), 'X');
fprintf("\nY following response to Helix trajectory :\n\n");
checker_py.trajectory_checker(simOut.X.Data(:,2), Y_ext(1:N), 'Y');
fprintf("\nZ following response to Helix trajectory :\n\n");
checker_py.trajectory_checker(simOut.X.Data(:,3), Z_ext(1:N), 'Z');
trajectory();