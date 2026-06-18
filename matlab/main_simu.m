clc; close all; clear;

projectDir = fileparts(mfilename('fullpath'));
addpath(fullfile(projectDir, 'parameters'));
addpath(fullfile(projectDir, 'post_processing'));
addpath(fullfile(projectDir, 'osqp_solver'));
addpath(fullfile(projectDir, 'MahonyAHRS'));

try rmdir('slprj', 's'); catch; end

%% 1. Load Parameters
fprintf('Loading system parameters...\t\t\t\t\t');
p = load_parameters();

fprintf('[done]\n')

%% 2. Design Controllers and Estimators
fprintf('Synthesizing control and estimation matrices...\t\t\t');
lqi_data    = design_LQI(p);
kalman_data = design_Kalman(p);
mpc_data    = design_MPC(p);
mpc_data = rmfield(mpc_data, {'P_osqp', 'A_osqp'});
fprintf('[done]\n')

%% 3. Simulation Setup & Reference Generation
fprintf('Generating reference trajectory...\t\t\t\t');
T_SIM = 10; % Total simulation time (s)

% Time vector based on the fastest sensor frequency (Mahony)
t = (0 : 1/p.f_mahony : T_SIM)';
N = length(t);

% Constant reference trajectory at [3, 3, 3] meters
X_ref = ones(p.Np + 1, N) * 3;
Y_ref = ones(p.Np + 1, N) * 3;
Z_ref = ones(p.Np + 1, N) * 3;

ref_matrix = zeros(3, p.Np + 1, N);
ref_matrix(1, :, :) = X_ref;
ref_matrix(2, :, :) = Y_ref;
ref_matrix(3, :, :) = Z_ref; 

% Create timeseries object for Simulink
ref = timeseries(ref_matrix, t);

% Cleaning workspace
clear N ref_matrix t X_ref Y_ref Z_ref projectDir;

fprintf('[done]\n')

%% 4. Building osqp
build_osqp_solver();

%% 5. Run Simulink Model
fprintf('Starting Simulink simulation...\n');
simOut = sim('modele_V6', 'StopTime', num2str(T_SIM));
fprintf('Simulation finished successfully!\n');

%% 6. Plot Results
plot_sim_results(simOut, ref);

%% 7. Export Data To csv
sim_data_to_csv(simOut, 'Hex', 'Dec');

%% 8. Compare C & Matlab
compare_c_to_matlab();