clc; close all; clear;

projectDir = fileparts(mfilename('fullpath'));
addpath(fullfile(projectDir, 'parameters'));
addpath(fullfile(projectDir, 'post_processing'));
addpath(fullfile(projectDir, 'osqp_solver'));
addpath(fullfile(projectDir, 'MahonyAHRS'));

try rmdir('slprj', 's'); catch; end

function print_line()
    fprintf('%s\n', repmat('-', 1, 75));
end

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

%% 3. Building osqp
build_osqp_solver();

%% 4. Run Simulink Model
%TRAJECTORIES_TO_TEST = {'StepX', 'StepY', 'StepZ', 'Castle', 'Lemniscate', 'Helix'};
TRAJECTORIES_TO_TEST = {'StepX'};
SHIFT_SECONDS = 2.5;

fprintf('\n\nStarting Batch Simulation...\n');
print_line();

for idx_traj = 1:length(TRAJECTORIES_TO_TEST)
    TRAJ_CHOICE = TRAJECTORIES_TO_TEST{idx_traj};
    
    fprintf('Scenario [%s] - Generating reference...\n', TRAJ_CHOICE);
    [ref_shift, ref_origin, t, T_SIM] = generate_reference(p, TRAJ_CHOICE, SHIFT_SECONDS);
    ref = ref_shift;
    
    fprintf('Scenario [%s] - Running Simulink model...\n', TRAJ_CHOICE);
    assignin('base', 'ref', ref);
    assignin('base', 'T_SIM', T_SIM);
    simOut = sim('modele_V6', 'StopTime', num2str(T_SIM));

    simOut.ref = ref_origin;
    csv_filename = sprintf('../checker/matlab_%s.csv', TRAJ_CHOICE);
    fprintf('Scenario [%s] - Exporting data to %s...\n', TRAJ_CHOICE, csv_filename);
    sim_data_to_csv(simOut, csv_filename, 'Dec');
    
    print_line();
end

%% 5. Check Results

fprintf('\nLaunching Python Requirements Checker...\n');

try
    py.sys.path().append('../');
    checker_py = py.importlib.import_module('checker.checker');
    py.importlib.reload(checker_py);

    for idx_traj = 1:length(TRAJECTORIES_TO_TEST)
        TRAJ_CHOICE = TRAJECTORIES_TO_TEST{idx_traj};

        csv_filename = sprintf('../checker/matlab_%s.csv', TRAJ_CHOICE);
        full_csv_path = fullfile(projectDir, csv_filename);

        checker_py.check_requirements(py.str(full_csv_path));
    end
catch ME
    warning(ME.identifier ,'%s', ME.message);
end

%%

plot_sim_results(simOut, ref_origin, 'Attitude');