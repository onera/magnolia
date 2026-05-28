clc; close all; clear;

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
fprintf('Generating trajectory references...\t\t\t\t');
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
clear N ref_matrix t X_ref Y_ref Z_ref;

fprintf('[done]\n')

%% 4. Building osqp
%build_osqp_solver();

%% 5. Run Simulink Model
fprintf('Starting Simulink simulation (modele_V6)...\n');
simOut = sim('modele_V6', 'StopTime', num2str(T_SIM));
fprintf('Simulation finished successfully!\n');

%% 6. Plot Results

% --- Plotting trajectory ---
trajectory();

fprintf('Generating plots...\t\t\t\t\t\t');

% --- Extraction of the position reference trajectory ---
p_ref_data = squeeze(ref.Data(:, 1, :))'; 

% --- Local function to apply Scope style to legends ---
styleScopeLegend = @(hLeg) set(hLeg, 'Location', 'northwest', ...
                                    'Orientation', 'horizontal', ...
                                    'Box', 'off', ...
                                    'Color', 'none');

% =========================================================================
% DASHBOARD
% =========================================================================

function Dashboard(simOut, ref, p_ref_data, styleScopeLegend)
    figure('Name', 'Dashboard: Control & Estimation Overview', 'NumberTitle', 'off', ...
           'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.85]);
    
    % Layout style Scope
    tiledlayout(3, 2, 'TileSpacing', 'tight', 'Padding', 'compact');
    
    % 1. Position tracking
    nexttile;
    stairs(ref.Time, p_ref_data, '-', 'LineWidth', 1.0); hold on;
    stairs(simOut.p_hat.Time, simOut.p_hat.Data(:,1:3), '-', 'LineWidth', 1.2);
    plot(simOut.X.Time, simOut.X.Data(:,1:3), '-', 'LineWidth', 1.8);
    grid on; ylabel('Position [m]'); title('Position Tracking (x, y, z)');
    lgd1 = legend('Ref x', 'Ref y', 'Ref z', 'Est x', 'Est y', 'Est z', 'Real x', 'Real y', 'Real z');
    styleScopeLegend(lgd1);
    yl = ylim; ylim([yl(1), yl(2) + 0.2*diff(yl)]); 
    
    % 2. Velocity
    nexttile;
    stairs(simOut.p_dot_hat.Time, simOut.p_dot_hat.Data(:,1:3), '-', 'LineWidth', 1.2); hold on;
    plot(simOut.X_dot.Time, simOut.X_dot.Data(:,1:3), '-', 'LineWidth', 1.8);
    grid on; ylabel('Velocity [m/s]'); title('Velocity Tracking (dx, dy, dz)');
    lgd2 = legend('Est dx', 'Est dy', 'Est dz', 'Real dx', 'Real dy', 'Real dz');
    styleScopeLegend(lgd2);
    yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
    
    % 3. Attitude tracking
    nexttile;
    stairs(simOut.eta_ref.Time, simOut.eta_ref.Data, '-', 'LineWidth', 1.0); hold on;
    stairs(simOut.eta_hat.Time, simOut.eta_hat.Data(:,1:3), '-', 'LineWidth', 1.2);
    plot(simOut.X.Time, simOut.X.Data(:,4:6), '-', 'LineWidth', 1.8);
    grid on; ylabel('Attitude [rad]'); title('Attitude Tracking (\phi, \theta, \psi)');
    lgd3 = legend('Ref \phi', 'Ref \theta', 'Ref \psi', 'Est \phi', 'Est \theta', 'Est \psi', 'Real \phi', 'Real \theta', 'Real \psi');
    styleScopeLegend(lgd3);
    yl = ylim; ylim([yl(1), yl(2) + 0.2*diff(yl)]);
    
    % 4. Angular rates
    nexttile;
    stairs(simOut.omega_hat.Time, simOut.omega_hat.Data(:,1:3), '-', 'LineWidth', 1.2); hold on;
    plot(simOut.X_dot.Time, simOut.X_dot.Data(:,4:6), '-', 'LineWidth', 1.8);
    grid on; ylabel('Angular Rate [rad/s]'); title('Angular Rates (p, q, r)');
    lgd4 = legend('Est p', 'Est q', 'Est r', 'Real p', 'Real q', 'Real r');
    styleScopeLegend(lgd4);
    yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
    
    % 5. Thrust
    nexttile;
    stairs(simOut.u.Time, simOut.u.Data(:,1), '-', 'LineWidth', 1.2); hold on;
    plot(simOut.v.Time, simOut.v.Data(:,1), '-', 'LineWidth', 1.8);
    grid on; ylabel('Thrust [N]'); xlabel('Time [s]'); title('Thrust: Commanded vs Real (T vs T_m)');
    lgd5 = legend('Cmd T', 'Real T_m (Motors)');
    styleScopeLegend(lgd5);
    yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
    
    % 6. Torques
    nexttile;
    stairs(simOut.u.Time, simOut.u.Data(:,2:4), '-', 'LineWidth', 1.2); hold on;
    plot(simOut.v.Time, simOut.v.Data(:,2:4), '-', 'LineWidth', 1.8);
    grid on; ylabel('Torque [N.m]'); xlabel('Time [s]'); title('Control Torques: Commanded vs Real (\tau vs \tau_m)');
    lgd6 = legend('\tau_{\phi} cmd', '\tau_{\theta} cmd', '\tau_{\psi} cmd', '\tau_{\phi} real', '\tau_{\theta} real', '\tau_{\psi} real');
    styleScopeLegend(lgd6);
    yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
end

% =========================================================================
% FULL-SCREEN FIGURES
% =========================================================================
createFullFigure = @(name) figure('Name', name, 'NumberTitle', 'off', ...
                                  'Units', 'normalized', 'Position', [0.01 0.05 0.98 0.88]);
% Position Tracking
function Plot_Position(simOut, ref, p_ref_data, styleScopeLegend, createFullFigure)
    createFullFigure('Position Tracking');
    stairs(ref.Time, p_ref_data, '-', 'LineWidth', 1.2); hold on;
    stairs(simOut.p_hat.Time, simOut.p_hat.Data(:,1:3), '-', 'LineWidth', 1.4);
    plot(simOut.X.Time, simOut.X.Data(:,1:3), '-', 'LineWidth', 2.0);
    grid on; ylabel('Position [m]'); xlabel('Time [s]'); title('Position Tracking (x, y, z)');
    lgd_f2 = legend('Ref x', 'Ref y', 'Ref z', 'Est x', 'Est y', 'Est z', 'Real x', 'Real y', 'Real z');
    styleScopeLegend(lgd_f2); yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]); 
end

% Velocity
function Plot_Velocity(simOut, styleScopeLegend, createFullFigure)
    createFullFigure('Velocity');
    stairs(simOut.p_dot_hat.Time, simOut.p_dot_hat.Data(:,1:3), '-', 'LineWidth', 1.4); hold on;
    plot(simOut.X_dot.Time, simOut.X_dot.Data(:,1:3), '-', 'LineWidth', 2.0);
    grid on; ylabel('Velocity [m/s]'); xlabel('Time [s]'); title('Velocity (dx, dy, dz)');
    lgd_f3 = legend('Est dx', 'Est dy', 'Est dz', 'Real dx', 'Real dy', 'Real dz');
    styleScopeLegend(lgd_f3); yl = ylim; ylim([yl(1), yl(2) + 0.12*diff(yl)]);
end

% Attitude Tracking
function Plot_Attitude(simOut, styleScopeLegend, createFullFigure)
    createFullFigure('Attitude Tracking');
    stairs(simOut.eta_ref.Time, simOut.eta_ref.Data, '-', 'LineWidth', 1.2); hold on;
    stairs(simOut.eta_hat.Time, simOut.eta_hat.Data(:,1:3), '-', 'LineWidth', 1.4);
    plot(simOut.X.Time, simOut.X.Data(:,4:6), '-', 'LineWidth', 2.0);
    grid on; ylabel('Attitude [rad]'); xlabel('Time [s]'); title('Attitude Tracking (\phi, \theta, \psi)');
    lgd_f4 = legend('Ref \phi', 'Ref \theta', 'Ref \psi', 'Est \phi', 'Est \theta', 'Est \psi', 'Real \phi', 'Real \theta', 'Real \psi');
    styleScopeLegend(lgd_f4); yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
end

% Angular Rates
function Plot_Angular_Rates(simOut, styleScopeLegend, createFullFigure)
    createFullFigure('Angular Rates');
    stairs(simOut.omega_hat.Time, simOut.omega_hat.Data(:,1:3), '-', 'LineWidth', 1.4); hold on;
    plot(simOut.X_dot.Time, simOut.X_dot.Data(:,4:6), '-', 'LineWidth', 2.0);
    grid on; ylabel('Angular Rate [rad/s]'); xlabel('Time [s]'); title('Angular Rates (p, q, r)');
    lgd_f5 = legend('Est p', 'Est q', 'Est r', 'Real p', 'Real q', 'Real r');
    styleScopeLegend(lgd_f5); yl = ylim; ylim([yl(1), yl(2) + 0.12*diff(yl)]);
end

% Thrust
function Plot_Thrust(simOut, styleScopeLegend, createFullFigure)
    createFullFigure('Thrust');
    stairs(simOut.u.Time, simOut.u.Data(:,1), '-', 'LineWidth', 1.4); hold on;
    plot(simOut.v.Time, simOut.v.Data(:,1), '-', 'LineWidth', 2.0);
    grid on; ylabel('Thrust [N]'); xlabel('Time [s]'); title('Thrust: Commanded vs Real (T vs T_m)');
    lgd_f6 = legend('Cmd T', 'Real T_m (Motors)');
    styleScopeLegend(lgd_f6); yl = ylim; ylim([yl(1), yl(2) + 0.12*diff(yl)]);
end

% Torques
function Plot_Torques(simOut, styleScopeLegend, createFullFigure)
    createFullFigure('Control Torques');
    stairs(simOut.u.Time, simOut.u.Data(:,2:4), '-', 'LineWidth', 1.4); hold on;
    plot(simOut.v.Time, simOut.v.Data(:,2:4), '-', 'LineWidth', 2.0);
    grid on; ylabel('Torque [N.m]'); xlabel('Time [s]'); title('Control Torques: Commanded vs Real (\tau vs \tau_m)');
    lgd_f7 = legend('\tau_{\phi} cmd', '\tau_{\theta} cmd', '\tau_{\psi} cmd', '\tau_{\phi} real', '\tau_{\theta} real', '\tau_{\psi} real');
    styleScopeLegend(lgd_f7); yl = ylim; ylim([yl(1), yl(2) + 0.12*diff(yl)]);
end

% --- Plotting Dashboard & Full-screen ---
Dashboard(simOut, ref, p_ref_data, styleScopeLegend);
Plot_Position(simOut, ref, p_ref_data, styleScopeLegend, createFullFigure);
Plot_Velocity(simOut, styleScopeLegend, createFullFigure);
Plot_Attitude(simOut, styleScopeLegend, createFullFigure);
Plot_Angular_Rates(simOut, styleScopeLegend, createFullFigure);
Plot_Thrust(simOut, styleScopeLegend, createFullFigure);
Plot_Torques(simOut, styleScopeLegend, createFullFigure);

fprintf('[done]\n');