function plot_sim_results(simOut, ref, varargin)
    p = inputParser;
    addRequired(p, 'simOut');
    addRequired(p, 'ref');
    addParameter(p, 'Dashboard', false, @islogical);
    addParameter(p, 'Trajectory', false, @islogical);
    addParameter(p, 'Position', false, @islogical);
    addParameter(p, 'Velocity', false, @islogical);
    addParameter(p, 'Attitude', false, @islogical);
    addParameter(p, 'AngularRate', false, @islogical);
    addParameter(p, 'Thrust', false, @islogical);
    addParameter(p, 'Torques', false, @islogical);
    
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
        modifiedVarargin = {'Dashboard', true, 'Trajectory', true, 'Position', true, 'Velocity', true, 'Attitude', true, 'AngularRate', true, 'Thrust', true, 'Torques', true};
    end
    
    parse(p, simOut, ref, modifiedVarargin{:});
    
    fprintf('Generating plots...\t\t\t\t\t\t');
    p_ref_data = squeeze(ref.Data(1:3, 1, :))'; 
    v_ref_data = squeeze(ref.Data(4:6, 1, :))'; % Extraction des vitesses de référence
    
    if p.Results.Dashboard, Dashboard(simOut, ref, p_ref_data, v_ref_data); end
    if p.Results.Trajectory, Plot_3D_Trajectory(simOut, p_ref_data); end
    if p.Results.Position,       Plot_Position(simOut, ref, p_ref_data); end
    if p.Results.Velocity,       Plot_Velocity(simOut, ref, v_ref_data); end % Passage des vitesses de référence
    if p.Results.Attitude,       Plot_Attitude(simOut); end
    if p.Results.AngularRate,       Plot_Angular_Rates(simOut); end
    if p.Results.Thrust,    Plot_Thrust(simOut); end
    if p.Results.Torques,   Plot_Torques(simOut); end
    
    fprintf('[done]\n');
end

function styleScopeLegend(lgd, fontsize)
    set(lgd, 'Location', 'northwest', ...      
              'Orientation', 'horizontal', ...   
              'Box', 'off', ...                
              'Color', 'none', ...               
              'FontSize', fontsize, ...              
              'FontWeight', 'bold')
    set(lgd.Axes, 'FontSize', fontsize) 
    set(findobj(lgd.Axes, 'Type', 'line'), 'LineWidth', 2.2) 
end

function fig = createFullFigure(name)
    fig = figure('Name', name, 'NumberTitle', 'off', ...
                 'Units', 'normalized', 'Position', [0.01 0.05 0.98 0.88]);
end

% =========================================================================
% DASHBOARD
% =========================================================================
function Dashboard(simOut, ref, p_ref_data, v_ref_data)
    figure('Name', 'Dashboard: Control & Estimation Overview', 'NumberTitle', 'off', ...
           'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.85]);
    
    tiledlayout(3, 2, 'TileSpacing', 'tight', 'Padding', 'compact');
    
    % 1. Position tracking
    nexttile;
    stairs(ref.Time, p_ref_data, '-', 'LineWidth', 1.0); hold on;
    stairs(simOut.p_hat.Time, simOut.p_hat.Data(:,1:3), '-', 'LineWidth', 1.2);
    plot(simOut.X.Time, simOut.X.Data(:,1:3), '-', 'LineWidth', 1.8);
    grid on; ylabel('Position [m]'); title('Position Tracking (x, y, z)');
    lgd1 = legend('Ref x', 'Ref y', 'Ref z', 'Est x', 'Est y', 'Est z', 'Real x', 'Real y', 'Real z');
    styleScopeLegend(lgd1, 10);
    yl = ylim; ylim([yl(1), yl(2) + 0.2*diff(yl)]); 
    
    % 2. Velocity (Avec Référence)
    nexttile;
    stairs(ref.Time, v_ref_data, '-', 'LineWidth', 1.0); hold on; % Ajout de la ref vitesse
    stairs(simOut.p_dot_hat.Time, simOut.p_dot_hat.Data(:,1:3), '-', 'LineWidth', 1.2);
    plot(simOut.X_dot.Time, simOut.X_dot.Data(:,1:3), '-', 'LineWidth', 1.8);
    grid on; ylabel('Velocity [m/s]'); title('Velocity Tracking (dx, dy, dz)');
    lgd2 = legend('Ref dx', 'Ref dy', 'Ref dz', 'Est dx', 'Est dy', 'Est dz', 'Real dx', 'Real dy', 'Real dz');
    styleScopeLegend(lgd2, 10);
    yl = ylim; ylim([yl(1), yl(2) + 0.2*diff(yl)]);
    
    % 3. Attitude tracking
    nexttile;
    stairs(simOut.eta_ref.Time, simOut.eta_ref.Data, '-', 'LineWidth', 1.0); hold on;
    stairs(simOut.eta_hat.Time, simOut.eta_hat.Data(:,1:3), '-', 'LineWidth', 1.2);
    plot(simOut.X.Time, simOut.X.Data(:,4:6), '-', 'LineWidth', 1.8);
    grid on; ylabel('Attitude [rad]'); title('Attitude Tracking (\phi, \theta, \psi)');
    lgd3 = legend('Ref \phi', 'Ref \theta', 'Ref \psi', 'Est \phi', 'Est \theta', 'Est \psi', 'Real \phi', 'Real \theta', 'Real \psi');
    styleScopeLegend(lgd3, 10);
    yl = ylim; ylim([yl(1), yl(2) + 0.2*diff(yl)]);
    
    % 4. Angular rates
    nexttile;
    stairs(simOut.omega_hat.Time, simOut.omega_hat.Data(:,1:3), '-', 'LineWidth', 1.2); hold on;
    plot(simOut.X_dot.Time, simOut.X_dot.Data(:,4:6), '-', 'LineWidth', 1.8);
    grid on; ylabel('Angular Rate [rad/s]'); title('Angular Rates (p, q, r)');
    lgd4 = legend('Est p', 'Est q', 'Est r', 'Real p', 'Real q', 'Real r');
    styleScopeLegend(lgd4, 10);
    yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
    
    % 5. Thrust
    nexttile;
    stairs(simOut.u.Time, simOut.u.Data(:,1), '-', 'LineWidth', 1.2); hold on;
    plot(simOut.v.Time, simOut.v.Data(:,1), '-', 'LineWidth', 1.8);
    grid on; ylabel('Thrust [N]'); xlabel('Time [s]'); title('Thrust: Commanded vs Real (T vs T_m)');
    lgd5 = legend('Cmd T', 'Real T_m (Motors)');
    styleScopeLegend(lgd5, 10);
    yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
    
    % 6. Torques
    nexttile;
    stairs(simOut.u.Time, simOut.u.Data(:,2:4), '-', 'LineWidth', 1.2); hold on;
    plot(simOut.v.Time, simOut.v.Data(:,2:4), '-', 'LineWidth', 1.8);
    grid on; ylabel('Torque [N.m]'); xlabel('Time [s]'); title('Control Torques: Commanded vs Real (\tau vs \tau_m)');
    lgd6 = legend('\tau_{\phi} cmd', '\tau_{\theta} cmd', '\tau_{\psi} cmd', '\tau_{\phi} real', '\tau_{\theta} real', '\tau_{\psi} real');
    styleScopeLegend(lgd6, 10);
    yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
end

% =========================================================================
% FULL SCREEN PLOTS
% =========================================================================
function Plot_Position(simOut, ref, p_ref_data)
    createFullFigure('Position Tracking');
    stairs(ref.Time, p_ref_data, '-', 'LineWidth', 1.2); hold on;
    stairs(simOut.p_hat.Time, simOut.p_hat.Data(:,1:3), '-', 'LineWidth', 1.4);
    plot(simOut.X.Time, simOut.X.Data(:,1:3), '-', 'LineWidth', 2.0);
    grid on; ylabel('Position [m]'); xlabel('Time [s]'); title('Position Tracking (x, y, z)');
    lgd_f2 = legend('Ref x', 'Ref y', 'Ref z', 'Est x', 'Est y', 'Est z', 'Real x', 'Real y', 'Real z');
    styleScopeLegend(lgd_f2, 15); yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]); 
end

function Plot_Velocity(simOut, ref, v_ref_data)
    createFullFigure('Velocity');
    stairs(ref.Time, v_ref_data, '-', 'LineWidth', 1.2); hold on; % Ajout de la ref vitesse
    stairs(simOut.p_dot_hat.Time, simOut.p_dot_hat.Data(:,1:3), '-', 'LineWidth', 1.4);
    plot(simOut.X_dot.Time, simOut.X_dot.Data(:,1:3), '-', 'LineWidth', 2.0);
    grid on; ylabel('Velocity [m/s]'); xlabel('Time [s]'); title('Velocity (dx, dy, dz)');
    lgd_f3 = legend('Ref dx', 'Ref dy', 'Ref dz', 'Est dx', 'Est dy', 'Est dz', 'Real dx', 'Real dy', 'Real dz');
    styleScopeLegend(lgd_f3, 15); yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
end

function Plot_Attitude(simOut)
    createFullFigure('Attitude Tracking');
    stairs(simOut.eta_ref.Time, simOut.eta_ref.Data, '-', 'LineWidth', 1.2); hold on;
    stairs(simOut.eta_hat.Time, simOut.eta_hat.Data(:,1:3), '-', 'LineWidth', 1.4);
    plot(simOut.X.Time, simOut.X.Data(:,4:6), '-', 'LineWidth', 2.0);
    grid on; ylabel('Attitude [rad]'); xlabel('Time [s]'); title('Attitude Tracking (\phi, \theta, \psi)');
    lgd_f4 = legend('Ref \phi', 'Ref \theta', 'Ref \psi', 'Est \phi', 'Est \theta', 'Est \psi', 'Real \phi', 'Real \theta', 'Real \psi');
    styleScopeLegend(lgd_f4, 15); yl = ylim; ylim([yl(1), yl(2) + 0.15*diff(yl)]);
end

function Plot_Angular_Rates(simOut)
    createFullFigure('Angular Rates');
    stairs(simOut.omega_hat.Time, simOut.omega_hat.Data(:,1:3), '-', 'LineWidth', 1.4); hold on;
    plot(simOut.X_dot.Time, simOut.X_dot.Data(:,4:6), '-', 'LineWidth', 2.0);
    grid on; ylabel('Angular Rate [rad/s]'); xlabel('Time [s]'); title('Angular Rates (p, q, r)');
    lgd_f5 = legend('Est p', 'Est q', 'Est r', 'Real p', 'Real q', 'Real r');
    styleScopeLegend(lgd_f5, 15); yl = ylim; ylim([yl(1), yl(2) + 0.12*diff(yl)]);
end

function Plot_Thrust(simOut)
    createFullFigure('Thrust');
    stairs(simOut.u.Time, simOut.u.Data(:,1), '-', 'LineWidth', 1.4); hold on;
    plot(simOut.v.Time, simOut.v.Data(:,1), '-', 'LineWidth', 2.0);
    grid on; ylabel('Thrust [N]'); xlabel('Time [s]'); title('Thrust: Commanded vs Real (T vs T_m)');
    lgd_f6 = legend('Cmd T', 'Real T_m (Motors)');
    styleScopeLegend(lgd_f6, 15); yl = ylim; ylim([yl(1), yl(2) + 0.12*diff(yl)]);
end

function Plot_Torques(simOut)
    createFullFigure('Control Torques');
    stairs(simOut.u.Time, simOut.u.Data(:,2:4), '-', 'LineWidth', 1.4); hold on;
    plot(simOut.v.Time, simOut.v.Data(:,2:4), '-', 'LineWidth', 2.0);
    grid on; ylabel('Torque [N.m]'); xlabel('Time [s]'); title('Control Torques: Commanded vs Real (\tau vs \tau_m)');
    lgd_f7 = legend('\tau_{\phi} cmd', '\tau_{\theta} cmd', '\tau_{\psi} cmd', '\tau_{\phi} real', '\tau_{\theta} real', '\tau_{\psi} real');
    styleScopeLegend(lgd_f7, 15); yl = ylim; ylim([yl(1), yl(2) + 0.12*diff(yl)]);
end

% =========================================================================
% 3D Trajectory
% =========================================================================
function Plot_3D_Trajectory(simOut, p_ref_data)
    x_data = simOut.X.Data(:, 1);
    y_data = simOut.X.Data(:, 2);
    z_data = simOut.X.Data(:, 3);
    x_ref_data = p_ref_data(:, 1);
    y_ref_data = p_ref_data(:, 2);
    z_ref_data = p_ref_data(:, 3);
    
    figure('Name', '3D Drone Trajectory', 'NumberTitle', 'off', 'Color', 'w');
    hold on;
    grid on;
    
    plot3(x_ref_data, y_ref_data, z_ref_data, '--', 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980]);
    plot3(x_data, y_data, z_data, '-', 'LineWidth', 2, 'Color', [0 0.4470 0.7410]);
    plot3(x_data(1), y_data(1), z_data(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    plot3(x_data(end), y_data(end), z_data(end), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); 
    
    xlabel('X [m]', 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontWeight', 'bold');
    title('3D Drone Trajectory', 'FontSize', 14);
    legend('Reference', 'Actual Trajectory', 'Start', 'End', 'Location', 'best', 'FontSize', 12, 'FontWeight', 'bold');
    
    axis equal; 
    view(3); 
    hold off;
end