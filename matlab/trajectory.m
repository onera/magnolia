fprintf('Generating 3D Trajectory Plot...\t\t\t\t');

x_data = simOut.X.Data(:, 1);
y_data = simOut.X.Data(:, 2);
z_data = simOut.X.Data(:, 3);

p_ref_data = squeeze(ref.Data(:, 1, :))'; 
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
legend('Reference', 'Actual Trajectory', 'Start', 'End', 'Location', 'best');

axis equal; 
view(3); 
hold off;

fprintf('[done]\n');