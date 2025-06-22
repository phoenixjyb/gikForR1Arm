% Plot Final Positions of R1 Robot Arm, Bottle, and Table
% This script visualizes the final configuration after GIK solving
% Loads the saved configuration from the main script

close all;

% Load the saved configuration
if ~exist('final_configuration.mat', 'file')
    error('final_configuration.mat not found. Please run the main script first to generate the final configuration.');
end

load('final_configuration.mat');
robotB = importrobot('fineUrdfs/r1_v2_1_0.urdf');

% Get final positions
T_gripper = getTransform(robotB, qClose, 'left_gripper_link');
T_finger1 = getTransform(robotB, qClose, 'left_gripper_finger_link1');
T_finger2 = getTransform(robotB, qClose, 'left_gripper_finger_link2');

pos_gripper = tform2trvec(T_gripper);
pos_finger1 = tform2trvec(T_finger1);
pos_finger2 = tform2trvec(T_finger2);
pos_bottle = tform2trvec(bottleInB.pose);

% Get table position (assuming it's at origin with some dimensions)
table_pos = [0, 0, 0]; % Table is typically at ground level
table_size = [1.0, 0.6, 0.02]; % Approximate table dimensions

% Create figure with subplots
figure('Position', [100, 100, 1200, 500]);

% Bird's Eye View (Top View - X-Y plane)
subplot(1, 2, 1);
hold on;
grid on;
axis equal;

% Plot table
table_corners = [
    table_pos(1) - table_size(1)/2, table_pos(2) - table_size(2)/2;
    table_pos(1) + table_size(1)/2, table_pos(2) - table_size(2)/2;
    table_pos(1) + table_size(1)/2, table_pos(2) + table_size(2)/2;
    table_pos(1) - table_size(1)/2, table_pos(2) + table_size(2)/2;
    table_pos(1) - table_size(1)/2, table_pos(2) - table_size(2)/2;
];
fill(table_corners(:,1), table_corners(:,2), [0.8, 0.8, 0.8], 'EdgeColor', 'k', 'LineWidth', 1);

% Plot bottle
bottle_radius = bottleInB.collision.Radius;
theta = 0:0.1:2*pi;
bottle_x = pos_bottle(1) + bottle_radius * cos(theta);
bottle_y = pos_bottle(2) + bottle_radius * sin(theta);
fill(bottle_x, bottle_y, [0.7, 0.9, 1.0], 'EdgeColor', 'b', 'LineWidth', 2);

% Plot gripper and fingers
plot(pos_gripper(1), pos_gripper(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(pos_finger1(1), pos_finger1(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(pos_finger2(1), pos_finger2(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

% Draw lines connecting gripper to fingers
plot([pos_gripper(1), pos_finger1(1)], [pos_gripper(2), pos_finger1(2)], 'g-', 'LineWidth', 2);
plot([pos_gripper(1), pos_finger2(1)], [pos_gripper(2), pos_finger2(2)], 'g-', 'LineWidth', 2);

% Add labels and title
xlabel('X (m)');
ylabel('Y (m)');
title('Bird''s Eye View (Top View)');
legend('Table', 'Bottle', 'Gripper Link', 'Finger Links', 'Location', 'best');

% Set axis limits
xlim([-0.5, 1.0]);
ylim([-0.5, 0.5]);

% 3D View
subplot(1, 2, 2);
hold on;
grid on;
axis equal;

% Plot table in 3D
table_vertices = [
    table_pos(1) - table_size(1)/2, table_pos(2) - table_size(2)/2, table_pos(3);
    table_pos(1) + table_size(1)/2, table_pos(2) - table_size(2)/2, table_pos(3);
    table_pos(1) + table_size(1)/2, table_pos(2) + table_size(2)/2, table_pos(3);
    table_pos(1) - table_size(1)/2, table_pos(2) + table_size(2)/2, table_pos(3);
];
table_faces = [1, 2, 3, 4];
patch('Vertices', table_vertices, 'Faces', table_faces, 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', 'k', 'LineWidth', 1);

% Plot bottle in 3D
[X, Y, Z] = cylinder(bottle_radius, 20);
Z = Z * bottleInB.collision.Length;
surf(X + pos_bottle(1), Y + pos_bottle(2), Z + pos_bottle(3), 'FaceColor', [0.7, 0.9, 1.0], 'EdgeColor', 'b', 'LineWidth', 1);

% Plot gripper and fingers in 3D
plot3(pos_gripper(1), pos_gripper(2), pos_gripper(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(pos_finger1(1), pos_finger1(2), pos_finger1(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(pos_finger2(1), pos_finger2(2), pos_finger2(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

% Draw lines connecting gripper to fingers in 3D
plot3([pos_gripper(1), pos_finger1(1)], [pos_gripper(2), pos_finger1(2)], [pos_gripper(3), pos_finger1(3)], 'g-', 'LineWidth', 2);
plot3([pos_gripper(1), pos_finger2(1)], [pos_gripper(2), pos_finger2(2)], [pos_gripper(3), pos_finger2(3)], 'g-', 'LineWidth', 2);

% Add labels and title
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D View');
legend('Table', 'Bottle', 'Gripper Link', 'Finger Links', 'Location', 'best');

% Set axis limits
xlim([-0.5, 1.0]);
ylim([-0.5, 0.5]);
zlim([0, 1.5]);

% Set view angle for better visualization
view(45, 30);

% Add overall title
sgtitle('Final Positions: R1 Robot Arm, Bottle, and Table', 'FontSize', 14, 'FontWeight', 'bold');

% Save the plot
saveas(gcf, 'final_positions_plot.png');
fprintf('Plot saved as final_positions_plot.png\n');

% Print position information
fprintf('\n=== Final Position Summary ===\n');
fprintf('Bottle Center:     (%.3f, %.3f, %.3f) m\n', pos_bottle(1), pos_bottle(2), pos_bottle(3));
fprintf('Gripper Link:      (%.3f, %.3f, %.3f) m\n', pos_gripper(1), pos_gripper(2), pos_gripper(3));
fprintf('Finger 1:          (%.3f, %.3f, %.3f) m\n', pos_finger1(1), pos_finger1(2), pos_finger1(3));
fprintf('Finger 2:          (%.3f, %.3f, %.3f) m\n', pos_finger2(1), pos_finger2(2), pos_finger2(3));
fprintf('Distance to bottle: %.3f m\n', norm(pos_gripper - pos_bottle)); 