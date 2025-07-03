% demo_show_urdf_animation.m
% Demo: Load URDF and STL meshes, visualize and animate the robot in MATLAB

% --- Marvin Setup ---
urdf_folder = fullfile('UrdfStlreader', 'Marvin_M6_S_R_CCS_689_URDF', 'urdf');
mesh_folder = fullfile('UrdfStlreader', 'Marvin_M6_S_R_CCS_689_URDF', 'meshes');
urdf_file = fullfile(urdf_folder, 'marvin_m6_s_r_ccs_689.urdf');
addpath(mesh_folder);
marvin = importrobot(urdf_file);
marvin.DataFormat = 'row';
marvin.Gravity = [0 0 -9.81];

% --- R1 Setup ---
r1_urdf_file = fullfile('fineUrdfs', 'R1_v2_1_0.urdf');
r1_mesh_folder = 'R1Meshes';
r1 = importrobot(r1_urdf_file, 'MeshPath', r1_mesh_folder, 'DataFormat', 'row');
r1.Gravity = [0 0 -9.81];

% --- Joint Info ---
% Marvin
marvinJointIdx = find(cellfun(@(b) ~strcmp(b.Joint.Type, 'fixed'), marvin.Bodies));
marvinNumJoints = numel(marvinJointIdx);
marvinLimits = zeros(marvinNumJoints,2);
for i = 1:marvinNumJoints
    joint = marvin.Bodies{marvinJointIdx(i)}.Joint;
    marvinLimits(i,:) = joint.PositionLimits;
end

% R1
r1JointIdx = find(cellfun(@(b) ~strcmp(b.Joint.Type, 'fixed'), r1.Bodies));
r1NumJoints = numel(r1JointIdx);
r1Limits = zeros(r1NumJoints,2);
for i = 1:r1NumJoints
    joint = r1.Bodies{r1JointIdx(i)}.Joint;
    r1Limits(i,:) = joint.PositionLimits;
end

% --- Trajectory ---
numSteps = 100;
t = linspace(0, 2*pi, numSteps);

marvin_traj = zeros(numSteps, marvinNumJoints);
for i = 1:marvinNumJoints
    low = marvinLimits(i,1); high = marvinLimits(i,2);
    if isfinite(low) && isfinite(high) && (high > low)
        marvin_traj(:,i) = low + (high-low)*(0.5 + 0.5*sin(t + (i-1)*pi/marvinNumJoints));
    else
        marvin_traj(:,i) = 0.5 * sin(t + (i-1)*pi/marvinNumJoints);
    end
end

r1_traj = zeros(numSteps, r1NumJoints);
for i = 1:r1NumJoints
    low = r1Limits(i,1); high = r1Limits(i,2);
    if isfinite(low) && isfinite(high) && (high > low)
        r1_traj(:,i) = low + (high-low)*(0.5 + 0.5*sin(t + (i-1)*pi/r1NumJoints + 1));
    else
        r1_traj(:,i) = 0.5 * sin(t + (i-1)*pi/r1NumJoints + 1);
    end
end

% --- Animation ---
figure('Name', 'URDF Robot Animation Demo: Marvin (left) vs R1 (right)');

for k = 1:numSteps
    % Marvin
    subplot(1,2,1);
    show(marvin, marvin_traj(k,:), 'PreservePlot', false, 'Frames', 'off');
    title(sprintf('Marvin\nStep %d/%d', k, numSteps));
    view(135, 20);

    % R1
    subplot(1,2,2);
    show(r1, r1_traj(k,:), 'PreservePlot', false, 'Frames', 'off');
    title(sprintf('R1\nStep %d/%d', k, numSteps));
    view(135, 20);

    drawnow;
end

disp('Animation complete.');