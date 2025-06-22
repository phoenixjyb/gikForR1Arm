% this script uses arm only urdfs to perform motion transfer
close all
clc
clear

%%
% --- 1. Load URDF models for robotA and robotB ---
robotA = importrobot('gr1t1_leftArm_only.urdf'); 
robotA.DataFormat = 'struct';
robotA.Gravity = [0 0 -9.81];

robotB = importrobot('r1_leftArm_only_arm_down.urdf'); % it seems that arms from down position as initial states can 
robotB.DataFormat = 'struct';
robotB.Gravity = [0 0 -9.81];

disp ('this function deals with left arm for now, not both arms')

% load predefined joint limits for robotarms
[jointLimitsA, jointLimitsB] = setJointLimits(); % limits are in degree
disp ('be aware that jointLimitB contains bounds for both arms, need to truncate it to left arm only')

% --- 2. Generate or load RobotA joint trajectory ---

% Example: random joint angles within joint limits (replace with actual data)
% joint commands for each joint in this example 


% give this set of ee-trajectory defined by joint angles, it shall output a
% value that is enclosed by workSpace ...

load('joint_angles_deg_for_reference_7dof.mat'); % 
qA = deg2rad(joint_angles_deg); % qA for joint angles in radian for arm A;
endEffectorNameA = robotA.BodyNames{end};

numPoints = length(qA);
node = ros2node('/robotA_node');

%%
% --- 3. Compute RobotA end-effector trajectory ---
% i.e., forward kinematics

T_A = zeros(4,4,numPoints);
for i = 1:numPoints
    configA = homeConfiguration(robotA);
    for j = 1:length(configA)
        configA(j).JointPosition = qA(i,j);
    end
    T_A(:,:,i) = getTransform(robotA, configA, endEffectorNameA);
end

%{
% plot out T_A to check
positions = squeeze(T_A(1:3, 4, :));  % [3 x numPoints]
positions = positions';    % [numPoints x 3]

figure;
plot3(positions(:,1), positions(:,2), positions(:,3), 'b.-', 'LineWidth', 1.5);
hold on;

% Mark start and end points
plot3(positions(1,1), positions(1,2), positions(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');  % Start point (green)
plot3(positions(end,1), positions(end,2), positions(end,3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  % End point (red)

% Labels and settings
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('Trajectory', 'Start', 'End');
title('End-Effector Trajectory with Start and End Points for robotA');
grid on; axis equal; view(3);

text(positions(1,1), positions(1,2), positions(1,3), '  Start', 'Color', 'g', 'FontWeight', 'bold');
text(positions(end,1), positions(end,2), positions(end,3), '  End', 'Color', 'r', 'FontWeight', 'bold');
%}

%% 
% --- 3.a setup ros2 node
nodeA = ros2node('/relay_node');
% --- 3.b step 2: publish it, given T_A 
publishSourceTrajectory(nodeA, '/robotA/ee_trajectory', T_A);

%%
% --- 4. Define coordinate transform from RobotA base to RobotB base ---
 % get right base name for both robots
left_arm_base_robotA = 'torso_link';
left_arm_base_robotB = 'left_arm_base_link'; 

% -- Get transform from A base to B base in world frame --
cfgA = homeConfiguration(robotA);
cfgB = homeConfiguration(robotB);

% Get transform from world to base of each robot
T_world_to_A = getTransform(robotA, cfgA, left_arm_base_robotA);
T_world_to_B = getTransform(robotB, cfgB, left_arm_base_robotB);

% Compute transform from A base to B base
T_A_to_B = T_world_to_A \ T_world_to_B;

% -- Transform trajectory from A base frame to B base frame --
numPoints = size(T_A, 3);
T_B = zeros(4, 4, numPoints);

for i = 1:numPoints
    T_B(:,:,i) = T_A_to_B \ T_A(:,:,i);
end

% {
% plot out T_B to check
positions = squeeze(T_B(1:3, 4, :));  % [3 x numPoints]
positions = positions';              % [numPoints x 3]

figure;
plot3(positions(:,1), positions(:,2), positions(:,3), 'b.-', 'LineWidth', 1.5);
hold on;

% Mark start and end points
plot3(positions(1,1), positions(1,2), positions(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');  % Start point (green)
plot3(positions(end,1), positions(end,2), positions(end,3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  % End point (red)

% Labels and settings
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('Trajectory', 'Start', 'End');
title('End-Effector Trajectory with Start and End Points for robotB');
grid on; axis equal; view(3);

text(positions(1,1), positions(1,2), positions(1,3), '  Start', 'Color', 'g', 'FontWeight', 'bold');
text(positions(end,1), positions(end,2), positions(end,3), '  End', 'Color', 'r', 'FontWeight', 'bold');
%}

endEffectorNameB = robotB.BodyNames{end};

%%
% --- 5. Solve inverse kinematics for RobotB to follow transformed trajectory ---
%{
% it appears ik gives inconsistent angle solution for robotB, hence code
snippet is ignoreed 

ik = inverseKinematics('RigidBodyTree', robotB);
weights = [0.1 0.1 0.1 1 1 1]; % weights for position and orientation

initialguess = homeConfiguration(robotB);
cfgB = homeConfiguration(robotB);
qB = zeros(numPoints, numel(cfgB));
% qB = zeros(numPoints, robotB.NumJoints);
for i = 1:numPoints
    [configSol, solInfo] = ik(endEffectorNameB, T_B(:,:,i), weights, initialguess);
    qB(i,:) = [configSol.JointPosition];
    initialguess = configSol; % update initial guess for faster convergence
end

%}

% {
% try gik: generalized inverse kinematics
% the goal for this retargeting issues is given:
% - Start in the home configuration;
% - No abrupt changes in robot configuration;
% - Keep the end-effector following the given trajectory,but allow for
% 5cm's inaccuracy;
% - allow robotB'EE reaching first point of the trajectory with a few
% steps, maximum 10 extra steps, don't need to instantly arrive at the trajectory at 1st step;
% - joints of robotB cannot exceed each's limits, the limit strucutre is
% provided in jointLimitsB;

qB = generateConstrainedIKTrajectory(robotB, T_B, jointLimitsB);
% joints' degree in radian;
% optional pair of inputs can be defined after jointLimitsB

%} 

% --- 5.b publish robot bjoint trajectory
cfgB = homeConfiguration(robotB);
cfgB = updateHomePositionforR1(cfgB); % manually update home position of left arm
jointNames = {cfgB.JointName};
nodeB = ros2node('/motion_transfer_node');
publishJointTrajectory(nodeB, '/robotB/joint_trajectory', qB, jointNames);


%%
% --- 6. Visualize end-effector trajectories for a simple comparison---
figure;
subplot(2,1,1);
plot(qA);
title('RobotA Joint Angles');
xlabel('Trajectory Point');
ylabel('Joint Angle (rad)');

subplot(2,1,2);
plot(qB);
title('RobotB Joint Angles');
xlabel('Trajectory Point');
ylabel('Joint Angle (rad)');

%%
% --- 7. Verify RobotB end-effector trajectory accuracy ---
% considering trajectory B has a ramp_up stage, numPoints shall be modified
size_qB = size(qB,1);
ramp_up_size = size_qB - numPoints;

T_B_actual = zeros(4,4,size_qB);
for i = 1:size_qB
    configB = homeConfiguration(robotB);
    for j = 1:length(configB)
        configB(j).JointPosition = qB(i,j);
    end
    T_B_actual(:,:,i) = getTransform(robotB, configB, endEffectorNameB);
end

% Calculate position error (Euclidean distance)
posError = zeros(numPoints,1);
T_B_no_rampup = T_B_actual(:,:,ramp_up_size+1:end);


for i = 1:numPoints
    p_target = tform2trvec(T_B(:,:,i));
    p_actual = tform2trvec(T_B_no_rampup(:,:,i));
    posError(i) = norm(p_actual - p_target);
end

figure;
plot(posError);
title('RobotB End-Effector Position Error');
xlabel('Trajectory Point');
ylabel('Position Error (m)');
grid on;

%%
% --- 8. compare robot joints' trajectories ---
visualizeDualRobotJointTrajectories(robotA, qA, robotB, qB)

%% double check reachable area for each arm
% {
% joint limits are set wrongly -- could well be, not needs to double check
numSamples = 5000;
eePosAs = computeReachableWorkspace(robotA,endEffectorNameA,numSamples);
eePosBs = computeReachableWorkspace(robotB,endEffectorNameB,numSamples);
figure;
s1=scatter3(eePosAs(:,1), eePosAs(:,2), eePosAs(:,3), 1);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Reachable Workspace of the Robot Arms');
axis equal; grid on;
hold on 
s2=scatter3(eePosBs(:,1), eePosBs(:,2), eePosBs(:,3), 1,'red');
% transfer A to B; % Homogenize all points (Nx4)
eeHomA = [eePosAs, ones(size(eePosAs,1), 1)]';  % 4xN
eeHomB = T_A_to_B * eeHomA;                        % 4xN
eePosAinBs = eeHomB(1:3, :)';                % Nx3
s3=scatter3(eePosAinBs(:,1), eePosAinBs(:,2), eePosAinBs(:,3), 1,'yellow');
legend([s1, s2, s3], {'Robot A Reachable', 'Robot B Reachable', 'Robot A Reachable in B coordinate'});
title('Reachability Comparison');
%}
% overlay robots with reachable area
show(robotA);
show(robotB);

%% define a mug in A, show it and then transform it to B's coordinate system

% {
[transMuginB,x,y,z,mugInA] = mugAsObjToGrasp(eePosAs);
[pts_transformed, mugInB]= transformObjectBetweenBases(robotA, robotB, transMuginB, ...
    left_arm_base_robotA , left_arm_base_robotB, mugInA);
save ('mugs.mat','mugInA','mugInB'); % now we can store mug position in B for robotB to grab
X_new = reshape(pts_transformed(1,:), size(x));
Y_new = reshape(pts_transformed(2,:), size(y));
Z_new = reshape(pts_transformed(3,:), size(z));

surf(x,y,z, 'FaceAlpha', 0.2); hold on;
surf(X_new, Y_new, Z_new, 'FaceAlpha', 0.6);
legend('Original', 'Transformed');
axis equal; view(3); grid on;
hold on
s2=scatter3(eePosAinBs(:,1), eePosAinBs(:,2), eePosAinBs(:,3), 1,'cyan');
show(robotB), show(robotA) % overlay robotA and robotB

%% now further overlay robotA and robotB
figure (6);
surf(x,y,z, 'FaceAlpha', 0.2); hold on;
surf(X_new, Y_new, Z_new, 'FaceAlpha', 0.6);

%}




