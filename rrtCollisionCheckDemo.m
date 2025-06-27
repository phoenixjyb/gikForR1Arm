% rrtCollisionCheckDemo.m
% Demo: Collision checking and RRT planning for R1 robot arm with obstacles
% No IK/GIK, just home config and a slightly moved left arm as goal

close all; clc; clear;

%% --- Load robot and environment ---
robotB = importrobot('fineUrdfs/r1_v2_1_0.urdf');
robotB = attachLeftArmCollisionPrimitives(robotB, 'R1Meshes', 'box');

robotB.DataFormat = 'struct';

% Update home position
initialGuess = homeConfiguration(robotB);
initialGuess = updateHomePositionforR1_wholeBody(robotB, initialGuess);

% Load obstacles
load('bottle.mat'); % bottleInB
load('table.mat');  % tableBox
load('cardbox.mat'); % cardbox

%% --- Lock all joints except left arm and fingers ---
leftArmJoints = {
    'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3', ...
    'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6'
};
fingerJoints = {'left_gripper_finger_joint1', 'left_gripper_finger_joint2'};
allJointNames = {initialGuess.JointName};

% Indices for left arm and fingers
movableIdx = find(ismember(allJointNames, [leftArmJoints, fingerJoints]));

%% --- Define start and goal configurations ---
qStartStruct = initialGuess;
qGoalStruct = initialGuess;
% Move left arm joints a bit for goal
for i = 1:numel(qGoalStruct)
    if ismember(qGoalStruct(i).JointName, leftArmJoints)
        qGoalStruct(i).JointPosition = qGoalStruct(i).JointPosition + 0.2*randn();
    end
end

robotB.DataFormat = 'row';
qStart = [qStartStruct.JointPosition];
qGoal  = [qGoalStruct.JointPosition];

%% --- Lock all non-movable joints in both configs ---
for i = 1:numel(qStart)
    if ~ismember(i, movableIdx)
        qGoal(i) = qStart(i); % lock non-movable joints
    end
end

%% --- Collision check at start and goal ---
[isColStart1, ~] = checkCollision(robotB, qStart, {tableBox}, 'Exhaustive', 'on');
[isColStart2, ~] = checkCollision(robotB, qStart, {bottleInB.collision}, 'Exhaustive', 'on');
[isColStart3, ~] = checkCollision(robotB, qStart, {cardbox}, 'Exhaustive', 'on');
[isColGoal1, ~] = checkCollision(robotB, qGoal, {tableBox}, 'Exhaustive', 'on');
[isColGoal2, ~] = checkCollision(robotB, qGoal, {bottleInB.collision}, 'Exhaustive', 'on');
[isColGoal3, ~] = checkCollision(robotB, qGoal, {cardbox}, 'Exhaustive', 'on');

if any(isColStart1) || any(isColStart2) || any(isColStart3)
    error('Start configuration is in collision!');
end
if any(isColGoal1) || any(isColGoal2) || any(isColGoal3)
    error('Goal configuration is in collision!');
end

%% --- RRT Planning ---
planner = manipulatorRRT(robotB, {tableBox, bottleInB.collision, cardbox}, ...
    'IgnoreSelfCollision', true, ...
    'MaxConnectionDistance', 0.1, ...
    'MaxIterations', 2000);

[pathObj, solnInfo] = plan(planner, qStart, qGoal);

if isempty(pathObj.States)
    disp('No collision-free path found by RRT.');
else
    disp('Collision-free path found by RRT!');
    % Visualize
    figure; hold on; axis equal; grid on;
    show(tableBox);
    show(cardbox);
    show(bottleInB.collision);
    for i = 1:size(pathObj.States,1)
        show(robotB, pathObj.States(i,:), 'PreservePlot', false, 'Collisions', 'on');
        drawnow;
    end
    title('RRT Planned Path for Left Arm');
end 