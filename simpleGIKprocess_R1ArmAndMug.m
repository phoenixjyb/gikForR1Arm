% 
close all

% === Setup ===
load ("bottle.mat") % this bottle.mat is created by createBottle.m file
load ('table.mat');% also need to load a table
robotB = importrobot('fineUrdfs/left_arm_module.urdf'); 
% robotB.BaseName = 'base_link';  
eeName = robotB.BodyNames{end};

%%%% arm initial position %%%%%
% 1.56,2.94,-2.54,0.0,0.0,0.0 for left arm
% -1.56,2.94,-2.54,0.0,0.0,0.0 for right arm

%%%%%% define the table to avoid and adding more 
%% 1.  Attach the box to the robot as a fixed body
tableBody            = rigidBody("tableObstacle");
jnt                  = rigidBodyJoint("tableFixed","fixed");
tableBody.Joint = jnt;
addCollision (tableBody, tableBox);
addBody(robotB, tableBody, robotB.BaseName);  % fixed to world/base

%% calculate eePosB
% Calculate reachable workspace points for robotB
eePosBs = calculateEEPosB(robotB, eeName);

%% visualize robot, bottle, and the table
% Call visualization function
visualizeRobotScene(robotB, bottleInB, tableBox, eePosBs);

%% 2.  Create a distance-bounds constraint
clearance = 0.02; % 2 cm gap
linksToProtect = { ...
    "left_gripper_link", ...
    "left_arm_link6", ...
    "left_arm_link5", ...
    "left_arm_link4", ...
    "left_arm_link3", ...
    }; % customization: manually defined links to project
distCs = cell(1,numel(linksToProtect));
for k = 1:numel(linksToProtect)
    distCs{k} = constraintDistanceBounds( ...
                  linksToProtect{k}, ...
                  "ReferenceBody","tableObstacle", ...
                  "Bounds",    [clearance 10] ); % upper bound cannot be infinite, and use 10m an example
end

%% Helper function to solve GIK and interpolate trajectory
function [qApproach, qGrasp, traj, solutionInfo, solutionInfo2] = solveGraspGIK(robot, eeName, graspPose, distConstraints, initialGuess)
    % Setup GIK solver
    gik = generalizedInverseKinematics( ...
        'RigidBodyTree', robot, ...
        'ConstraintInputs', {'pose', 'joint','distance'});

    % Pose constraints
    approachOffset = trvec2tform([0, 0, -0.1]);  % 10 cm above
    approachPose = graspPose * approachOffset;

    poseApproach = constraintPoseTarget(eeName);
    poseApproach.TargetTransform = approachPose;
    poseApproach.Weights = [1, 1];

    poseTgt = constraintPoseTarget(eeName);
    poseTgt.TargetTransform = graspPose;
    poseTgt.Weights = [1, 1];

    % Joint constraint
    jointTgt = constraintJointBounds(robot);
    allDist = [distConstraints{:}];

    % Solve GIK for approach and grasp
    [qApproach, solutionInfo] = gik(initialGuess, poseApproach, jointTgt, allDist);
    [qGrasp, solutionInfo2]  = gik(qApproach, poseTgt, jointTgt, allDist);

    % Interpolate trajectories
    n1 = 50; n2 = 50;
    traj1 = zeros(n1, numel(qApproach));
    traj2 = zeros(n2, numel(qGrasp));

    for j = 1:numel(qApproach)
        traj1(:, j) = linspace(initialGuess(j).JointPosition, qApproach(j).JointPosition, n1);
        traj2(:, j) = linspace(qApproach(j).JointPosition, qGrasp(j).JointPosition, n2);
    end
    traj = [traj1; traj2];
end

%%

initialGuess = homeConfiguration(robotB);
initialGuess = updateHomePositionforR1_leftArm(initialGuess);

[qApproach, qGrasp, traj, solutionInfo, solutionInfo2] = solveGraspGIK( ...
    robotB, eeName, bottleInB.graspPose, distCs, initialGuess);

if ~strcmp(solutionInfo.Status, 'Success')
    error('GIK for approach failed: %s', solutionInfo.Status);
end
if ~strcmp(solutionInfo2.Status, 'Success')
    error('GIK for grasp failed: %s', solutionInfo2.Status);
end

% === Animate ===
figure;
ax = axes; view(3); axis equal; grid on;
show(robotB, qApproach, 'PreservePlot', false, 'Parent', ax);
hold on;
show(bottleInB.collision);
plotTransforms(tform2trvec(bottleInB.graspPose), tform2quat(bottleInB.graspPose), 'FrameSize', 0.05);

nTotal = size(traj,1);
for t = 1:nTotal
    qNow = qGrasp;
    for j = 1:numel(qNow)
        qNow(j).JointPosition = traj(t,j);
    end
    show(robotB, qNow, 'PreservePlot', false, 'Parent', ax);
    pause(0.5)
    drawnow;
end


%% save file as json
% {
% Convert to a cell array of rows
trajCell = num2cell(traj, 2);  % each row becomes a 1x6 array in a cell

% Optional: structure the data for clarity
trajStruct = struct('trajectory', trajCell);

% Convert to JSON
jsonStr = jsonencode(trajStruct);

% Write to file
fid = fopen('traj_exmaple.json', 'w');
fwrite(fid, jsonStr, 'char');
fclose(fid);
%}