close all

% === Setup ===
load ("bottle.mat") % this bottle.mat is created by createBottle.m file
load ('table.mat');% also need to load a table
robotB = importrobot('fineUrdfs/r1_v2_1_0.urdf');   
eeName = 'left_gripper_link'; % end-effector name for left arm'; this shall be the target pose for the GIK solver;

% eeName = 'left_tool_tip'; % end-effector name the actually gripper's tip'; this shall be the target pose for the GIK solver;
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
% eePosBs = calculateEEPosB(robotB, eeName);

%% visualize robot, bottle, and the table
% Call visualization function
% visualizeRobotScene(robotB, bottleInB, tableBox, eePosBs);

%% 2.  Create a distance-bounds constraint
clearance = 0.02; % 2 cm gap
linksToProtect = { ...
    "left_gripper_link", ...
    "left_arm_link6", ...
    "left_arm_link5", ...
    "left_arm_link4", ...
    "left_arm_link3", ...
    "left_arm_link2", ...
    "left_arm_link1", ...
    "right_gripper_link", ...
    "right_arm_link6", ...
    "right_arm_link5", ...
    "right_arm_link4", ...
    "right_arm_link3", ...
    "right_arm_link2", ...
    "right_arm_link1" ...
    }; % customization: manually defined links to project
distCs = cell(1,numel(linksToProtect));
for k = 1:numel(linksToProtect)
    distCs{k} = constraintDistanceBounds( ...
                  linksToProtect{k}, ...
                  "ReferenceBody","tableObstacle", ...
                  "Bounds",    [clearance 10] ); % upper bound cannot be infinite, and use 10m an example
end

%%

gik = generalizedInverseKinematics( ...
    'RigidBodyTree', robotB, ...
    'ConstraintInputs', {'pose', 'joint','distance'});

poseTgt = constraintPoseTarget(eeName);
poseTgt.TargetTransform = bottleInB.graspPose;
poseTgt.Weights = [1, 1];  % Position + Orientation

jointTgt = constraintJointBounds(robotB);  % define joint bounds

% === Compute Approach Pose ===
approachOffset = trvec2tform([0, 0, -0.1]);  % approach 10cm above
approachPose = bottleInB.graspPose * approachOffset;

poseApproach = constraintPoseTarget(eeName);
poseApproach.TargetTransform = approachPose;
poseApproach.Weights = [1, 1];


%% check reachability and adjust torso if necessary
torsoJoints = {
    "torso_joint1", ...
    "torso_joint2", ...
    "torso_joint3", ...
    "torso_joint4"
};

leftArmJoints = {
    "left_arm_joint1", ...
    "left_arm_joint2", ...
    "left_arm_joint3", ...
    "left_arm_joint4", ...
    "left_arm_joint5", ...
    "left_arm_joint6"
};

% Convert torsoJoints and leftArmJoints from cell arrays to string arrays
torsoJointsStr = string(torsoJoints);
leftArmJointsStr = string(leftArmJoints);

% Validate that the defined joints exist in the robot model
allJointNames = string(cellfun(@(b) b.Joint.Name, robotB.Bodies, 'UniformOutput', false));
assert(all(ismember(torsoJointsStr, allJointNames)), 'One or more torsoJoints not found in robotB.');
assert(all(ismember(leftArmJointsStr, allJointNames)), 'One or more leftArmJoints not found in robotB.');

targetPose = bottleInB.graspPose;

% === Offline Reachability Check === 
cloudPath = 'eeCloud_leftArm.mat';
useCloud = exist(cloudPath, 'file');

if useCloud
    fprintf('Using precomputed reachability cloud from %s\n', cloudPath);
    load(cloudPath, 'eePositions');
else
    fprintf('No reachability cloud found. Generating...\n');
    eePositions = [];

    % Sample joint space of left arm
    cfgHome = homeConfiguration(robotB);
    jointNamesCfg = {cfgHome.JointName};
    nSamples = 6;
    grid = cell(1, numel(leftArmJointsStr));

    for k = 1:numel(leftArmJointsStr)
        idx = find(strcmp(jointNamesCfg, leftArmJointsStr(k)));
        lims = robotB.Bodies{find(strcmp(allJointNames, leftArmJointsStr(k)))}.Joint.PositionLimits;
        grid{k} = linspace(lims(1), lims(2), nSamples);
    end
    [grid{:}] = ndgrid(grid{:});
    samples = cellfun(@(x) x(:), grid, 'UniformOutput', false);
    samples = [samples{:}];
    % FK and save positions using joint names for safe mapping
    jointNamesCfg = {cfgHome.JointName};
    for i = 1:size(samples,1)
        q = cfgHome;
        for j = 1:numel(leftArmJointsStr)
            idx = find(strcmp(jointNamesCfg, leftArmJointsStr(j)));
            q(idx).JointPosition = samples(i,j);
        end
        T = getTransform(robotB, q, eeName);
        eePositions(i,:) = tform2trvec(T); %#ok<AGROW>
    end
    visualizeRobotScene(robotB, bottleInB, tableBox, eePositions); % visualize the reachability cloud and the bottle and the table and the robot

    % Save the reachability cloud
    fprintf('Saving reachability cloud to %s\n', cloudPath);
    save(cloudPath, 'eePositions');

end

% Check if grasp pose is within reach
graspPos = tform2trvec(bottleInB.graspPose);
dists = vecnorm(eePositions - graspPos, 2, 2);
if all(dists > 0.15) % this threshold can be adjusted
    warning('Target pose is outside sampled reachability cloud. Will likely fail.');
end

[reachable, qSol, info] = checkLeftArmReachability(robotB, targetPose, torsoJoints, leftArmJoints);

if ~reachable
    disp('Left arm cannot reach with fixed torso. Attempting torso adjustment...');
else
    disp('Left arm can reach target pose with fixed torso.');
end


% === Solve GIK for Approach and Grasp ===
initialGuess = homeConfiguration(robotB);
initialGuess = updateHomePositionforR1_wholeBody(initialGuess); % update home position based on Galaxea robot's arm down config.

allDist = [distCs{:}]; 

% Solve GIK for approach pose first
% This will give us a configuration that is suitable for grasping
% the bottle from above, avoiding the table.

% This is a two-step process:
% 1. Solve for the approach pose
% 2. Use the approach pose as an initial guess to solve for the grasp pose
% The first step is to find a configuration that allows the robot to
% approach the bottle from above, avoiding the table.
% The second step is to find a configuration that allows the robot to
% grasp the bottle, using the approach pose as an initial guess.


[qApproach, solutionInfo] = gik(initialGuess, poseApproach, jointTgt, allDist);

[qGrasp, solutionInfo2] = gik(qApproach, poseTgt, jointTgt, allDist);


% === Interpolate Trajectory ===
n1 = 50;  % number of steps
n2 = 50;
% Interpolate from initialGuess to qApproach
traj1 = zeros(n1, numel(qApproach));
for j = 1:numel(qApproach)
    traj1(:, j) = linspace(initialGuess(j).JointPosition, qApproach(j).JointPosition, n1);
end

% Interpolate from qApproach to qGrasp
traj2 = zeros(n2, numel(qGrasp));
for j = 1:numel(qGrasp)
    traj2(:, j) = linspace(qApproach(j).JointPosition, qGrasp(j).JointPosition, n2);
end

% Concatenate trajectories
traj = [traj1; traj2];


%%
% === Animate ===
figure;
ax = axes; view(3); axis equal; grid on;
show(robotB, qApproach, 'PreservePlot', false, 'Parent', ax);
hold on;
show(bottleInB.collision);
plotTransforms(tform2trvec(bottleInB.graspPose), tform2quat(bottleInB.graspPose), 'FrameSize', 0.05);

for t = 1:(n1+n2)
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