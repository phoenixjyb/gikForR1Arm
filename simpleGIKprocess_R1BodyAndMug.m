close all

% === Setup ===
load ("bottle.mat") % this bottle.mat is created by createBottle.m file
% tweak the bottle pose to be grasped by the left arm


% --- Move bottle and keep its grasp offset consistent ---
% define offset values in terms of tr
x_value = 0.0; % Move bottle to X=0.0m
yaw_value = 0.0; % 0 deg yaw
T_shift  = trvec2tform([x_value, 0, 0]);                 
T_rotZ30 = axang2tform([0 0 1 deg2rad(yaw_value)]); 

% 1️⃣ Compute the grasp pose offset relative to the current bottle frame *before* moving it
T_graspLocal = bottleInB.pose \ bottleInB.graspPose;

% 2️⃣ Move the bottle in the world

bottleInB.pose = T_shift * T_rotZ30 * bottleInB.pose;
bottleInB.collision.Pose = bottleInB.pose;

% 3️⃣ Re‑establish the graspPose using the stored local offset and add a tool offset
graspOffset = trvec2tform([0.0, 0, 0]); % 0cm forward offset for the tool
bottleInB.graspPose = bottleInB.pose * T_graspLocal * graspOffset;

load ('table.mat');% also need to load a table
robotB = importrobot('fineUrdfs/r1_v2_1_0.urdf');   
eeName = 'left_gripper_link'; % end-effector name for left arm'; this shall be the target pose for the GIK solver;

% eeName = 'left_tool_tip'; % end-effector name the actually gripper's tip'; this shall be the target pose for the GIK solver;
%%%% arm initial position %%%%%
% 1.56,2.94,-2.54,0.0,0.0,0.0 for left arm
% -1.56,2.94,-2.54,0.0,0.0,0.0 for right arm

% === Define Initial Robot Configuration ===
initialGuess = homeConfiguration(robotB);
initialGuess = updateHomePositionforR1_wholeBody(initialGuess);

% Ensure finger joints start in open position
fingerJointNames = {'left_gripper_finger_joint1', 'left_gripper_finger_joint2'};
for i = 1:numel(initialGuess)
    if ismember(initialGuess(i).JointName, fingerJointNames)
        initialGuess(i).JointPosition = 0.05; % Set fingers to open position (0.05 = fully open)
    end
end

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
    }; % customization: manually defined links to protect;
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

%%
% check reachability and adjust torso if necessary
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

% Extract grasp position for reachability checking
graspPos = tform2trvec(targetPose);

% === GIK-based Reachability Check ===
fprintf('Performing GIK-based reachability check...\n');
gik_check = generalizedInverseKinematics('RigidBodyTree', robotB, 'ConstraintInputs', {'pose', 'joint'});
poseTgt_check = constraintPoseTarget(eeName);
poseTgt_check.TargetTransform = targetPose;
% Use a loose tolerance for the check to see if it's generally possible
poseTgt_check.PositionTolerance = 0.05; % 5cm
poseTgt_check.OrientationTolerance = deg2rad(15); % 15 degrees

% Temporarily use only the left arm joints for a quick check
jointBounds_check = constraintJointBounds(robotB);
for i = 1:numel(initialGuess)
    if ~ismember(initialGuess(i).JointName, leftArmJointsStr)
        fixedPos = initialGuess(i).JointPosition;
        jointBounds_check.Bounds(i,:) = [fixedPos, fixedPos];
    end
end

[~, solutionInfo_check] = gik_check(initialGuess, poseTgt_check, jointBounds_check);
isReachable = strcmpi(solutionInfo_check.Status, 'success');
% =====================================

if isReachable
    disp('GIK Check SUCCESS: Grasp pose is reachable using fixed torso.');

    % Lock all joints except left arm
    jointBounds = constraintJointBounds(robotB);
    for i = 1:numel(initialGuess)
        jointName = initialGuess(i).JointName;
        if ~ismember(jointName, leftArmJointsStr)
            fixedPos = initialGuess(i).JointPosition;
            jointBounds.Bounds(i,:) = [fixedPos, fixedPos];
        end
    end

    % Solve GIK with torso and right arm locked
    bottleDiameter = bottleInB.collision.Radius * 2;
    [qApproach, qGrasp, qClose, traj, solutionInfo, solutionInfo2, solutionInfo3] = solveGraspGIK(robotB, eeName, targetPose, distCs, initialGuess, jointBounds, bottleDiameter);
    
else
    disp('GIK Check FAILED: Grasp pose is unreachable with fixed torso.  ▶  Stage 2: torso adjustment');

    % ----- 1.  Smarter sampling of torso poses over joint1 and joint2 -----
    angleRange = [-0.3, -0.15, 0, 0.15, 0.3];  % adjustable lean/tilt range
    [T1, T2] = ndgrid(angleRange, angleRange);  % joint1 = lean forward/backward, joint2 = tilt side to side
    nSamples = numel(T1);

    torsoSamples = [T1(:), T2(:), repmat([0 0], nSamples, 1)];  % explicitly leave joints 3 and 4 unchanged

    successTorso = false;   % flag
    qTorsoChosen = [];

    % ----- 2.  Try each torso sample -----
    qTorsoArray = repmat(initialGuess, nSamples, 1);
    for iSample = 1:nSamples
        for tj = 1:numel(torsoJointsStr)
            idx = find(strcmp({qTorsoArray(iSample,:).JointName}, torsoJointsStr(tj)));
            qTorsoArray(iSample,idx).JointPosition = torsoSamples(iSample,tj);
        end
    end

    reachableFlags = false(nSamples,1);
    qTorsoResults = cell(nSamples,1);

    parfor s = 1:size(torsoSamples,1)
        qTorso = qTorsoArray(s,:);
        eePositionsNew = generateReachabilityWithTorso(robotB, qTorso, leftArmJointsStr, eeName);

        if any(vecnorm(eePositionsNew - graspPos,2,2) <= 0.15)
            reachableFlags(s) = true;
            qTorsoResults{s} = qTorso;
        else
            reachableFlags(s) = false;
            qTorsoResults{s} = [];
        end
    end

    idxReachable = find(reachableFlags, 1);

    if isempty(idxReachable)
        error('All torso samples tried, target still unreachable.');
    else
        disp("  ✓ Reachable with torso sample " + idxReachable);
        successTorso = true;
        qTorsoChosen = qTorsoResults{idxReachable};
    end

    % Update initial guess with chosen torso joints
    initialGuess = qTorsoChosen;

    % Lock only torso + left arm for final GIK solve
    jointBounds = constraintJointBounds(robotB);
    keepJoints = [torsoJointsStr; leftArmJointsStr];
    keepJoints = keepJoints(:);  % ensure column vector
    for i = 1:numel(initialGuess)
        jName = initialGuess(i).JointName;
        if ~ismember(jName, keepJoints)
            jointBounds.Bounds(i,:) = [initialGuess(i).JointPosition, initialGuess(i).JointPosition];
        end
    end

    % Final two-stage GIK with updated torso
    bottleDiameter = bottleInB.collision.Radius * 2;
    [qApproach, qGrasp, qClose, traj, solutionInfo, solutionInfo2, solutionInfo3] = ...
        solveGraspGIK(robotB, eeName, targetPose, distCs, initialGuess, jointBounds, bottleDiameter);
end


disp("GIK status (approach): " + string(solutionInfo.Status));
if ~strcmpi(solutionInfo.Status,'success')
    warning("Approach stage not fully successful. Showing best available solution.");
end
disp(solutionInfo);   % show full diagnostic info

disp("GIK status (grasp): " + string(solutionInfo2.Status));
if ~strcmpi(solutionInfo2.Status,'success')
    warning("Grasp stage not fully successful. Showing best available solution.");
end
disp(solutionInfo2);  % show full diagnostic info

disp("GIK status (finger close): " + string(solutionInfo3.Status));
if ~strcmpi(solutionInfo3.Status,'success')
    warning("Finger closing stage not fully successful. Showing best available solution.");
end
disp(solutionInfo3);  % show full diagnostic info

% === Print Final Link Positions ===
T_gripper = getTransform(robotB, qClose, 'left_gripper_link');
T_finger1 = getTransform(robotB, qClose, 'left_gripper_finger_link1');
T_finger2 = getTransform(robotB, qClose, 'left_gripper_finger_link2');

pos_gripper = tform2trvec(T_gripper);
pos_finger1 = tform2trvec(T_finger1);
pos_finger2 = tform2trvec(T_finger2);

% Get bottle position for comparison
pos_bottle = tform2trvec(bottleInB.pose);

fprintf('\n--- Final Link & Bottle Positions (in base frame) ---\n');
fprintf('Bottle Center:             (X, Y, Z) = (%.4f, %.4f, %.4f)\n', pos_bottle(1), pos_bottle(2), pos_bottle(3));
fprintf('left_gripper_link:         (X, Y, Z) = (%.4f, %.4f, %.4f)\n', pos_gripper(1), pos_gripper(2), pos_gripper(3));
fprintf('left_gripper_finger_link1: (X, Y, Z) = (%.4f, %.4f, %.4f)\n', pos_finger1(1), pos_finger1(2), pos_finger1(3));
fprintf('left_gripper_finger_link2: (X, Y, Z) = (%.4f, %.4f, %.4f)\n', pos_finger2(1), pos_finger2(2), pos_finger2(3));
fprintf('-----------------------------------------------------\n\n');

%%
% === Animate ===
figure;
ax = axes; view(3); axis equal; grid on;
show(robotB, qApproach, 'PreservePlot', false, 'Parent', ax);
hold on;
show(bottleInB.collision);
plotTransforms(tform2trvec(bottleInB.graspPose), tform2quat(bottleInB.graspPose), 'FrameSize', 0.05);

nTotal = size(traj,1);
for t = 1:nTotal
    % Create configuration from trajectory data directly
    qNow = initialGuess;  % Start with initial configuration
    for j = 1:numel(qNow)
        qNow(j).JointPosition = traj(t,j);
    end
    
    show(robotB, qNow, 'PreservePlot', false, 'Parent', ax);
    pause(0.5)
    drawnow;
end

% Show final closed gripper state
show(robotB, qClose, 'PreservePlot', false, 'Parent', ax);
title('Final state: Gripper closed around bottle');
pause(1);

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

%% helper functions
function [success, qNewTorso] = solveTorsoToHelpReach(robot, targetPose, torsoJoints, armJoints, eeName)
    gik = generalizedInverseKinematics( ...
        'RigidBodyTree', robot, ...
        'ConstraintInputs', {'pose', 'joint'});

    poseTgt = constraintPoseTarget(eeName);
    poseTgt.TargetTransform = targetPose;
    poseTgt.Weights = [0.3 0.3];  % relaxed constraint for better convergence

    jointTgt = constraintJointBounds(robot);
    qInit = homeConfiguration(robot);

    jointNamesToKeep = string([torsoJoints(:); armJoints(:)]);
    for i = 1:numel(qInit)
        thisJoint = qInit(i).JointName;
        if ~ismember(thisJoint, jointNamesToKeep)
            jointTgt.Bounds(i,:) = [qInit(i).JointPosition, qInit(i).JointPosition];
        end
    end

    [qSol, solutionInfo] = gik(qInit, poseTgt, jointTgt);
    success = strcmp(solutionInfo.Status, 'Success');
    qNewTorso = qSol;
end

function eePositions = generateReachabilityWithTorso(robot, qTorso, jointList, eeName)
    cfgHome = qTorso;
    jointNamesCfg = {cfgHome.JointName};
    nSamples = 6;
    gridSamples = cell(1, numel(jointList));
    allJointNames = string(cellfun(@(x) x.Joint.Name, robot.Bodies, 'UniformOutput', false));

    for k = 1:numel(jointList)
        lims = robot.Bodies{strcmp(allJointNames, jointList{k})}.Joint.PositionLimits;
        gridSamples{k} = linspace(lims(1), lims(2), nSamples);
    end
    [gridSamples{:}] = ndgrid(gridSamples{:});
    samples = cellfun(@(x) x(:), gridSamples, 'UniformOutput', false);
    samples = [samples{:}];

    eePositions = zeros(size(samples,1), 3);
    for i = 1:size(samples,1)
        q = cfgHome;
        for j = 1:numel(jointList)
            idx = find(strcmp(jointNamesCfg, jointList{j}));
            q(idx).JointPosition = samples(i,j);
        end
        T = getTransform(robot, q, eeName);
        eePositions(i,:) = tform2trvec(T);
    end
end

function [qApproach, qGrasp, qClose, traj, solutionInfo, solutionInfo2, solutionInfo3] = solveGraspGIK(robot, eeName, graspPose, distConstraints, initialGuess, jointBounds, bottleDiameter)
    % Setup GIK solver
    gik = generalizedInverseKinematics( ...
        'RigidBodyTree', robot, ...
        'ConstraintInputs', {'pose', 'joint','distance'});

    % Define approach pose
    approachOffset = trvec2tform([0, 0, -0.1]);  % 10 cm above grasp pose
    approachPose = graspPose * approachOffset;

    % Pose constraints
    poseApproach = constraintPoseTarget(eeName);
    poseApproach.TargetTransform = approachPose;
    poseApproach.Weights = [0.5, 0.5];

    poseGrasp = constraintPoseTarget(eeName);
    poseGrasp.TargetTransform = graspPose;
    poseGrasp.Weights = [0.5, 0.5];

    % Joint constraint
    if nargin < 6
        jointTgt = constraintJointBounds(robot);
    else
        jointTgt = jointBounds;
    end
    
    % Find finger joint indices to keep them open during stages 1 and 2
    fingerJointNames = {'left_gripper_finger_joint1', 'left_gripper_finger_joint2'};
    fingerIndices = [];
    for i = 1:numel(initialGuess)
        if ismember(initialGuess(i).JointName, fingerJointNames)
            fingerIndices = [fingerIndices, i];
        end
    end
    
    % Create joint constraints that lock finger joints to open position (0.05) for stages 1 and 2
    jointTgt_stages12 = jointTgt;
    if ~isempty(fingerIndices)
        for idx = fingerIndices
            jointTgt_stages12.Bounds(idx,:) = [0.05, 0.05]; % Keep fingers open at position 0.05
        end
    end
    
    allDist = [distConstraints{:}];

    % Solve GIK for approach and grasp stages with fingers kept open
    [qApproach, solutionInfo] = gik(initialGuess, poseApproach, jointTgt_stages12, allDist);
    [qGrasp, solutionInfo2] = gik(qApproach, poseGrasp, jointTgt_stages12, allDist);

    % Stage 3: Close fingers for grasping
    qClose = qGrasp;
    
    % --- Calculate required finger position based on bottle diameter ---
    % Based on URDF, finger joints are prismatic with limits [0, 0.05]
    % Position 0.05 = fully open, Position 0.0 = fully closed
    initialFingerGap = 0.0269 + 0.05 * 2; % Gap when joints are at 0.05 (open) - each finger moves 0.05m
    targetFingerGap = bottleDiameter;
    
    % Calculate how much to close the fingers
    % If we want to reduce the gap from initialFingerGap to targetFingerGap,
    % each finger needs to move inward by (initialFingerGap - targetFingerGap) / 2
    fingerClosingDistance = (initialFingerGap - targetFingerGap) / 2;
    fingerJointPos = 0.05 - fingerClosingDistance;
    
    % Clamp the value to be within joint limits [0, 0.05]
    fingerJointPos = max(0, min(0.05, fingerJointPos));
    
    fprintf('Target bottle diameter: %.4f m\n', bottleDiameter);
    fprintf('Initial finger gap (open): %.4f m\n', initialFingerGap);
    fprintf('Target finger gap: %.4f m\n', targetFingerGap);
    fprintf('Finger closing distance per finger: %.4f m\n', fingerClosingDistance);
    fprintf('Calculated finger joint position: %.4f\n', fingerJointPos);
    fprintf('Finger closing distance: %.4f m\n', 0.05 - fingerJointPos);
    
    % Close fingers to the calculated position
    if ~isempty(fingerIndices)
        for idx = fingerIndices
            qClose(idx).JointPosition = fingerJointPos;
        end
        solutionInfo3.Status = 'Success';
        solutionInfo3.ExitFlag = 1;
    else
        warning('Finger joints not found in robot configuration');
        solutionInfo3.Status = 'Warning';
        solutionInfo3.ExitFlag = 0;
    end

    % Interpolate trajectories for all three stages
    n1 = 50; n2 = 50; n3 = 30;
    traj1 = zeros(n1, numel(qApproach));
    traj2 = zeros(n2, numel(qGrasp));
    traj3 = zeros(n3, numel(qClose));

    for j = 1:numel(qApproach)
        traj1(:, j) = linspace(initialGuess(j).JointPosition, qApproach(j).JointPosition, n1);
        traj2(:, j) = linspace(qApproach(j).JointPosition, qGrasp(j).JointPosition, n2);
        traj3(:, j) = linspace(qGrasp(j).JointPosition, qClose(j).JointPosition, n3);
    end

    traj = [traj1; traj2; traj3];
end