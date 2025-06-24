close all

% === Setup ===
% Ensure you have run createAndPlaceBottleTable.m first to generate bottle, table, and cardbox objects
load('bottle.mat'); % bottleInB created by createAndPlaceBottleTable.m
load('table.mat');  % tableBox created by createAndPlaceBottleTable.m
load('cardbox.mat'); % cardbox created by createAndPlaceBottleTable.m
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

% Set robotB to struct format for all configuration handling
robotB = importrobot('fineUrdfs/r1_v2_1_0.urdf');
robotB = attachLeftArmCollisionBoxes(robotB, 'R1Meshes');
robotB.DataFormat = 'struct';
eeName = 'left_gripper_link';

% Arm initial position reference
% Left arm: 1.56, 2.94, -2.54, 0.0, 0.0, 0.0
% Right arm: -1.56, 2.94, -2.54, 0.0, 0.0, 0.0

% === Define Initial Robot Configuration ===
initialGuess = homeConfiguration(robotB);
initialGuess = updateHomePositionforR1_wholeBody(robotB, initialGuess);

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

%% Add cardboard box as a fixed body for collision
cardboxBody = rigidBody("cardboxObstacle");
jntCard = rigidBodyJoint("cardboxFixed","fixed");
cardboxBody.Joint = jntCard;
addCollision(cardboxBody, cardbox);
addBody(robotB, cardboxBody, robotB.BaseName);

%% calculate eePosB
% Calculate reachable workspace points for robotB
% eePosBs = calculateEEPosB(robotB, eeName);

%% visualize robot, bottle, and the table
% Call visualization function
% visualizeRobotScene(robotB, bottleInB, tableBox, eePosBs);

%% After robotB, tableBox, cardbox, and bottleInB are set up
visualizeRobotSceneWithCollisions(robotB, tableBox, cardbox, bottleInB);

%% 2.  Create a distance-bounds constraint
clearance = 0.02; % 2 cm gap
linksToProtect = { ...
    "left_gripper_link", ...
    "left_gripper_finger_link1", ...
    "left_gripper_finger_link2", ...
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

% Add distance constraint for the cardboard box
for k = 1:numel(linksToProtect)
    distCs{end+1} = constraintDistanceBounds( ...
                  linksToProtect{k}, ...
                  "ReferenceBody","cardboxObstacle", ...
                  "Bounds",    [clearance 10] );
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
poseApproach.Weights = [1, 0.5];

poseGrasp = constraintPoseTarget(eeName);
poseGrasp.TargetTransform = bottleInB.graspPose;
poseGrasp.Weights = [1, 0.5];

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

    % Lock all joints except left arm and finger joints
    allowedJoints = [leftArmJointsStr(:); string(fingerJointNames(:))];
    jointBounds = constraintJointBounds(robotB);
    for i = 1:numel(initialGuess)
        jointName = initialGuess(i).JointName;
        if ~ismember(jointName, allowedJoints)
            fixedPos = initialGuess(i).JointPosition;
            jointBounds.Bounds(i,:) = [fixedPos, fixedPos];
        end
    end

    % Find finger joint indices to keep them open during stages 1 and 2
    fingerJointNames = {'left_gripper_finger_joint1', 'left_gripper_finger_joint2'};
    fingerIndices = [];
    for i = 1:numel(initialGuess)
        if ismember(initialGuess(i).JointName, fingerJointNames)
            fingerIndices = [fingerIndices, i];
        end
    end
    % Use jointBounds for jointTgt_stages12 and pass to solveGraspGIK
    jointTgt_stages12 = jointBounds;
    if ~isempty(fingerIndices)
        for idx = fingerIndices
            jointTgt_stages12.Bounds(idx,:) = [0.05, 0.05]; % Keep fingers open at position 0.05
        end
    end

    % Table collision avoidance constraints remain active (allDist)
    allDist = [distCs{:}];

    % Solve GIK with torso and right arm locked
    bottleDiameter = bottleInB.collision.Radius * 2;
    [qApproach, qGrasp, qClose, traj, solutionInfo, solutionInfo2, solutionInfo3] = solveGraspGIK(robotB, eeName, targetPose, distCs, initialGuess, jointTgt_stages12, bottleDiameter);
    
    % === Motion Planning with manipulatorRRT (collision-free path) ===
    robotB.DataFormat = 'row'; % Switch to row format for planner
    obstacles = {tableBox, cardbox, bottleInB.collision};
    planner = manipulatorRRT(robotB, obstacles);
    planner.MaxConnectionDistance = 0.1;
    planner.MaxIterations = 3000;

    qStart = [initialGuess.JointPosition];
    qGoal = [qGrasp.JointPosition];
    disp(class(robotB)); % Should print 'rigidBodyTree'
    keyboard
    collidingPairs = tempCheckSelfCollision(robotB, qStart);
    [pathObj, solnInfo] = plan(planner, qStart, qGoal);

    robotB.DataFormat = 'struct'; % Restore struct format after planning

    if isempty(pathObj.States)
        warning('No collision-free path found by the planner (with bottle as obstacle).');
    else
        disp('Collision-free path found by manipulatorRRT (with bottle as obstacle). Visualizing...');
        figure; hold on; axis equal; grid on;
        show(tableBox);
        show(cardbox);
        show(bottleInB.collision);
        for i = 1:size(pathObj.States,1)
            show(robotB, pathObj.States(i,:), 'PreservePlot', false, 'Collisions', 'on');
            drawnow;
        end
        title('Planned Collision-Free Path (Approach, Bottle as Obstacle)');
    end
    
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

    % Lock all joints except left arm and finger joints
    allowedJoints = [leftArmJointsStr(:); string(fingerJointNames(:))];
    jointBounds = constraintJointBounds(robotB);
    for i = 1:numel(initialGuess)
        jointName = initialGuess(i).JointName;
        if ~ismember(jointName, allowedJoints)
            fixedPos = initialGuess(i).JointPosition;
            jointBounds.Bounds(i,:) = [fixedPos, fixedPos];
        end
    end

    % Find finger joint indices to keep them open during stages 1 and 2
    fingerJointNames = {'left_gripper_finger_joint1', 'left_gripper_finger_joint2'};
    fingerIndices = [];
    for i = 1:numel(initialGuess)
        if ismember(initialGuess(i).JointName, fingerJointNames)
            fingerIndices = [fingerIndices, i];
        end
    end
    % Use jointBounds for jointTgt_stages12 and pass to solveGraspGIK
    jointTgt_stages12 = jointBounds;
    if ~isempty(fingerIndices)
        for idx = fingerIndices
            jointTgt_stages12.Bounds(idx,:) = [0.05, 0.05]; % Keep fingers open at position 0.05
        end
    end

    % Table collision avoidance constraints remain active (allDist)
    allDist = [distCs{:}];

    % Final two-stage GIK with updated torso
    bottleDiameter = bottleInB.collision.Radius * 2;
    [qApproach, qGrasp, qClose, traj, solutionInfo, solutionInfo2, solutionInfo3] = ...
        solveGraspGIK(robotB, eeName, targetPose, distCs, initialGuess, jointTgt_stages12, bottleDiameter);
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

% Print x difference between finger center and bottle center
finger_center_x = (pos_finger1(1) + pos_finger2(1)) / 2;
x_diff = finger_center_x - pos_bottle(1);
fprintf('X difference (finger center - bottle center): %.6f m\n', x_diff);

% Print y difference between finger center and bottle center
finger_center_y = (pos_finger1(2) + pos_finger2(2)) / 2;
y_diff = finger_center_y - pos_bottle(2);
fprintf('Y difference (finger center - bottle center): %.6f m\n', y_diff);

%%
% === Animate (Enhanced Visualization) ===
animSpeed = 0.2; % Animation speed (seconds per frame)

% --- Video export setup ---
v = VideoWriter('robot_grasp_animation.mp4', 'MPEG-4');
v.FrameRate = 1/animSpeed;
open(v);

figure('Name','R1 Arm Grasp Animation','Position',[100 100 1200 700]);
ax = axes; view(45,30); axis equal; grid on; hold on;

% Draw table as semi-transparent box
if exist('tableBox','var')
    [Xb,Yb,Zb] = meshgrid([-0.5 0.5]*tableBox.X, [-0.5 0.5]*tableBox.Y, [0 1]*tableBox.Z);
    Xb = Xb(:) + tableBox.Pose(1,4);
    Yb = Yb(:) + tableBox.Pose(2,4);
    Zb = Zb(:) + tableBox.Pose(3,4);
    fill3(Xb([1 2 4 3]),Yb([1 2 4 3]),Zb([1 2 4 3]),[0.8 0.8 0.8],'FaceAlpha',0.3,'EdgeColor','none');
end

% Draw cardboard box
if exist('cardbox','var')
    show(cardbox, 'Parent', ax);
end

% Draw bottle as transparent cylinder
bottle_radius = bottleInB.collision.Radius;
bottle_height = bottleInB.collision.Length;
pos_bottle = tform2trvec(bottleInB.pose);
[bx,by,bz] = cylinder(bottle_radius,30);
bz = bz * bottle_height + pos_bottle(3);
surf(bx+pos_bottle(1), by+pos_bottle(2), bz, 'FaceAlpha',0.3, 'EdgeColor','none', 'FaceColor',[0.2 0.5 1]);

% Mark approach, grasp, and close poses
plot3(pos_bottle(1),pos_bottle(2),pos_bottle(3)+bottle_height/2,'ko','MarkerSize',10,'MarkerFaceColor','y');

% Preallocate trajectory arrays
nTotal = size(traj,1);
gripper_traj = zeros(nTotal,3);
finger1_traj = zeros(nTotal,3);
finger2_traj = zeros(nTotal,3);
finger_center_traj = zeros(nTotal,3);
x_diff_traj = zeros(nTotal,1);
y_diff_traj = zeros(nTotal,1);

% Ensure DataFormat is 'row' for collision checking
robotB.DataFormat = 'row';
qInitVec = homeConfiguration(robotB); % returns a vector in 'row' format
% --- Use single output for attached obstacles (table/cardbox as rigid bodies) ---
inCollision = checkCollision(robotB, qInitVec, 'Exhaustive', 'on');
if inCollision
    disp('Initial configuration is in collision.');
else
    disp('Initial configuration is collision-free.');
end
figure; hold on; axis equal; grid on;
show(robotB, qInitVec, 'PreservePlot', false);
show(cardbox);
show(tableBox);
title('Initial Robot Configuration with Obstacles');

for t = 1:nTotal
    qNow = initialGuess;
    for j = 1:numel(qNow)
        qNow(j).JointPosition = traj(t,j);
    end
    qVec = [qNow.JointPosition];
    % --- Use single output for external objects (cardbox, tableBox) ---
    inCollision = checkCollision(robotB, qVec, {cardbox, tableBox});
    if inCollision
        warning('Collision detected with cardbox or table at step %d! Animation halted.', t);
        keyboard
    end
    T_gripper = getTransform(robotB, qNow, 'left_gripper_link');
    T_finger1 = getTransform(robotB, qNow, 'left_gripper_finger_link1');
    T_finger2 = getTransform(robotB, qNow, 'left_gripper_finger_link2');
    pos_gripper = tform2trvec(T_gripper);
    pos_finger1 = tform2trvec(T_finger1);
    pos_finger2 = tform2trvec(T_finger2);
    gripper_traj(t,:) = pos_gripper;
    finger1_traj(t,:) = pos_finger1;
    finger2_traj(t,:) = pos_finger2;
    finger_center_traj(t,:) = (pos_finger1 + pos_finger2)/2;
    x_diff_traj(t) = finger_center_traj(t,1) - pos_bottle(1);
    y_diff_traj(t) = finger_center_traj(t,2) - pos_bottle(2);
end

% Plot full trajectory paths
plot3(gripper_traj(:,1),gripper_traj(:,2),gripper_traj(:,3),'r--','LineWidth',1.5);
plot3(finger1_traj(:,1),finger1_traj(:,2),finger1_traj(:,3),'g-.','LineWidth',1);
plot3(finger2_traj(:,1),finger2_traj(:,2),finger2_traj(:,3),'g-.','LineWidth',1);
plot3(finger_center_traj(:,1),finger_center_traj(:,2),finger_center_traj(:,3),'b-','LineWidth',2);

% Animation loop
for t = 1:nTotal
    qNow = initialGuess;
    for j = 1:numel(qNow)
        qNow(j).JointPosition = traj(t,j);
    end
    qVec = [qNow.JointPosition];
    % --- Use single output for external objects (cardbox, tableBox) ---
    inCollision = checkCollision(robotB, qVec, {cardbox, tableBox});
    if inCollision
        warning('Collision detected with cardbox or table at step %d! Animation halted.', t);
        break;
    end
    cla(ax);
    % Table
    if exist('tableBox','var')
        fill3(Xb([1 2 4 3]),Yb([1 2 4 3]),Zb([1 2 4 3]),[0.8 0.8 0.8],'FaceAlpha',0.3,'EdgeColor','none');
    end
    % Cardboard box
    if exist('cardbox','var')
        show(cardbox, 'Parent', ax);
    end
    % Bottle
    surf(bx+pos_bottle(1), by+pos_bottle(2), bz, 'FaceAlpha',0.3, 'EdgeColor','none', 'FaceColor',[0.2 0.5 1]);
    % Draw bottle top as a filled circle for top view
    theta = linspace(0, 2*pi, 30);
    bottle_top_x = pos_bottle(1) + bottle_radius * cos(theta);
    bottle_top_y = pos_bottle(2) + bottle_radius * sin(theta);
    bottle_top_z = pos_bottle(3) + bottle_height;
    fill3(bottle_top_x, bottle_top_y, bottle_top_z*ones(size(bottle_top_x)), [0.2 0.5 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    % Draw bottle bottom as a filled circle for top view
    bottle_bottom_x = pos_bottle(1) + bottle_radius * cos(theta);
    bottle_bottom_y = pos_bottle(2) + bottle_radius * sin(theta);
    bottle_bottom_z = pos_bottle(3);
    fill3(bottle_bottom_x, bottle_bottom_y, bottle_bottom_z*ones(size(bottle_bottom_x)), [0.2 0.5 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    % Trajectories
    plot3(gripper_traj(1:t,1),gripper_traj(1:t,2),gripper_traj(1:t,3),'r--','LineWidth',1.5);
    plot3(finger1_traj(1:t,1),finger1_traj(1:t,2),finger1_traj(1:t,3),'g-.','LineWidth',1);
    plot3(finger2_traj(1:t,1),finger2_traj(1:t,2),finger2_traj(1:t,3),'g-.','LineWidth',1);
    plot3(finger_center_traj(1:t,1),finger_center_traj(1:t,2),finger_center_traj(1:t,3),'b-','LineWidth',2);
    % Robot
    show(robotB, qNow, 'PreservePlot', false, 'Parent', ax);
    % Stage markers
    if t==1
        title('Stage 1: Approach','FontSize',14,'Color','b');
    elseif t==round(nTotal/2)
        title('Stage 2: Grasp','FontSize',14,'Color','m');
    elseif t==nTotal
        title('Stage 3: Close','FontSize',14,'Color','r');
    end
    % Finger center to bottle center line
    plot3([finger_center_traj(t,1) pos_bottle(1)], [finger_center_traj(t,2) pos_bottle(2)], [finger_center_traj(t,3) pos_bottle(3)], 'k:', 'LineWidth', 2);
    % Real-time metrics
    txt = sprintf('X diff: %.3f m\nY diff: %.3f m', x_diff_traj(t), y_diff_traj(t));
    text(finger_center_traj(t,1), finger_center_traj(t,2), finger_center_traj(t,3)+0.05, txt, 'FontSize', 10, 'Color', 'k');
    % Camera preset (isometric)
    view(45,30);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; grid on;
    pause(animSpeed);
    % --- Capture and write video frame ---
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% --- Close video file ---
close(v);
disp('Video saved as robot_grasp_animation.mp4');

% === Top-down view video export ===
v2 = VideoWriter('robot_grasp_topview.mp4', 'MPEG-4');
v2.FrameRate = 1/animSpeed;
open(v2);

figure('Name','R1 Arm Grasp Animation (Top View)','Position',[100 100 1200 700]);
ax2 = axes; view(0,90); axis equal; grid on; hold on;

% Draw table as semi-transparent box
if exist('tableBox','var')
    fill3(Xb([1 2 4 3]),Yb([1 2 4 3]),Zb([1 2 4 3]),[0.8 0.8 0.8],'FaceAlpha',0.3,'EdgeColor','none');
end
% Draw cardboard box
if exist('cardbox','var')
    show(cardbox, 'Parent', ax2);
end
% Draw bottle as transparent cylinder
surf(bx+pos_bottle(1), by+pos_bottle(2), bz, 'FaceAlpha',0.3, 'EdgeColor','none', 'FaceColor',[0.2 0.5 1]);
plot3(pos_bottle(1),pos_bottle(2),pos_bottle(3)+bottle_height/2,'ko','MarkerSize',10,'MarkerFaceColor','y');

for t = 1:nTotal
    qNow = initialGuess;
    for j = 1:numel(qNow)
        qNow(j).JointPosition = traj(t,j);
    end
    qVec = [qNow.JointPosition];
    % --- Use single output for external objects (cardbox, tableBox) ---
    inCollision = checkCollision(robotB, qVec, {cardbox, tableBox});
    if inCollision
        warning('Collision detected with cardbox or table at step %d! Animation halted.', t);
        break;
    end
    cla(ax2);
    % Table
    if exist('tableBox','var')
        fill3(Xb([1 2 4 3]),Yb([1 2 4 3]),Zb([1 2 4 3]),[0.8 0.8 0.8],'FaceAlpha',0.3,'EdgeColor','none');
    end
    % Cardboard box
    if exist('cardbox','var')
        show(cardbox, 'Parent', ax2);
    end
    % Bottle
    surf(bx+pos_bottle(1), by+pos_bottle(2), bz, 'FaceAlpha',0.3, 'EdgeColor','none', 'FaceColor',[0.2 0.5 1]);
    % Draw bottle top as a filled circle for top view
    theta = linspace(0, 2*pi, 30);
    bottle_top_x = pos_bottle(1) + bottle_radius * cos(theta);
    bottle_top_y = pos_bottle(2) + bottle_radius * sin(theta);
    bottle_top_z = pos_bottle(3) + bottle_height;
    fill3(bottle_top_x, bottle_top_y, bottle_top_z*ones(size(bottle_top_x)), [0.2 0.5 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    % Draw bottle bottom as a filled circle for top view
    bottle_bottom_x = pos_bottle(1) + bottle_radius * cos(theta);
    bottle_bottom_y = pos_bottle(2) + bottle_radius * sin(theta);
    bottle_bottom_z = pos_bottle(3);
    fill3(bottle_bottom_x, bottle_bottom_y, bottle_bottom_z*ones(size(bottle_bottom_x)), [0.2 0.5 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    % Trajectories
    plot3(gripper_traj(1:t,1),gripper_traj(1:t,2),gripper_traj(1:t,3),'r--','LineWidth',1.5);
    plot3(finger1_traj(1:t,1),finger1_traj(1:t,2),finger1_traj(1:t,3),'g-.','LineWidth',1);
    plot3(finger2_traj(1:t,1),finger2_traj(1:t,2),finger2_traj(1:t,3),'g-.','LineWidth',1);
    plot3(finger_center_traj(1:t,1),finger_center_traj(1:t,2),finger_center_traj(1:t,3),'b-','LineWidth',2);
    % Robot
    show(robotB, qNow, 'PreservePlot', false, 'Parent', ax2);
    % Stage markers
    if t==1
        title('Stage 1: Approach (Top View)','FontSize',14,'Color','b');
    elseif t==round(nTotal/2)
        title('Stage 2: Grasp (Top View)','FontSize',14,'Color','m');
    elseif t==nTotal
        title('Stage 3: Close (Top View)','FontSize',14,'Color','r');
    end
    % Finger center to bottle center line
    plot3([finger_center_traj(t,1) pos_bottle(1)], [finger_center_traj(t,2) pos_bottle(2)], [finger_center_traj(t,3) pos_bottle(3)], 'k:', 'LineWidth', 2);
    % Real-time metrics
    txt = sprintf('X diff: %.3f m\nY diff: %.3f m', x_diff_traj(t), y_diff_traj(t));
    text(finger_center_traj(t,1), finger_center_traj(t,2), finger_center_traj(t,3)+0.05, txt, 'FontSize', 10, 'Color', 'k');
    % Camera preset (top-down)
    view(0,90);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; grid on;
    pause(animSpeed);
    % --- Capture and write video frame ---
    frame2 = getframe(gcf);
    writeVideo(v2, frame2);
end

close(v2);
disp('Top-down video saved as robot_grasp_topview.mp4');

% Save final configuration for visualization
save('final_configuration.mat', 'qClose', 'bottleInB', 'tableBox');
fprintf('Final configuration saved to final_configuration.mat\n');

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

    % --- Finger Position Calculation (as before) ---
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
    
    % --- Define Virtual Target for the Gripper Link ---
    % The `graspPose` is the target for the point of contact on the bottle.
    % We need to calculate a "virtual" target for `eeName` (the gripper link)
    % so that its fingers end up aligned with the graspPose.
    
    % Compute the midpoint between the two fingers for the grasp point
    y1 = 0.013453 + fingerJointPos;
    y2 = -0.013453 - fingerJointPos;
    y_mid = (y1 + y2) / 2;
    finger_link_length = 0.045; % Finer adjustment for length from joint to tip
    x_offset = 0.03689 + finger_link_length;
    T_gripper_to_finger_mid = trvec2tform([x_offset, y_mid, 0.00012067]);
    
    % To make the finger midpoint reach graspPose, the gripper must target a pose "backed off" by this transform.
    virtualGraspPose = graspPose * inv(T_gripper_to_finger_mid);
    fprintf('INFO: Gripper target pose adjusted with updated X offset (midpoint) to position fingers correctly.\n');

    % --- Define Approach and Grasp Constraints ---
    % Define approach pose relative to the gripper's final (virtual) target pose
    approachOffset = trvec2tform([0, 0, -0.1]);
    approachPose = virtualGraspPose * approachOffset;

    % Pose constraints for the gripper link (eeName)
    poseApproach = constraintPoseTarget(eeName);
    poseApproach.TargetTransform = approachPose;
    poseApproach.Weights = [1, 0.5];

    poseGrasp = constraintPoseTarget(eeName);
    poseGrasp.TargetTransform = virtualGraspPose;
    poseGrasp.Weights = [1, 0.5];

    % Find finger joint indices to keep them open during stages 1 and 2
    fingerJointNames = {'left_gripper_finger_joint1', 'left_gripper_finger_joint2'};
    fingerIndices = [];
    for i = 1:numel(initialGuess)
        if ismember(initialGuess(i).JointName, fingerJointNames)
            fingerIndices = [fingerIndices, i];
        end
    end
    % Create joint constraints that lock finger joints to open position (0.05) for stages 1 and 2
    jointTgt_stages12 = jointBounds;
    if ~isempty(fingerIndices)
        for idx = fingerIndices
            jointTgt_stages12.Bounds(idx,:) = [0.05, 0.05]; % Keep fingers open at position 0.05
        end
    end

    % Table collision avoidance constraints remain active (allDist)
    allDist = [distConstraints{:}];

    % Solve GIK for approach and grasp stages with fingers kept open
    [qApproach, solutionInfo] = gik(initialGuess, poseApproach, jointTgt_stages12, allDist);
    [qGrasp, solutionInfo2] = gik(qApproach, poseGrasp, jointTgt_stages12, allDist);

    % Stage 3: Close fingers for grasping
    qClose = qGrasp;
    
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

    % Print minimum distance between finger center and bottle after each stage
    T_finger1_approach = getTransform(robot, qApproach, 'left_gripper_finger_link1');
    T_finger2_approach = getTransform(robot, qApproach, 'left_gripper_finger_link2');
    pos_finger1_approach = tform2trvec(T_finger1_approach);
    pos_finger2_approach = tform2trvec(T_finger2_approach);
    finger_center_approach = (pos_finger1_approach + pos_finger2_approach) / 2;
    bottle_center = tform2trvec(graspPose);
    dist_approach = norm(finger_center_approach(1:2) - bottle_center(1:2));
    fprintf('Approach stage: min XY distance (finger center - bottle center): %.6f m\n', dist_approach);

    T_finger1_grasp = getTransform(robot, qGrasp, 'left_gripper_finger_link1');
    T_finger2_grasp = getTransform(robot, qGrasp, 'left_gripper_finger_link2');
    pos_finger1_grasp = tform2trvec(T_finger1_grasp);
    pos_finger2_grasp = tform2trvec(T_finger2_grasp);
    finger_center_grasp = (pos_finger1_grasp + pos_finger2_grasp) / 2;
    dist_grasp = norm(finger_center_grasp(1:2) - bottle_center(1:2));
    fprintf('Grasp stage: min XY distance (finger center - bottle center): %.6f m\n', dist_grasp);
end

% === End of main script ===

% Plot final positions for summary visualization
plotFinalPositions;