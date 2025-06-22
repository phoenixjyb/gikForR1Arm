% 
close all

% === Setup ===
load ("bottle.mat") % this bottle.mat is created by createBottle.m file
load ('table.mat');% also need to load a table
robotB = importrobot ('r1_v2_1_0.urdf'); % this imports the whole body of R1
% will need to add joint intial position for robotB so that arms are down
robotB = updateHomePositionforR1_wb(robotB); % update home position based on Galaxea robot's arm down config.
robotB.BaseName = 'base_link'; % for the whole body, this is already defined...
eeName = robotB.BodyNames{end};

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


% === Solve GIK for Approach and Grasp ===
initialGuess = homeConfiguration(robotB);
initialGuess = updateHomePositionforR1(initialGuess); % update home position based on Galaxea robot's arm down config.

allDist = [distCs{:}]; 

[qApproach, solutionInfo] = gik(initialGuess, poseApproach, jointTgt,allDist);
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