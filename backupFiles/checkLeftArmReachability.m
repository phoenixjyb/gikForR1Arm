function [reachable, qSol, info, lockTorso] = checkLeftArmReachability(robot, targetPose, torsoJoints, leftArmJoints)
% checkLeftArmReachability  Check if left arm can reach target with torso locked
%
%   [reachable, qSol, info, lockTorso] = checkLeftArmReachability(robot, targetPose, ...
%       torsoJoints, leftArmJoints)
%
%   Inputs
%   ------
%   robot        : rigidBodyTree, full-body robot model (DataFormat 'row').
%   targetPose   : 4x4 homogeneous transform (target grasp pose).
%   torsoJoints  : cellstr, names of torso joints to lock.
%   leftArmJoints: cellstr, names of the left arm joints to move.
%
%   Outputs
%   -------
%   reachable : true if arm alone can reach, false otherwise.
%   qSol      : joint configuration solution (if reachable).
%   info      : solver info struct from generalizedInverseKinematics.
%   lockTorso : constraintJointBounds object locking the torso joints (if reachable), [] otherwise.

arguments
    robot
    targetPose (4,4) double
    torsoJoints cell
    leftArmJoints cell
end

% Set up the generalized inverse kinematics solver
gik = generalizedInverseKinematics( ...
    'RigidBodyTree', robot, ...
    'ConstraintInputs', {'pose', 'joint'});

% Define the target pose constraint for the left arm end-effector
poseTgt = constraintPoseTarget('left_gripper_link');
poseTgt.TargetTransform = targetPose;
poseTgt.Weights = [1 1];

% Lock torso joints (fix them at zero)
lockTorso = constraintJointBounds(robot);
jointConfig = homeConfiguration(robot);
jointLimits = zeros(numel(jointConfig), 2);
for i = 1:numel(jointConfig)
    jointLimits(i, :) = robot.Bodies{i}.Joint.PositionLimits;
end
lockTorso.Bounds = jointLimits;  % Use actual joint limits
jointNames = cellfun(@(b) b.Joint.Name, robot.Bodies, 'UniformOutput', false);
idx = find(ismember(string(jointNames), string(torsoJoints)));
lockTorso.Bounds(idx,:) = repmat([0 0], numel(idx), 1);  % lock torso


% Set initial configuration to home position
qHome = homeConfiguration(robot);

% Run the solver
[qSol, info] = gik(qHome, poseTgt, lockTorso);

% Determine reachability
reachable = info.ExitFlag > 0;

% If not reachable, try again with torso unlocked
if ~reachable
    % Unlock torso joints to assist reachability
    unlockTorso = constraintJointBounds(robot);
    unlockTorso.Bounds = jointLimits;
    unlockTorso.Bounds(idx,:) = repmat([-pi pi], numel(idx), 1);  % allow torso joints to move

    [qSol, info] = gik(qHome, poseTgt, unlockTorso);
    reachable = info.ExitFlag > 0;

    % If reachable after unlocking torso, lock torso to that pose
    if reachable
        lockTorso = constraintJointBounds(robot);
        lockTorso.Bounds = jointLimits;
        for i = 1:numel(idx)
            jointIdx = idx(i);
            lockTorso.Bounds(jointIdx, :) = [qSol(jointIdx), qSol(jointIdx)];
        end
    else
        lockTorso = [];
    end
end

end
