function q_traj = generateConstrainedIKTrajectory(robotB, eeTrajectory, jointLimitsB, varargin)
% generateConstrainedIKTrajectory generates joint trajectories for robotB
% to follow a given end-effector trajectory using GIK with constraints.
%
% Inputs:
%   robotB         - RigidBodyTree robot model
%   eeTrajectory   - 4x4xN array of homogeneous transforms (target poses)
%   jointLimitsB   - Struct with .JointName and .PositionLimits
%
% Optional Name-Value Pairs:
%   'RampSteps'            - Number of steps to ramp into the first pose (default 10)
%   'SmoothnessThreshold'  - Max joint step between frames (default 0.3)
%   'OrientationTolerance' - Orientation tolerance in radians (default 0.3)
%
% Output:
%   q_traj         - NxJ joint configuration trajectory

% --- Parse optional arguments ---
p = inputParser;
addParameter(p, 'RampSteps', 10);
addParameter(p, 'SmoothnessThreshold', 0.3);
addParameter(p, 'OrientationTolerance', 0.3);
parse(p, varargin{:});

rampSteps = p.Results.RampSteps;
smoothThresh = p.Results.SmoothnessThreshold;
orientationTol = p.Results.OrientationTolerance;

% --- Setup GIK solver ---
gik = generalizedInverseKinematics(...
    'RigidBodyTree', robotB, ...
    'ConstraintInputs', {'pose', 'joint'});

gik.SolverAlgorithm = 'LevenbergMarquardt';
% gik.SolverAlgorithm = 'SequentialQuadraticProgramming';

% End-effector name
eeName = robotB.BodyNames{end};  % You can customize this

% Pose constraint
poseTgt = constraintPoseTarget(eeName);
poseTgt.PositionTolerance = 0.05;       % 5 cm tolerance
poseTgt.OrientationTolerance = orientationTol;

% notice that jointLimitsB is in degree and contains both left and right
% arm limits, now need to truncate it and change it to radian
% Create joint limit constraint
jointLim = constraintJointBounds(robotB);

% Only use the first 6 fields (left arm joints)
fields = fieldnames(jointLimitsB);
leftArmFields = fields(1:6);

for i = 1:numel(leftArmFields)
    jointName = leftArmFields{i};
    limitsDeg = jointLimitsB.(jointName);
    limitsRad = deg2rad(limitsDeg);

    % Find the correct index based on joint name
    for k = 1:numel(jointLim.Bounds)
        % Use dynamic field access
        try
            name = getfield(jointLim.Bounds(k), 'JointName');  % Safe access
        catch
            warning('Could not access JointName in Bounds(%d)', k);
            continue;
        end

        if strcmp(name, jointName)
            jointLim.Bounds(k) = updateJointBound(jointLim.Bounds(k), limitsRad);
            break;
        end
    end
end

function bound = updateJointBound(bound, limits)
    % Safely update JointPositionBounds for constraintJointBounds
    try
        bound.JointPositionBounds = limits;
    catch
        warning('Could not assign JointPositionBounds');
    end
end



% --- Initialization ---
cfg_home = homeConfiguration(robotB);
q_prev = [cfg_home.JointPosition];
numJoints = numel(cfg_home);
numWaypoints = size(eeTrajectory, 3);
q_traj = zeros(numWaypoints + rampSteps, numJoints);

% --- IK Loop ---
for t = 1:(numWaypoints + rampSteps)
    if t <= rampSteps
        % Ramp-in: blend home and first pose
        alpha = t / rampSteps;
        tgtPose = eeTrajectory(:, :, 1);
        startPose = getTransform(robotB, cfg_home, eeName);
        blendedPos = alpha * tform2trvec(tgtPose) + (1 - alpha) * tform2trvec(startPose);
        blendedRot = tform2rotm(startPose)^(1 - alpha) * tform2rotm(tgtPose)^alpha;
        blendedPose = trvec2tform(blendedPos) * rotm2tform(blendedRot);
    else
        blendedPose = eeTrajectory(:, :, t - rampSteps);
    end

    % Apply pose constraint
    poseTgt.TargetTransform = blendedPose;

    % Solve IK
    [q_sol, sol_info] = gik(q_prev, poseTgt, jointLim);

    % Enforce smoothness
    dq = norm(q_sol - q_prev);
    if dq > smoothThresh
        q_sol = q_prev + smoothThresh * (q_sol - q_prev) / dq;
    end

    q_traj(t, :) = q_sol;
    q_prev = q_sol;
end
end