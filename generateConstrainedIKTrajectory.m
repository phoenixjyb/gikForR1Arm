function q_traj = generateConstrainedIKTrajectory(robotB, eeTrajectory, jointLimitsB, varargin)
% generateConstrainedIKTrajectory generates joint trajectories for robotB
% to follow a given end-effector trajectory using GIK with constraints.
%
% Inputs:
%   robotB         - RigidBodyTree robot model
%   eeTrajectory   - 4x4xN array of homogeneous transforms (target poses)
%   jointLimitsB   - struct with fields named after joints (first 6 only)
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
gik.SolverAlgorithm = 'BFGSGradientProjection';

% End-effector name
eeName = robotB.BodyNames{end};

% Pose constraint
poseTgt = constraintPoseTarget(eeName);
poseTgt.PositionTolerance = 0.05;       % 5 cm tolerance
poseTgt.OrientationTolerance = orientationTol;

% Joint limits constraint
jointLim = constraintJointBounds(robotB);
fields = fieldnames(jointLimitsB);
leftArmFields = fields(1:6);

for i = 1:numel(leftArmFields)
    jointName = leftArmFields{i};
    limitsDeg = jointLimitsB.(jointName);
    limitsRad = deg2rad(limitsDeg);

    for k = 1:numel(jointLim.Bounds)
        try
            name = getfield(jointLim.Bounds(k), 'JointName');
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

% --- Initialization ---
cfg_home = homeConfiguration(robotB);
q_prev = [cfg_home.JointPosition];
numJoints = numel(cfg_home);
numWaypoints = size(eeTrajectory, 3);
q_traj = zeros(numWaypoints + rampSteps, numJoints);

% --- GIK Loop ---
for t = 1:(numWaypoints + rampSteps)
    if t <= rampSteps
        alpha = t / rampSteps;
        tgtPose = eeTrajectory(:, :, 1);
        startPose = getTransform(robotB, cfg_home, eeName);
        blendedPos = alpha * tform2trvec(tgtPose) + (1 - alpha) * tform2trvec(startPose);

        R1 = tform2rotm(startPose);
        R2 = tform2rotm(tgtPose);
        q1 = rotm2quat(R1);
        q2 = rotm2quat(R2);
        qInterp = slerpQuaternion(q1, q2, alpha);

        blendedPose = trvec2tform(blendedPos) * quat2tform(qInterp);
    else
        blendedPose = eeTrajectory(:, :, t - rampSteps);
    end

    poseTgt.TargetTransform = blendedPose;
    [q_struct, ~] = gik(structFromVector(robotB, q_prev), poseTgt, jointLim);
    q_sol = [q_struct.JointPosition];

    dq = norm(q_sol - q_prev);
    if dq > smoothThresh
        q_sol = q_prev + smoothThresh * (q_sol - q_prev) / dq;
    end

    q_traj(t, :) = q_sol;
    q_prev = q_sol;
end
end


%% helper functions

function bound = updateJointBound(bound, limits)
    try
        bound.JointPositionBounds = limits;
    catch
        warning('Could not assign JointPositionBounds');
    end
end

function qInterp = slerpQuaternion(q1, q2, t)
    q1 = q1 / norm(q1);
    q2 = q2 / norm(q2);
    dotProd = dot(q1, q2);

    if dotProd < 0.0
        q2 = -q2;
        dotProd = -dotProd;
    end

    if dotProd > 0.9995
        qInterp = (1 - t) * q1 + t * q2;
        qInterp = qInterp / norm(qInterp);
        return;
    end

    theta = acos(dotProd);
    sinTheta = sin(theta);
    qInterp = (sin((1 - t) * theta) * q1 + sin(t * theta) * q2) / sinTheta;
end

function configStruct = structFromVector(robot, qvec)
    cfgTemplate = homeConfiguration(robot);
    for i = 1:numel(cfgTemplate)
        cfgTemplate(i).JointPosition = qvec(i);
    end
    configStruct = cfgTemplate;
end