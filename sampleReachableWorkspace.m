function eePositions = sampleReachableWorkspace(robot, endEffectorName, numSamples, visualize, jointsToSample)
% sampleReachableWorkspace Samples joint configurations and computes reachable EE positions.
%
% Inputs:
%   robot           - rigidBodyTree model
%   endEffectorName - name of end-effector link (string)
%   numSamples      - number of joint samples (default: 5000)
%   visualize       - (optional) true to plot the workspace (default: false)
%   jointsToSample  - (optional) cell array of joint names to sample (others locked at home)
%
% Output:
%   eePositions     - Nx3 matrix of reachable end-effector positions

    if nargin < 3 || isempty(numSamples)
        numSamples = 5000;
    end
    if nargin < 4
        visualize = false;
    end
    if nargin < 5
        jointsToSample = [];
    end

    robot.DataFormat = 'struct';
    cfgTemplate = homeConfiguration(robot);
    n = numel(cfgTemplate);
    jointNames = {cfgTemplate.JointName};

    % If jointsToSample is empty, sample all joints (backward compatible)
    if isempty(jointsToSample)
        jointsToSample = jointNames;
    end

    % Find indices of joints to sample
    sampleIdx = find(ismember(jointNames, jointsToSample));
    nSample = numel(sampleIdx);

    lower = zeros(1, nSample);
    upper = zeros(1, nSample);
    for k = 1:nSample
        jointName = jointsToSample{k};
        found = false;
        for b = 1:length(robot.Bodies)
            joint = robot.Bodies{b}.Joint;
            if strcmp(joint.Name, jointName)
                limits = joint.PositionLimits;
                % Handle infinite limits for continuous joints
                if isinf(limits(1))
                    limits(1) = -pi;
                end
                if isinf(limits(2))
                    limits(2) = pi;
                end
                lower(k) = limits(1);
                upper(k) = limits(2);
                found = true;
                break;
            end
        end
        if ~found
            error('Joint "%s" not found in robot.Bodies.', jointName);
        end
    end

    qSamples = rand(numSamples, nSample) .* (upper - lower) + lower;
    eePositions = zeros(numSamples, 3);
    for i = 1:numSamples
        cfg = cfgTemplate;
        % Set sampled joints
        for j = 1:nSample
            cfg(sampleIdx(j)).JointPosition = qSamples(i, j);
        end
        % All other joints remain at home position
        try
            T = getTransform(robot, cfg, endEffectorName);
            eePositions(i, :) = tform2trvec(T);
        catch
            eePositions(i, :) = [NaN, NaN, NaN];
        end
    end
    eePositions = eePositions(~any(isnan(eePositions), 2), :);

    if visualize
        figure; hold on; view(3); grid on;
        scatter3(eePositions(:,1), eePositions(:,2), eePositions(:,3), 4, 'filled');
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title(['Reachable Workspace of ', endEffectorName]);
        axis equal;
    end
end 