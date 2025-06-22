function [eePositions] = computeReachableWorkspace(robot, endEffectorName, numSamples)
% COMPUTEREACHABLEWORKSPACE Samples joint configurations and visualizes reachable EE positions.
%
% Inputs:
%   robot           - rigidBodyTree model
%   endEffectorName - name of end-effector link (string)
%   numSamples      - number of joint samples (default: 5000)
%
% Output:
%   eePositions     - Nx3 matrix of reachable end-effector positions

    if nargin < 3
        numSamples = 5000;
    end

    % Set data format
    robot.DataFormat = 'struct';

    % Get movable joints
    cfgTemplate = homeConfiguration(robot);
    n = numel(cfgTemplate);
    jointNames = {cfgTemplate.JointName};

    % Extract joint limits for movable joints (without getJoint)
    lower = zeros(1, n);
    upper = zeros(1, n);

    for i = 1:n
        jointName = jointNames{i};
        found = false;

        for b = 1:length(robot.Bodies)
            joint = robot.Bodies{b}.Joint;
            if strcmp(joint.Name, jointName)
                limits = joint.PositionLimits;
                lower(i) = limits(1);
                upper(i) = limits(2);
                found = true;
                break;
            end
        end

        if ~found
            error('Joint "%s" not found in robot.Bodies.', jointName);
        end
    end

    % Sample joint configurations
    qSamples = rand(numSamples, n) .* (upper - lower) + lower;

    % Compute end-effector positions
    eePositions = zeros(numSamples, 3);
    for i = 1:numSamples
        cfg = cfgTemplate;
        for j = 1:n
            cfg(j).JointPosition = qSamples(i, j);
        end

        try
            T = getTransform(robot, cfg, endEffectorName);
            eePositions(i, :) = tform2trvec(T);
        catch
            eePositions(i, :) = [NaN, NaN, NaN];  % handle invalid samples
        end
    end

    % Remove invalid samples
    eePositions = eePositions(~any(isnan(eePositions), 2), :);

    % Visualization, can be skipped
    %{
    figure; hold on; view(3); grid on;
    scatter3(eePositions(:,1), eePositions(:,2), eePositions(:,3), 4, 'filled');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(['Reachable Workspace of ', endEffectorName]);
    axis equal;
    %}
end