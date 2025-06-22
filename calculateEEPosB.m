function eePosB = calculateEEPosB(robotB, endEffectorName)
    % Calculate reachable workspace points for robotB
    % 
    % Input:
    %   robotB - RigidBodyTree object for robotB
    %   endEffectorName - Name of the end-effector body
    % 
    % Output:
    %   eePosB - End-effector positions in robotB's base frame (numPoints x 3)
    
    % Set data format
    robotB.DataFormat = 'struct';
    
    % Get movable joints
    cfgTemplate = homeConfiguration(robotB);
    n = numel(cfgTemplate);
    jointNames = {cfgTemplate.JointName};
    
    % Extract joint limits for movable joints
    lower = zeros(1, n);
    upper = zeros(1, n);
    
    for i = 1:n
        jointName = jointNames{i};
        found = false;
        
        for b = 1:length(robotB.Bodies)
            joint = robotB.Bodies{b}.Joint;
            if strcmp(joint.Name, jointName)
                limits = joint.PositionLimits;
                % Handle infinite limits for continuous joints
                if isinf(limits(1))
                    limits(1) = -pi;
                end
                if isinf(limits(2))
                    limits(2) = pi;
                end
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
    numSamples = 5000; % Number of samples
    qSamples = rand(numSamples, n) .* (upper - lower) + lower;
    
    % Compute end-effector positions
    eePosB = zeros(numSamples, 3);
    for i = 1:numSamples
        cfg = cfgTemplate;
        for j = 1:n
            cfg(j).JointPosition = qSamples(i, j);
        end
        
        try
            T = getTransform(robotB, cfg, endEffectorName);
            eePosB(i, :) = tform2trvec(T);
        catch
            eePosB(i, :) = [NaN, NaN, NaN];  % handle invalid samples
        end
    end
    
    % Remove invalid samples
    eePosB = eePosB(~any(isnan(eePosB), 2), :);
end
