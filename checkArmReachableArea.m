
function [eePositions]=checkArmReachableArea(robot, ee_link_name)
% this function is superceded by computeReachableWorkspace

numSamples = 20000;
cfg = homeConfiguration(robot); % check for R1
n = numel(cfg);
jointSamples = rand(numSamples, n);
for i = 1:n
    lower = robot.Bodies{i}.Joint.PositionLimits(1);
    upper = robot.Bodies{i}.Joint.PositionLimits(2);
    jointSamples(:, i) = lower + (upper - lower) * jointSamples(:, i);
end

eePositions = zeros(numSamples, 3);

for i = 1:numSamples
    q = jointSamples(i,:);
    cfgStruct = vectorToConfig(robot, q);  % Us
    T = getTransform(robot, cfgStruct, ee_link_name);  % Replace with your EE name
    eePositions(i,:) = tform2trvec(T);
end

end

