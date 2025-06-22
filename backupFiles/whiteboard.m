cfg = initialGuess;  % or however you defined it

for i = 1:length(cfg)
    j = robotB.Bodies{i}.Joint;
    if isprop(j, 'PositionLimits')
        limits = j.PositionLimits;
        val = cfg(i).JointPosition;
        if val < limits(1) || val > limits(2)
            fprintf('⚠️ Joint %s: %.3f out of [%.3f, %.3f]\n', ...
                j.Name, val, limits(1), limits(2));
        end
    end
end