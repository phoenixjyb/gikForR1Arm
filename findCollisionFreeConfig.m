function initialGuess = findCollisionFreeConfig(robotB, initialGuess, tableBox, bottle, cardbox)
% findCollisionFreeConfig - Search for a collision-free initial configuration
%   initialGuess = findCollisionFreeConfig(robotB, initialGuess, tableBox, bottle, cardbox)
%   Tries random perturbations of the initial guess to find a collision-free config.

qStart = [initialGuess.JointPosition];
maxTries = 500;
found = false;
for i = 1:maxTries
    qTest = qStart + 0.05*randn(size(qStart));
    [isColliding1, sep1] = checkCollision(robotB, qTest, {tableBox}, 'Exhaustive', 'on');
    [isColliding2, sep2] = checkCollision(robotB, qTest, {bottle}, 'Exhaustive', 'on');
    [isColliding3, sep3] = checkCollision(robotB, qTest, {cardbox}, 'Exhaustive', 'on');
    minSep = min([min(sep1(:)), min(sep2(:)), min(sep3(:))]);
    disp(['Try ', num2str(i), ': min sep = ', num2str(minSep)]);
    if ~any(isColliding1) && ~any(isColliding2) && ~any(isColliding3)
        disp('Found a collision-free configuration:');
        disp(qTest);
        % Update struct config for GIK/IK
        robotB.DataFormat = 'struct';
        for j = 1:numel(initialGuess)
            initialGuess(j).JointPosition = qTest(j);
        end
        found = true;
        return;
    end
end
warning('Could not find a collision-free configuration nearby.');
% Still update struct config for GIK/IK
robotB.DataFormat = 'struct';
for j = 1:numel(initialGuess)
    initialGuess(j).JointPosition = qStart(j);
end
end 