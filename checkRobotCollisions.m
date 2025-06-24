function collidingPairs = checkRobotCollisions(robotB, qStart, varargin)
% checkRobotCollisions checks for self-colliding body pairs in robotB at qStart
% and for collisions with external obstacles if provided.
% Usage:
%   collidingPairs = checkRobotCollisions(robotB, qStart)
%   collidingPairs = checkRobotCollisions(robotB, qStart, tableObstacle, cardboxObstacle)

    % Evaluate inSelfCollision using checkCollision (single output)
    inSelfCollision = checkCollision(robotB, qStart, 'IgnoreSelfCollision', 'off', 'Exhaustive', 'on', 'SkippedSelfCollisions', cell(0,2));
    fprintf('inSelfCollision (single output): %d\n', inSelfCollision);

    [isColliding, sepDist, witnessPts] = checkCollision(robotB, qStart, ...
        'IgnoreSelfCollision', 'off', 'Exhaustive', 'on', 'SkippedSelfCollisions', cell(0,2));
    bodyNames = cellfun(@(b) b.Name, robotB.Bodies, 'UniformOutput', false);

    % Print all pairs and their sepDist values
    fprintf('Pairwise separation distances (sepDist):\n');
    for i = 1:length(bodyNames)
        for j = i+1:length(bodyNames)
            fprintf('%s - %s: %f\n', bodyNames{i}, bodyNames{j}, sepDist(i,j));
        end
    end

    collidingPairs = {};
    if isColliding
        % sepDist is a matrix: (N x N), negative values mean collision
        for i = 1:length(bodyNames)
            for j = i+1:length(bodyNames)
                if sepDist(i,j) < 0
                    collidingPairs{end+1,1} = bodyNames{i};
                    collidingPairs{end,2} = bodyNames{j};
                end
            end
        end
    end

    if isempty(collidingPairs)
        disp('No self-colliding pairs found.');
    else
        disp('Colliding body pairs:');
        disp(collidingPairs);
    end

    % Check for collisions with external obstacles if provided
    if nargin > 2
        for k = 1:length(varargin)
            obstacle = varargin{k};
            if ~isempty(obstacle)
                [inCollisionWithObstacle, sepDistExt, ~] = checkCollision(robotB, qStart, {obstacle});
                % sepDistExt: [nLinks x nObstacles] (here, nObstacles=1)
                if isvector(sepDistExt) && length(sepDistExt) == length(bodyNames)
                    [minDist, minIdx] = min(sepDistExt);
                    if minDist <= 0
                        fprintf('Collision with obstacle %d! Closest link: %s, sepDist = %.6f\n', k, bodyNames{minIdx}, minDist);
                    else
                        fprintf('No collision with obstacle %d. Closest link: %s, sepDist = %.6f\n', k, bodyNames{minIdx}, minDist);
                    end
                elseif ismatrix(sepDistExt) && size(sepDistExt,2) > 1
                    % Multiple obstacles at once (not typical for this function, but handle gracefully)
                    nObs = size(sepDistExt,2);
                    for obsIdx = 1:nObs
                        [minDist, minIdx] = min(sepDistExt(:,obsIdx));
                        if minDist <= 0
                            fprintf('Collision with obstacle %d (column %d)! Closest link: %s, sepDist = %.6f\n', k, obsIdx, bodyNames{minIdx}, minDist);
                        else
                            fprintf('No collision with obstacle %d (column %d). Closest link: %s, sepDist = %.6f\n', k, obsIdx, bodyNames{minIdx}, minDist);
                        end
                    end
                else
                    fprintf('sepDistExt shape is unexpected. Please check collision object setup.\n');
                end
            end
        end
    end
end