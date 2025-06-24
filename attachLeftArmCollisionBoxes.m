function robot = attachLeftArmCollisionBoxes(robot, meshDir)
% Attach collision boxes to left arm and gripper links based on STL bounding boxes

leftArmLinks = {'left_arm_base_link', 'left_arm_link1', 'left_arm_link2', ...
                'left_arm_link3', 'left_arm_link4', 'left_arm_link5', ...
                'left_arm_link6', 'left_gripper_link', ...
                'left_gripper_finger_link1', 'left_gripper_finger_link2'};

for i = 1:numel(leftArmLinks)
    linkName = leftArmLinks{i};
    stlFile = fullfile(meshDir, [linkName, '.STL']);
    if exist(stlFile, 'file')
        % Read STL vertices robustly
        [~, vertices] = stlread(stlFile);
        if ~isnumeric(vertices) || size(vertices,2) ~= 3
            % Try triangulation object
            TR = stlread(stlFile);
            vertices = TR.Points;
        end
        minV = min(vertices, [], 1);
        maxV = max(vertices, [], 1);
        boxSize = maxV - minV;
        boxCenter = double((minV + maxV) / 2);  % Ensure double precision
        boxCenter = boxCenter(:)';  % Ensure 1x3 row vector
        if numel(boxCenter) ~= 3
            error('Expected boxCenter to be a 3D point, got %d elements', numel(boxCenter));
        end
        cb = collisionBox(boxSize(1), boxSize(2), boxSize(3));
        tform = trvec2tform(boxCenter);  % Create transform first
        cb.Pose = tform;  % Then assign it
        % Replace body with new collision
        bodyNames = cellfun(@(b) b.Name, robot.Bodies, 'UniformOutput', false);
        idx = find(strcmp(bodyNames, linkName));
        if ~isempty(idx)
            oldBody = robot.Bodies{idx};
            newBody = rigidBody(oldBody.Name);
            newBody.Joint = oldBody.Joint;
            addCollision(newBody, cb, eye(4)); % Pose is already set in cb
            replaceBody(robot, newBody.Name, newBody);
            disp(['Added collision box to ', linkName]);
        else
            warning([linkName, ' not found in robot model!']);
        end
    else
        warning(['STL file not found for ', linkName, ': ', stlFile]);
    end
end
end 