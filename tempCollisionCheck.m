% Minimal example: checkCollision with external objects in MATLAB, with manual collision geometry

% 1. Import robot (use your URDF or a built-in one if available)
robot = importrobot('fineUrdfs/r1_v2_1_0.urdf');
robot.DataFormat = 'row';

% Create a map to store actual collision objects by body name
collisionMap = containers.Map;

% 2. Add collision objects to the robot in MATLAB
% Add a box to the left gripper link
idx_gripper = find(strcmp({robot.BodyNames{:}}, 'left_gripper_link'));
if ~isempty(idx_gripper)
    oldBody = robot.Bodies{idx_gripper};
    newBody = rigidBody(oldBody.Name);
    newBody.Joint = oldBody.Joint;
    cb_gripper = collisionBox(0.1, 0.04, 0.04);
    addCollision(newBody, cb_gripper, trvec2tform([0, 0, 0]));
    replaceBody(robot, newBody.Name, newBody);
    collisionMap('left_gripper_link') = cb_gripper;
    disp('Added collision box to left_gripper_link.');
else
    warning('left_gripper_link not found!');
end
% Add a box to the robot base (do not use replaceBody for the base!)
cb_base = collisionBox(0.2, 0.2, 0.1);
addCollision(robot.Base, cb_base, trvec2tform([0, 0, 0.05]));
collisionMap(robot.BaseName) = cb_base;

% --- Add collision boxes to all left arm links using STL bounding boxes ---
leftArmLinks = {'left_arm_base_link', 'left_arm_link1', 'left_arm_link2', ...
                'left_arm_link3', 'left_arm_link4', 'left_arm_link5', ...
                'left_arm_link6', 'left_gripper_link', ...
                'left_gripper_finger_link1', 'left_gripper_finger_link2'};

for i = 1:numel(leftArmLinks)
    linkName = leftArmLinks{i};
    stlFile = fullfile('R1Meshes', [linkName, '.STL']);
    if exist(stlFile, 'file')
        try
            [~, vertices] = stlread(stlFile);
            if ~isnumeric(vertices) || size(vertices,2) ~= 3
                % Try triangulation object
                TR = stlread(stlFile);
                vertices = TR.Points;
            end
        catch
            error(['Failed to read vertices from STL file: ', stlFile]);
        end
        minV = min(vertices, [], 1);
        maxV = max(vertices, [], 1);
        boxSize = maxV - minV;
        boxCenter = (minV + maxV) / 2;
        disp('boxCenter:');
        disp(boxCenter);
        disp('size(boxCenter):');
        disp(size(boxCenter));
        cb = collisionBox(boxSize(1), boxSize(2), boxSize(3));
        cb.Pose = trvec2tform(reshape(boxCenter, 1, 3));
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

% 3. Set robot to home configuration
q = homeConfiguration(robot); % returns a row vector

% 4. Get left gripper position in home configuration
T_gripper = getTransform(robot, q, 'left_gripper_link');
pos_gripper = tform2trvec(T_gripper);
fprintf('Left gripper position: [%.2f, %.2f, %.2f]\n', pos_gripper(1), pos_gripper(2), pos_gripper(3));

% 5. Create a larger collision box at the gripper position
box_gripper = collisionBox(0.4, 0.4, 0.4); % 40cm cube
box_gripper.Pose = trvec2tform(pos_gripper); % Place box at gripper
fprintf('Box (gripper) pose: [%.2f, %.2f, %.2f]\n', box_gripper.Pose(1,4), box_gripper.Pose(2,4), box_gripper.Pose(3,4));

% Debug: Print collisions for gripper
if isKey(collisionMap, 'left_gripper_link')
    disp('Collision object for gripper (from map):');
    disp(collisionMap('left_gripper_link'));
else
    disp('No collision object stored for gripper.');
end

% Check if the small box on the gripper is fully contained in the large box at the gripper
if isKey(collisionMap, 'left_gripper_link')
    smallBox = collisionMap('left_gripper_link');
    % Compute the small box's pose in the world frame
    T_gripper = getTransform(robot, q, 'left_gripper_link');
    smallBoxWorld = smallBox;
    smallBoxWorld.Pose = T_gripper * smallBox.Pose;
    contained = isBoxContained(smallBoxWorld, box_gripper);
    if contained
        disp('Small box at gripper is fully contained within the large box at gripper.');
    else
        disp('Small box at gripper is NOT fully contained within the large box at gripper.');
    end
else
    warning('No collision object for gripper to check containment.');
end

% 6. Check for collision at gripper
inCollision_gripper = checkCollision(robot, q, {box_gripper});
if inCollision_gripper
    disp('Collision detected between robot and box at gripper!');
else
    disp('No collision between robot and box at gripper.');
end

% 7. Get robot base position
T_base = getTransform(robot, q, robot.BaseName);
pos_base = tform2trvec(T_base);
fprintf('Robot base position: [%.2f, %.2f, %.2f]\n', pos_base(1), pos_base(2), pos_base(3));

% 8. Create a larger collision box at the base position
box_base = collisionBox(0.4, 0.4, 0.4); % 40cm cube
box_base.Pose = trvec2tform(pos_base); % Place box at base
fprintf('Box (base) pose: [%.2f, %.2f, %.2f]\n', box_base.Pose(1,4), box_base.Pose(2,4), box_base.Pose(3,4));

% 9. Check for collision at base
inCollision_base = checkCollision(robot, q, {box_base});
if inCollision_base
    disp('Collision detected between robot and box at base!');
else
    disp('No collision between robot and box at base.');
end

% 10. Visualize collision geometry and both boxes
figure; hold on; axis equal; grid on;
show(robot, q, 'Collisions', 'on', 'PreservePlot', false); % Only show collision geometry
show(box_gripper);
show(box_base);
title('Robot Collision Geometry with Boxes at Gripper and Base (Manual Collision Added)');
legend('Robot Collision Geometry', 'Box at Gripper', 'Box at Base');

% --- Load assigned obstacles (cardbox and table) from .mat files ---
load('table.mat');   % loads variable: tableBox
load('cardbox.mat'); % loads variable: cardbox

% --- Check collision between the full robot arm and assigned boxes ---
inCollision_assigned = checkCollision(robot, q, {cardbox, tableBox});
if inCollision_assigned
    disp('Collision detected between robot and cardbox or table!');
else
    disp('No collision between robot and cardbox/table.');
end

% --- Visualize the robot and assigned obstacles ---
figure; hold on; axis equal; grid on;
show(robot, q, 'Collisions', 'on', 'PreservePlot', false);
show(cardbox);
show(tableBox);
title('Robot and Assigned Obstacles (Cardbox, Table)');
legend('Robot', 'Cardbox', 'Table');

function isContained = isBoxContained(innerBox, outerBox)
    % Get the 8 corners of the inner box in world coordinates
    [ix, iy, iz] = ndgrid([-0.5 0.5]*innerBox.X, [-0.5 0.5]*innerBox.Y, [-0.5 0.5]*innerBox.Z);
    innerCorners = [ix(:), iy(:), iz(:)]';
    innerCorners = [innerCorners; ones(1,8)];
    innerCornersWorld = innerBox.Pose * innerCorners;
    innerCornersWorld = innerCornersWorld(1:3, :);

    % Transform the corners into the outer box's local frame
    outerInv = inv(outerBox.Pose);
    innerInOuter = outerInv * [innerCornersWorld; ones(1,8)];
    innerInOuter = innerInOuter(1:3, :);

    % Check if all corners are within the outer box's half-dimensions
    isInside = all(abs(innerInOuter(1,:)) <= outerBox.X/2 + 1e-8 & ...
                   abs(innerInOuter(2,:)) <= outerBox.Y/2 + 1e-8 & ...
                   abs(innerInOuter(3,:)) <= outerBox.Z/2 + 1e-8);
    isContained = isInside;
end

ss = manipulatorStateSpace(robot);
sv = manipulatorCollisionBodyValidator(ss);
sv.ValidationDistance = 0.01;