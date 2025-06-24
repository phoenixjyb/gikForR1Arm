% Minimal example: checkCollision with external objects in MATLAB, with manual collision geometry

% 1. Import robot (use your URDF or a built-in one if available)
robot = importrobot('fineUrdfs/r1_v2_1_0.urdf');
robot.DataFormat = 'row';

% 2. Add collision objects to the robot in MATLAB
% Add a box to the left gripper link
idx_gripper = find(strcmp({robot.BodyNames{:}}, 'left_gripper_link'));
if ~isempty(idx_gripper)
    addCollision(robot.Bodies{idx_gripper}, collisionBox(0.1, 0.04, 0.04), trvec2tform([0, 0, 0]));
    disp('Added collision box to left_gripper_link.');
else
    warning('left_gripper_link not found!');
end
% Add a box to the robot base
addCollision(robot.Base, collisionBox(0.2, 0.2, 0.1), trvec2tform([0, 0, 0.05]));

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