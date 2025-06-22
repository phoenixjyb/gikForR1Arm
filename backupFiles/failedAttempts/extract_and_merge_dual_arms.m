
function extract_and_merge_dual_arms(urdfPath)

robot = importrobot(urdfPath);

% Extract left arm
leftLinks = {'left_arm_base_link', 'left_arm_link1', 'left_arm_link2', ...
             'left_arm_link3', 'left_arm_link4', 'left_arm_link5', ...
             'left_arm_link6', 'left_gripper_link'};
leftJoints = {'left_arm_base_joint', 'left_arm_joint1', 'left_arm_joint2', ...
              'left_arm_joint3', 'left_arm_joint4', 'left_arm_joint5', ...
              'left_arm_joint6', 'left_gripper_joint'};

leftRoot = 'left_arm_root';
leftArm = rigidBodyTree('DataFormat','row','MaxNumBodies', numel(leftLinks)+1);
addBody(leftArm, rigidBody(leftRoot), leftArm.BaseName);

% Add each joint/link to left arm
for i = 1:numel(leftJoints)
    jointName = leftJoints{i};
    body = copy(robot.Bodies{robot.getBodyIndex(robot.getBody(jointName).Name)});
    addBody(leftArm, body, body.Parent.Name);
end

% Transform offset for left arm base
tform_left = getTransform(robot, homeConfiguration(robot), 'left_arm_base_link', 'base_link');
leftArm.Base.Tform = tform_left;

% Extract right arm
rightLinks = {'right_arm_base_link', 'right_arm_link1', 'right_arm_link2', ...
              'right_arm_link3', 'right_arm_link4', 'right_arm_link5', ...
              'right_arm_link6', 'right_gripper_link'};  % fingers removed later
rightJoints = {'right_arm_base_joint', 'right_arm_joint1', 'right_arm_joint2', ...
               'right_arm_joint3', 'right_arm_joint4', 'right_arm_joint5', ...
               'right_arm_joint6', 'right_gripper_joint'};

rightRoot = 'right_arm_root';
rightArm = rigidBodyTree('DataFormat','row','MaxNumBodies', numel(rightLinks)+1);
addBody(rightArm, rigidBody(rightRoot), rightArm.BaseName);

% Add each joint/link to right arm
for i = 1:numel(rightJoints)
    jointName = rightJoints{i};
    body = copy(robot.Bodies{robot.getBodyIndex(robot.getBody(jointName).Name)});
    if contains(body.Name, 'finger')
        continue;  % skip fingers
    end
    addBody(rightArm, body, body.Parent.Name);
end

% Transform offset for right arm base
tform_right = getTransform(robot, homeConfiguration(robot), 'right_arm_base_link', 'base_link');
rightArm.Base.Tform = tform_right;

% Export each arm
exportrobot(leftArm, 'left_arm_aligned.urdf');
exportrobot(rightArm, 'right_arm_aligned.urdf');

% Merge both into one tree (optional)
mergedArm = rigidBodyTree('DataFormat','row');
addBody(mergedArm, rigidBody('base'), mergedArm.BaseName);

% Add left arm under new base
for i = 1:length(leftArm.BodyNames)
    b = copy(leftArm.getBody(leftArm.BodyNames{i}));
    if ~strcmp(b.Name, leftArm.BaseName)
        parent = b.Parent.Name;
        addBody(mergedArm, b, parent);
    end
end

% Add right arm under new base
for i = 1:length(rightArm.BodyNames)
    b = copy(rightArm.getBody(rightArm.BodyNames{i}));
    if ~strcmp(b.Name, rightArm.BaseName)
        parent = b.Parent.Name;
        addBody(mergedArm, b, parent);
    end
end

exportrobot(mergedArm, 'dual_arm_merged.urdf');
end
