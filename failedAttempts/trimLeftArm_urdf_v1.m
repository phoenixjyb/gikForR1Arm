% filepath: /Users/yanbo/Library/CloudStorage/OneDrive-Personal/ConvergeAI/RobotMotion/gikForR1Arm/trimBodyUrdf_r1.m

% Load full robot
robotFull = importrobot('r1_v2_1_0.urdf');
robotFull.DataFormat = 'row';

% Define links
left_arm_tip = 'left_gripper_link';
left_arm_base = 'left_arm_base_link';
right_arm_tip = 'right_gripper_link';
right_arm_base = 'right_arm_base_link';

% Get transform chain
homeConfig = robotFull.homeConfiguration;
T_base_to_arm = getTransform(robotFull, homeConfig, 'base_link', 'left_arm_base_link');
fprintf('Transform from base_link to left_arm_base_link:\n');
disp(T_base_to_arm);

% Initialize chain structure
left_arm_chain = struct('name', {}, 'parent', {}, 'children', {}, ...
    'mass', {}, 'inertia', {}, 'visual', {}, 'collision', {}, ...
    'joint_name', {}, 'joint_type', {}, 'joint_origin_xyz', {}, ...
    'joint_origin_rpy', {}, 'joint_axis', {}, 'joint_limits', {}, ...
    'joint_transform', {}, 'body', {});

% Get first link
current_bodyL = robotFull.getBody(left_arm_tip);
idx = 1;

% Store first link properties
left_arm_chain(idx).name = left_arm_tip;
left_arm_chain(idx).parent = current_bodyL.Parent.Name;
left_arm_chain(idx).children = {};
left_arm_chain(idx).mass = current_bodyL.Mass;
left_arm_chain(idx).inertia = current_bodyL.Inertia;
left_arm_chain(idx).visual = current_bodyL.Visuals;
left_arm_chain(idx).collision = current_bodyL.Collisions;
left_arm_chain(idx).joint_name = current_bodyL.Joint.Name;
left_arm_chain(idx).joint_type = current_bodyL.Joint.Type;
left_arm_chain(idx).joint_axis = current_bodyL.Joint.JointAxis;
left_arm_chain(idx).joint_limits = current_bodyL.Joint.PositionLimits;
left_arm_chain(idx).joint_transform = current_bodyL.Joint.JointToParentTransform;
left_arm_chain(idx).body = current_bodyL;

% Build chain from tip to base
while true
    parent_name = current_bodyL.Parent.Name;
    if strcmp(parent_name, robotFull.Base.Name) || strcmp(parent_name, left_arm_base)
        break;
    end
    
    % Add parent as new link
    idx = idx + 1;
    current_bodyL = robotFull.getBody(parent_name);
    
    % Store complete properties
    left_arm_chain(idx).name = parent_name;
    left_arm_chain(idx).parent = current_bodyL.Parent.Name;
    left_arm_chain(idx).children = {left_arm_chain(idx-1).name};
    left_arm_chain(idx).mass = current_bodyL.Mass;
    left_arm_chain(idx).inertia = current_bodyL.Inertia;
    left_arm_chain(idx).visual = current_bodyL.Visuals;
    left_arm_chain(idx).collision = current_bodyL.Collisions;
    left_arm_chain(idx).joint_name = current_bodyL.Joint.Name;
    left_arm_chain(idx).joint_type = current_bodyL.Joint.Type;
    left_arm_chain(idx).joint_axis = current_bodyL.Joint.JointAxis;
    left_arm_chain(idx).joint_limits = current_bodyL.Joint.PositionLimits;
    left_arm_chain(idx).joint_transform = current_bodyL.Joint.JointToParentTransform;
    left_arm_chain(idx).body = current_bodyL;
    
    % Update previous link's parent
    left_arm_chain(idx-1).parent = parent_name;
end

% Reverse chain for base-to-tip order
left_arm_chain = flip(left_arm_chain);

% Create new robot
robotLeft = rigidBodyTree('DataFormat', 'row');

% Create and add world_to_arm_offset with transform
offsetBody = rigidBody('world_to_arm_offset');
offsetJoint = rigidBodyJoint('fix_offset', 'fixed');
setFixedTransform(offsetJoint, T_base_to_arm);
offsetBody.Joint = offsetJoint;
addBody(robotLeft, offsetBody, robotLeft.BaseName);

% Add bodies in order
for i = 1:length(left_arm_chain)
    bname = left_arm_chain(i).name;
    new_body = rigidBody(bname);
    new_joint = rigidBodyJoint(left_arm_chain(i).joint_name, left_arm_chain(i).joint_type);
    
    % Set joint properties
    if ~strcmp(left_arm_chain(i).joint_type, 'fixed')
        new_joint.JointAxis = left_arm_chain(i).joint_axis;
        new_joint.PositionLimits = left_arm_chain(i).joint_limits;
    end
    setFixedTransform(new_joint, left_arm_chain(i).joint_transform);
    
    % Set body properties
    new_body.Mass = left_arm_chain(i).mass;
    new_body.Inertia = left_arm_chain(i).inertia;
    new_body.Joint = new_joint;
    
    % Add body to tree
    if i == 1
        parent_name = offsetBody.Name;
    else
        parent_name = left_arm_chain(i-1).name;
    end
    
    fprintf('Adding body %s with parent %s\n', bname, parent_name);
    addBody(robotLeft, new_body, parent_name);
end

% Export URDF
exportURDF(robotLeft, 'left_arm.urdf');