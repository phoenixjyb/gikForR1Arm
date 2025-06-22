% filepath: /Users/yanbo/Library/CloudStorage/OneDrive-Personal/ConvergeAI/RobotMotion/gikForR1Arm/trimBodyUrdf_r1.m

% Load and setup robot
robotFull = importrobot('r1_v2_1_0.urdf');
robotFull.DataFormat = 'row';

% Define arm links
left_arm_tip = 'left_gripper_link';
left_arm_base = 'left_arm_base_link';
right_arm_tip = 'right_gripper_link';
right_arm_base = 'right_arm_base_link';

% Build left arm chain with parent-child info
left_arm_chain = struct('name', {}, 'parent', {}, 'children', {}, 'body', {});
current_bodyL = robotFull.getBody(left_arm_tip);

% Add first link
idx = 1;
left_arm_chain(idx).name = left_arm_tip;
left_arm_chain(idx).parent = current_bodyL.Parent.Name;
left_arm_chain(idx).children = {};
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
    left_arm_chain(idx).name = parent_name;
    left_arm_chain(idx).parent = current_bodyL.Parent.Name;
    left_arm_chain(idx).children = {left_arm_chain(idx-1).name};
    left_arm_chain(idx).body = current_bodyL;
    
    % Update previous link's parent
    left_arm_chain(idx-1).parent = parent_name;
end

% Reverse chain for base-to-tip order
left_arm_chain = flip(left_arm_chain);

% Function to copy complete joint properties with validation
function new_body = copyBodyWithJointProps(original_body)
    new_body = copy(original_body);
    if ~isempty(original_body.Joint)
        % Handle non-fixed joints
        if ~strcmp(original_body.Joint.Type, 'fixed')
            % Ensure valid joint axis
            if ~isempty(original_body.Joint.JointAxis) && ~any(isnan(original_body.Joint.JointAxis))
                new_body.Joint.JointAxis = original_body.Joint.JointAxis;
            else
                new_body.Joint.JointAxis = [0 0 1];  % Default Z-axis
            end
            
            % IMPORTANT: Always set position limits for URDF compliance
            % Check if existing limits are valid
            has_valid_limits = ~isempty(original_body.Joint.PositionLimits) && ...
                             ~any(isnan(original_body.Joint.PositionLimits)) && ...
                             numel(original_body.Joint.PositionLimits) == 2 && ...
                             original_body.Joint.PositionLimits(1) < original_body.Joint.PositionLimits(2);
            
            if has_valid_limits
                new_body.Joint.PositionLimits = original_body.Joint.PositionLimits;
            else
                % Set appropriate default limits based on joint type
                switch original_body.Joint.Type
                    case 'revolute'
                        new_body.Joint.PositionLimits = [-pi, pi];
                    case 'prismatic'
                        new_body.Joint.PositionLimits = [-1, 1];  % 1 meter in each direction
                    otherwise
                        new_body.Joint.PositionLimits = [-pi, pi];  % Default fallback
                end
            end
            
            % Copy home position if valid
            if ~isempty(original_body.Joint.HomePosition)
                new_body.Joint.HomePosition = original_body.Joint.HomePosition;
            end
        end
        
        % Set transform using proper method
        if ~isempty(original_body.Joint.JointToParentTransform)
            setFixedTransform(new_body.Joint, original_body.Joint.JointToParentTransform);
        end
    end
end

% Create left arm robot
robotLeft = rigidBodyTree('DataFormat', 'row');

% Get transform from base to left_arm_base_link
T_world_to_base = getTransform(robotFull, robotFull.homeConfiguration, 'left_arm_base_link', 'base_link');

% Create and add world_to_arm_offset
offsetBody = rigidBody('world_to_arm_offset');
offsetJoint = rigidBodyJoint('fix_offset', 'fixed');
setFixedTransform(offsetJoint, T_world_to_base);
offsetBody.Joint = offsetJoint;
addBody(robotLeft, offsetBody, robotLeft.BaseName);

% Add left arm bodies in correct order
for i = 1:length(left_arm_chain)
    bname = left_arm_chain(i).name;
    original_body = left_arm_chain(i).body;
    new_body = copyBodyWithJointProps(original_body);
    
    % For the first link, attach to the offset body
    if i == 1
        parent_name = offsetBody.Name;
    else
        % For subsequent links, attach to the previous link
        parent_name = left_arm_chain(i-1).name;
    end
    
    fprintf('Adding body %s with parent %s\n', bname, parent_name);
    addBody(robotLeft, new_body, parent_name);
end

% Export URDF
exportURDF(robotLeft, 'left_arm.urdf');

% Build right arm chain
right_arm_chain = {right_arm_tip};
current_bodyR = robotFull.getBody(right_arm_tip);
while true
    parent_name = current_bodyR.Parent.Name;
    if strcmp(parent_name, robotFull.Base.Name) || strcmp(parent_name, right_arm_base)
        break;
    end
    right_arm_chain{end+1} = parent_name;
    current_bodyR = robotFull.getBody(parent_name);
end

% Reverse the chains to get base-to-tip order
right_arm_chain = flip(right_arm_chain);

% Create right arm robot
robotRight = rigidBodyTree('DataFormat', 'row');

% Get transform from base to right_arm_base_link
T_world_to_base_R = getTransform(robotFull, robotFull.homeConfiguration, 'right_arm_base_link', 'base_link');

% Create and add world_to_arm_offset for right arm
R_offsetBody = rigidBody('world_to_arm_offset');
R_offsetJoint = rigidBodyJoint('fix_offset', 'fixed');
setFixedTransform(R_offsetJoint, T_world_to_base_R);
R_offsetBody.Joint = R_offsetJoint;
addBody(robotRight, R_offsetBody, robotRight.BaseName);

% Add right arm bodies in order from base to tip
for i = 1:length(right_arm_chain)
    bname = right_arm_chain{i};
    original_body = robotFull.getBody(bname);
    
    % For the first link, attach to the offset body
    if i == 1
        parent_name = R_offsetBody.Name;
    else
        % For subsequent links, attach to the previous link
        parent_name = right_arm_chain{i-1};
    end
    
    fprintf('Adding right arm body %s with parent %s\n', bname, parent_name);
    addBody(robotRight, copy(original_body), parent_name);
end

% Save modified URDFs
exportURDF(robotRight, 'right_arm.urdf');