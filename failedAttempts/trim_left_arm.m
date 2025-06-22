% filepath: /Users/yanbo/Library/CloudStorage/OneDrive-Personal/ConvergeAI/RobotMotion/gikForR1Arm/trim_left_arm.m
function trim_left_arm(input_urdf, output_urdf)
    % Read the URDF file
    urdf_content = fileread(input_urdf);
    
    % Extract the left arm links and joints
    left_arm_links = {'left_arm_base_link', 'left_arm_link1', 'left_arm_link2', ...
                      'left_arm_link3', 'left_arm_link4', 'left_arm_link5', ...
                      'left_arm_link6', 'left_gripper_link', ...
                      'left_gripper_finger_link1', 'left_gripper_finger_link2'};
    left_arm_joints = {'left_arm_base_joint', 'left_arm_joint1', 'left_arm_joint2', ...
                       'left_arm_joint3', 'left_arm_joint4', 'left_arm_joint5', ...
                       'left_arm_joint6', 'left_gripper_joint', ...
                       'left_gripper_finger_joint1', 'left_gripper_finger_joint2'};
    
    % Initialize new URDF content
    new_urdf = ['<robot name="left_arm_robot">' newline];
    
    % Extract world frame (base_link)
    base_link_pattern = '(?s)<link\s+name="base_link">.*?</link>';
    base_link_match = regexp(urdf_content, base_link_pattern, 'match', 'once');
    if ~isempty(base_link_match)
        new_urdf = [new_urdf base_link_match newline];
    end
    
    % Extract left arm links
    for i = 1:length(left_arm_links)
        link_pattern = ['(?s)<link\s+name="' left_arm_links{i} '">.*?</link>'];
        link_match = regexp(urdf_content, link_pattern, 'match', 'once');
        if ~isempty(link_match)
            new_urdf = [new_urdf link_match newline];
        end
    end
    
    % Extract left arm joints
    for i = 1:length(left_arm_joints)
        joint_pattern = ['(?s)<joint\s+name\s*=\s*"' left_arm_joints{i} '".*?</joint>'];
        joint_match = regexp(urdf_content, joint_pattern, 'match', 'once');
        if ~isempty(joint_match)
            if strcmp(left_arm_joints{i}, 'left_arm_base_joint')
                % Change parent to base_link and update origin values
                joint_match = regexprep(joint_match, '<parent\s+link=".*?"/>', '<parent link="base_link"/>');
                joint_match = regexprep(joint_match, '<origin\s+xyz="[^"]+"', '<origin xyz="0 0 0"');
                joint_match = regexprep(joint_match, '<origin([^>]+)rpy="[^"]+"', '<origin$1rpy="0 0 0"');
            end
            new_urdf = [new_urdf joint_match newline];
        end
    end
    
    % Close the robot tag
    new_urdf = [new_urdf '</robot>' newline];
    
    % Write the new URDF to the output file
    fid = fopen(output_urdf, 'w');
    fprintf(fid, '%s', new_urdf);
    fclose(fid);
    
    disp(['Trimmed URDF saved to: ' output_urdf]);
end