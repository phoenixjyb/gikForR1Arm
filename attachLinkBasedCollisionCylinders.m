function robot = attachLinkBasedCollisionCylinders(robot, qConfig)
% Attach collision cylinders only along main kinematic chains using link (body) names:
% - Left arm: left_arm_link1 to left_gripper_finger_link2
% - Right arm: right_arm_link1 to right_gripper_finger_link2
% - Torso: torso_link1 to torso_link4
% Uses link order, not parent/child tree.
% qConfig: configuration struct (e.g., updated home position)

if nargin < 2 || isempty(qConfig)
    qConfig = homeConfiguration(robot);
end

% Define link chains (update these names to match your robot's actual BodyNames)
leftArmLinks = {'left_arm_link1', 'left_arm_link2', 'left_arm_link3', 'left_arm_link4', 'left_arm_link5', 'left_arm_link6', 'left_gripper_link', 'left_gripper_finger_link1', 'left_gripper_finger_link2'};
rightArmLinks = {'right_arm_link1', 'right_arm_link2', 'right_arm_link3', 'right_arm_link4', 'right_arm_link5', 'right_arm_link6', 'right_gripper_link', 'right_gripper_finger_link1', 'right_gripper_finger_link2'};
torsoLinks = {'torso_link1', 'torso_link2', 'torso_link3', 'torso_link4'};

% Helper to get link positions in base frame
getLinkPos = @(linkName) tform2trvec(getTransform(robot, qConfig, linkName));

% Attach cylinders for a chain
function attachChainCylinders(linkList, radius)
    for i = 1:(numel(linkList)-1)
        pos1 = getLinkPos(linkList{i});
        pos2 = getLinkPos(linkList{i+1});
        link_vec = pos2 - pos1;
        link_length = norm(link_vec);
        if link_length < 1e-4
            continue;
        end
        cyl = collisionCylinder(radius, link_length);
        center = (pos1 + pos2) / 2;
        z_axis = link_vec / link_length;
        default_z = [0 0 1];
        if all(abs(z_axis - default_z) < 1e-8)
            R = eye(3);
        elseif all(abs(z_axis + default_z) < 1e-8)
            R = axang2rotm([1 0 0 pi]);
        else
            axis = cross(default_z, z_axis);
            angle = acos(dot(default_z, z_axis));
            R = axang2rotm([axis/norm(axis), angle]);
        end
        pose = [R, center(:); 0 0 0 1];
        cyl.Pose = pose;
        % Attach to the child link (link i+1)
        for b = 1:numel(robot.Bodies)
            if strcmp(robot.Bodies{b}.Name, linkList{i+1})
                addCollision(robot.Bodies{b}, cyl, eye(4));
                break;
            end
        end
    end
end

% Left arm: arm segments 0.05m, fingers 0.01m
attachChainCylinders(leftArmLinks(1:7), 0.05);
attachChainCylinders(leftArmLinks(7:end), 0.01);
% Right arm: arm segments 0.05m, fingers 0.01m
attachChainCylinders(rightArmLinks(1:7), 0.05);
attachChainCylinders(rightArmLinks(7:end), 0.01);
% Torso: 0.1m
attachChainCylinders(torsoLinks, 0.1);

for i = 1:numel(leftArmLinks)
    disp(leftArmLinks{i});
    disp(getLinkPos(leftArmLinks{i}));
end

end 