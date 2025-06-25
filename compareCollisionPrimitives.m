% compareCollisionPrimitives.m
% Visualize R1 robot with collisionBox vs. collisionCylinder on left arm links

close all; clc; clear;

robotBox = importrobot('fineUrdfs/r1_v2_1_0.urdf');
robotCyl = importrobot('fineUrdfs/r1_v2_1_0.urdf');
robotBox.DataFormat = 'struct';
robotCyl.DataFormat = 'struct';

% Update home position
qHome = homeConfiguration(robotBox);
qHome = updateHomePositionforR1_wholeBody(robotBox, qHome);

leftArmLinks = {
    'left_arm_link1', 'left_arm_link2', 'left_arm_link3', ...
    'left_arm_link4', 'left_arm_link5', 'left_arm_link6'
};

for i = 1:numel(leftArmLinks)
    linkName = leftArmLinks{i};
    stlFile = fullfile('R1Meshes', [linkName, '.STL']);
    if exist(stlFile, 'file')
        [~, vertices] = stlread(stlFile);
        % Estimate box parameters
        minV = min(vertices, [], 1);
        maxV = max(vertices, [], 1);
        boxSize = maxV - minV;
        boxCenter = (minV + maxV) / 2;
        cb = collisionBox(boxSize(1), boxSize(2), boxSize(3));
        cb.Pose = trvec2tform(boxCenter);
        % Attach to robotBox
        bodyIdx = find(strcmp({robotBox.Bodies.Name}, linkName));
        if ~isempty(bodyIdx)
            addCollision(robotBox.Bodies{bodyIdx}, cb, cb.Pose);
        end
        % Estimate cylinder parameters (assume Z is main axis)
        zmin = min(vertices(:,3));
        zmax = max(vertices(:,3));
        cylLength = zmax - zmin;
        xy = vertices(:,1:2);
        center = mean(xy,1);
        radii = sqrt(sum((xy - center).^2,2));
        cylRadius = max(radii);
        cyl = collisionCylinder(cylRadius, cylLength);
        cylCenter = [center, (zmin+zmax)/2];
        cyl.Pose = trvec2tform(cylCenter);
        % Attach to robotCyl
        bodyIdx2 = find(strcmp({robotCyl.Bodies.Name}, linkName));
        if ~isempty(bodyIdx2)
            addCollision(robotCyl.Bodies{bodyIdx2}, cyl, cyl.Pose);
        end
    else
        warning(['STL file not found for ', linkName]);
    end
end

figure('Name','Collision Primitives Comparison','Position',[100 100 1200 600]);

subplot(1,2,1);
show(robotBox, qHome, 'PreservePlot', false, 'Collisions', 'on');
title('Robot with collisionBox on Left Arm');
view(45,30); axis equal; grid on;

subplot(1,2,2);
show(robotCyl, qHome, 'PreservePlot', false, 'Collisions', 'on');
title('Robot with collisionCylinder on Left Arm');
view(45,30); axis equal; grid on; 