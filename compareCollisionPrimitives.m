% compareCollisionPrimitives.m
% Visualize R1 robot with collisionBox vs. collisionCylinder on left arm links (default colors)

close all; clc; clear;

robotBox = importrobot('fineUrdfs/r1_v2_1_0.urdf');
robotCyl = importrobot('fineUrdfs/r1_v2_1_0.urdf');
robotBox.DataFormat = 'struct';
robotCyl.DataFormat = 'struct';

robotBox = attachLeftArmCollisionPrimitives(robotBox, 'R1Meshes', 'box');
robotCyl = attachLeftArmCollisionPrimitives(robotCyl, 'R1Meshes', 'cylinder');

qHome = homeConfiguration(robotBox);
qHome = updateHomePositionforR1_wholeBody(robotBox, qHome);

figure('Name','Collision Primitives Comparison','Position',[100 100 1200 600]);

subplot(1,2,1);
show(robotBox, qHome, 'PreservePlot', false, 'Collisions', 'on');
title('Robot with collisionBox on Left Arm');
view(45,30); axis equal; grid on;

subplot(1,2,2);
show(robotCyl, qHome, 'PreservePlot', false, 'Collisions', 'on');
title('Robot with collisionCylinder on Left Arm');
view(45,30); axis equal; grid on;