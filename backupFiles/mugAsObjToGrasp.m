% --- Given ---

function [transformed_mug_in_B,x,y,z, mugInA] = mugAsObjToGrasp(eePosAs)

k = 1;
[~, mugPosition] = kmeans(eePosAs, k);
mugPosition(3) = mugPosition(3)+0.1; % x y z
mugPosition(1) = mugPosition(1)-0.1; % x y z
mugCenter = mugPosition;     % center of mug base
mugRadius = 0.03; % 
mugHeight = 0.21; % 

% --- Define multiple grasp poses ---
numGrasps = 4;
graspTforms = cell(numGrasps, 1);

for i = 1:numGrasps
    angle = 2 * pi * (i-1) / numGrasps;

    % Position: around the mug, at mid-height
    graspPos = mugCenter + [mugRadius * cos(angle), mugRadius * sin(angle), mugHeight/2];

    % Orientation: z-axis toward mug center, x-axis up (side grasp)
    z_axis = normalize(mugCenter - graspPos);
    x_axis = [0 0 1];  % vertical (up)
    y_axis = cross(z_axis, x_axis);
    x_axis = cross(y_axis, z_axis);  % re-orthogonalize

    R = [x_axis(:), y_axis(:), z_axis(:)];
    R = orth(R);  % ensure valid rotation matrix

    % Combine into homogeneous transform
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = graspPos(:);
    graspTforms{i} = T;

    % Visualization: draw axes
    plotTransforms(graspPos, rotm2quat(R), 'FrameSize', 0.05);
end


% --- Visualize Mug and Reachable Points ---
figure

scatter3(eePosAs(:,1), eePosAs(:,2), eePosAs(:,3), 2, 'b', 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Grasp Poses Around Mug');
axis equal; grid on;
hold on
%%

% Create cylinder
[theta, z] = meshgrid(linspace(0, 2*pi, 30), linspace(0, mugHeight, 2));
x = mugRadius * cos(theta);
y = mugRadius * sin(theta);
z = z;
% Translate to mug position
x = x + mugPosition(1);
y = y + mugPosition(2);
z = z + mugPosition(3);

% Plot
surf(x, y, z, 'FaceColor', [0.8 0.3 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
% Axis setup
view(3);
axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Mug Visualization & Reachable Points');
legend('Reachable Points', 'Grasp Poses with a Mug');

%% reshape mug meshgrid to facilitate coordinate transformation in B.
mugPts = numel(x);
pts = [x(:),y(:),z(:)]';
transformed_mug_in_B = [pts; ones(1,mugPts)]; % only reshape it

%% output mugs in both A and B's frames
% 
mug = struct();
mugPose = trvec2tform(mugCenter);  % 4×4 pose matrix
mug.radius = mugRadius;
mug.height = mugHeight;
mug.pose   = mugPose;

mug.collision = collisionCylinder(mug.radius, mug.height);
mug.collision.Pose = mug.pose * trvec2tform([0, 0, mug.height/2]);  % center at middle height

% define a top-down grasp pose
graspOffset = trvec2tform([0, 0, mug.height + 0.02]) * axang2tform([1 0 0 pi]);  % 2 cm above mug
mug.graspPose = mug.pose * graspOffset;
mugInA = mug;
% now we need mug strcut in B as well


%{
figure; hold on; view(3); axis equal; grid on;
show(mug.collision);
trplot(mug.graspPose, 'frame', 'grasp', 'length', 0.05);  % Requires Peter Corke's toolbox
%}