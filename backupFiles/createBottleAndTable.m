% --- Given ---

function [x,y,z,bottleInB,tableBox] = createBottleAndTable(eePosBs,robotB)

% eePosB is not used to create a bottle, but useful to check if it is
% reachable

% bottle is placed on table edge, with a cardbox in-between; table height how about .

bottlePosition(1) = 0.4; % x  
bottlePosition(2) = 0; % y
bottlePosition(3) = 1.15; % z
bottleCenter = bottlePosition;     % center of bottle base
bottleRadius = 0.03; % in meter
bottleHeight = 0.21; % in meter

% --- Define multiple grasp poses ---
numGrasps = 4;
graspTforms = cell(numGrasps, 1);

for i = 1:numGrasps
    angle = 2 * pi * (i-1) / numGrasps;

    % Position: around the bottle, at mid-height
    graspPos = bottleCenter + [bottleRadius * cos(angle), bottleRadius * sin(angle), bottleHeight/2];

    % Orientation: z-axis toward bottle center, x-axis up (side grasp)
    z_axis = normalize(bottleCenter - graspPos);
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

% --- Visualize bottle and Reachable Points ---
figure

scatter3(eePosBs(:,1), eePosBs(:,2), eePosBs(:,3), 2, 'b', 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Grasp Poses Around bottle');
grid on;
hold on
show(robotB);

%%

% Create cylinder
[theta, z] = meshgrid(linspace(0, 2*pi, 30), linspace(0, bottleHeight, 2));
x = bottleRadius * cos(theta);
y = bottleRadius * sin(theta);
z = z;
% Translate to bottle position
x = x + bottlePosition(1);
y = y + bottlePosition(2);
z = z + bottlePosition(3);

% Plot bottle with mesh
surf(x, y, z, 'FaceColor', [0.8 0.3 0.3], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
% Axis setup
view(3);
axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('bottle Visualization & Reachable Points');
legend('Reachable Points', 'Grasp Poses with a bottle');

%% output bottles in B's frames
% 
bottle = struct();
bottlePose = trvec2tform(bottleCenter);  % 4×4 pose matrix
bottle.radius = bottleRadius;
bottle.height = bottleHeight;
bottle.pose   = bottlePose;

bottle.collision = collisionCylinder(bottle.radius, bottle.height);
bottle.collision.Pose = bottle.pose * trvec2tform([0, 0, bottle.height/2]);  % center at middle height

% Define a side grasp pose: approach from +X, fingers close in Y (XY plane), at mid-height
graspOffset = trvec2tform([bottle.radius + 0.02, 0, bottle.height/2]) * axang2tform([0 1 0 pi/2]);
bottle.graspPose = bottle.pose * graspOffset;

bottleInB = bottle;

%% now define a table and check 

%— Bottle (fixed in world) —%
% bottle position is fixed above

%— Table Parameters —%
tableSide         = 0.6;    % 60 cm square
edgeOffset        = 0.10;   % 10 cm in from the edge; 
tableTopThickness = 0.02;   % 2 cm thick
tableTopZ         = bottlePosition(3);
tableBottomZ      = tableTopZ - tableTopThickness;

%— Compute Table Center so bottle is from –X edge —%
% For –X edge: bottleX = tableCenterX + (–tableSide/2 + edgeOffset)
% → tableCenterX = bottleX – (–tableSide/2 + edgeOffset)
tableCenterX = bottlePosition(1) - ( -tableSide/2 + edgeOffset );
tableCenterY = bottlePosition(2);

%— Corners relative to center —%
xRel = [-1  1  1 -1] * (tableSide/2);
yRel = [-1 -1  1  1] * (tableSide/2);

%--- Table Top ---%
patch(xRel+tableCenterX, yRel+tableCenterY, tableTopZ*ones(1,4), [0.8 0.8 0.7], 'EdgeColor','none');
patch(xRel+tableCenterX, yRel+tableCenterY, tableBottomZ*ones(1,4), [0.8 0.8 0.7], 'EdgeColor','none');
for k = 1:4
    k2 = mod(k,4)+1;
    patch( ...
      xRel([k k2 k2 k])+tableCenterX, ...
      yRel([k k2 k2 k])+tableCenterY, ...
      [tableBottomZ tableBottomZ tableTopZ tableTopZ], ...
      [0.8 0.8 0.7], 'EdgeColor','none' );
end

%--- Legs ---%
legWidth  = 0.05;
legHeight = tableBottomZ;
legXs_rel = [-tableSide/2+legWidth/2, tableSide/2-legWidth/2];
legYs_rel = legXs_rel;
for ix_rel = legXs_rel
  for iy_rel = legYs_rel
    ix = ix_rel + tableCenterX;
    iy = iy_rel + tableCenterY;
    % bottom/top faces
    fill3([ix-legWidth/2,ix+legWidth/2,ix+legWidth/2,ix-legWidth/2], ...
          [iy-legWidth/2,iy-legWidth/2,iy+legWidth/2,iy+legWidth/2], ...
          [0 0 0 0], [0.6 0.4 0.2]);
    fill3([ix-legWidth/2,ix+legWidth/2,ix+legWidth/2,ix-legWidth/2], ...
          [iy-legWidth/2,iy-legWidth/2,iy+legWidth/2,iy+legWidth/2], ...
          [legHeight,legHeight,legHeight,legHeight], [0.6 0.4 0.2]);
    % vertical faces
    patch([ix-legWidth/2,ix+legWidth/2,ix+legWidth/2,ix-legWidth/2], ...
          [iy-legWidth/2,iy-legWidth/2,iy-legWidth/2,iy-legWidth/2], ...
          [0 0 legHeight legHeight], [0.6 0.4 0.2], 'EdgeColor','none');
    patch([ix+legWidth/2,ix+legWidth/2,ix+legWidth/2,ix+legWidth/2], ...
          [iy-legWidth/2,iy+legWidth/2,iy+legWidth/2,iy-legWidth/2], ...
          [0 0 legHeight legHeight], [0.6 0.4 0.2], 'EdgeColor','none');
    patch([ix+legWidth/2,ix-legWidth/2,ix-legWidth/2,ix+legWidth/2], ...
          [iy+legWidth/2,iy+legWidth/2,iy+legWidth/2,iy+legWidth/2], ...
          [0 0 legHeight legHeight], [0.6 0.4 0.2], 'EdgeColor','none');
    patch([ix-legWidth/2,ix-legWidth/2,ix-legWidth/2,ix-legWidth/2], ...
          [iy+legWidth/2,iy-legWidth/2,iy-legWidth/2,iy+legWidth/2], ...
          [0 0 legHeight legHeight], [0.6 0.4 0.2], 'EdgeColor','none');
  end
end

%--- Draw Bottle ---%
[cx, cy, cz] = cylinder(bottleRadius, 30);
cz = cz * bottleHeight + bottlePosition(3);
surf(cx + bottlePosition(1), cy + bottlePosition(2), cz, ...
     'FaceColor',[0.7 0.3 0.2], 'EdgeColor','none');

view(45,30)
title('Square Table with Bottle Placed 10 cm from –X Edge')



%% -- (2) Build the collisionBox for the tabletop -- %%

% collisionBox takes [X, Y, Z] size, and is centered at its Pose translation
tableBox = collisionBox( tableSide, tableSide, tableTopThickness );

% Place its geometric center at (tableCenterX, tableCenterY, middle-of-thickness)
tableBox.Pose = trvec2tform([ ...
    tableCenterX, ...
    tableCenterY, ...
    tableBottomZ + tableTopThickness/2  ...
]);


