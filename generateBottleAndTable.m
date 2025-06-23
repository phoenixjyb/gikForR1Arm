% --- Given ---

function [bottleInB, tableBox, cardbox] = generateBottleAndTable(eePosBs, robotB)

% --- Parameters ---
% Bottle
bottlePosition = [0.4, 0, 1.15];
bottleRadius = 0.03;
bottleHeight = 0.21;

% Table
tableSide = 0.6;
edgeOffset = 0.10;
tableTopThickness = 0.02;
tableTopZ = bottlePosition(3);
tableBottomZ = tableTopZ - tableTopThickness;

% Cardbox
cardboxLength = 0.2;
cardboxWidth  = 0.2;
cardboxHeight = 0.20;

% --- Cardbox Placement ---
cardboxCenterX = bottlePosition(1);
cardboxCenterY = bottlePosition(2);
cardboxCenterZ = tableTopZ + cardboxHeight/2;

% --- Bottle Placement ---
bottleBaseZ = cardboxCenterZ + cardboxHeight/2;
bottleCenter = [bottlePosition(1), bottlePosition(2), bottleBaseZ];
bottlePose = trvec2tform(bottleCenter);

% --- Table Placement ---
tableCenterX = bottlePosition(1) - ( -tableSide/2 + edgeOffset );
tableCenterY = bottlePosition(2);

% --- Create collision objects ---
bottle = struct();
bottle.radius = bottleRadius;
bottle.height = bottleHeight;
bottle.pose = bottlePose;
bottle.collision = collisionCylinder(bottle.radius, bottle.height);
bottle.collision.Pose = bottlePose * trvec2tform([0, 0, bottleHeight/2]);
graspOffset = trvec2tform([bottle.radius + 0.02, 0, bottle.height/2]) * axang2tform([0 1 0 pi/2]);
bottle.graspPose = bottle.pose * graspOffset;

cardbox = collisionBox(cardboxLength, cardboxWidth, cardboxHeight);
cardbox.Pose = trvec2tform([cardboxCenterX, cardboxCenterY, cardboxCenterZ]);

tableBox = collisionBox(tableSide, tableSide, tableTopThickness);
tableBox.Pose = trvec2tform([tableCenterX, tableCenterY, tableBottomZ + tableTopThickness/2]);

% --- Visualization ---
figure; hold on; grid on; axis equal; view(3);
scatter3(eePosBs(:,1), eePosBs(:,2), eePosBs(:,3), 2, 'b', 'filled');
show(robotB);

% Show cardbox in orange
patches_before = findobj(gca, 'Type', 'Patch');
show(cardbox);
patches_after = findobj(gca, 'Type', 'Patch');
h_cardbox = setdiff(patches_after, patches_before);
if ~isempty(h_cardbox)
    set(h_cardbox, 'FaceColor', [1.0 0.6 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
end

% Show bottle in blue
patches_before = findobj(gca, 'Type', 'Patch');
show(bottle.collision);
patches_after = findobj(gca, 'Type', 'Patch');
h_bottle = setdiff(patches_after, patches_before);
if ~isempty(h_bottle)
    set(h_bottle, 'FaceColor', [0.2 0.5 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
end

% Show table in light gray
patches_before = findobj(gca, 'Type', 'Patch');
show(tableBox);
patches_after = findobj(gca, 'Type', 'Patch');
h_table = setdiff(patches_after, patches_before);
if ~isempty(h_table)
    set(h_table, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robot, Bottle, Cardbox, Table, and Reachable Points');
legend('Reachable Points', 'Robot', 'Cardbox', 'Bottle', 'Table');

% --- Print object dimensions and positions for debugging ---
fprintf('\n--- Object Dimensions and Positions ---\n');
% Table
fprintf('Table center: (%.3f, %.3f, %.3f) m\n', tableCenterX, tableCenterY, tableBox.Pose(3,4));
fprintf('Table size:   %.3f x %.3f x %.3f m\n', tableSide, tableSide, tableTopThickness);
fprintf('Table top Z:  %.3f m\n', tableBox.Pose(3,4) + tableTopThickness/2);
% Cardbox
fprintf('Cardbox center: (%.3f, %.3f, %.3f) m\n', cardboxCenterX, cardboxCenterY, cardboxCenterZ);
fprintf('Cardbox size:   %.3f x %.3f x %.3f m\n', cardboxLength, cardboxWidth, cardboxHeight);
fprintf('Cardbox top Z:  %.3f m\n', cardboxCenterZ + cardboxHeight/2);
% Bottle
fprintf('Bottle base (pose): (%.3f, %.3f, %.3f) m\n', bottle.pose(1,4), bottle.pose(2,4), bottle.pose(3,4));
fprintf('Bottle height: %.3f m\n', bottleHeight);
fprintf('Bottle top Z:  %.3f m\n', bottle.pose(3,4) + bottleHeight);
fprintf('--------------------------------------\n');

bottleInB = bottle;


