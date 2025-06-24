function visualizeRobotSceneWithCollisions(robot, tableBox, cardbox, bottleInB)
% Visualize robot with collision boxes, table, cardbox, and bottle

figure; hold on; axis equal; grid on;
show(robot, homeConfiguration(robot), 'Collisions', 'on', 'PreservePlot', false);

if exist('tableBox', 'var') && ~isempty(tableBox)
    show(tableBox);
end
if exist('cardbox', 'var') && ~isempty(cardbox)
    show(cardbox);
end
if exist('bottleInB', 'var') && isfield(bottleInB, 'collision') && ~isempty(bottleInB.collision)
    show(bottleInB.collision);
end

title('Robot with Table, Cardbox, Bottle, and Arm Collision Boxes');
legend('Robot Collision Geometry', 'Table', 'Cardbox', 'Bottle');
view(3);
end 