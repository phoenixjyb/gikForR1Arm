function visualizeRobotScene(robotB, bottleInB, tableBox, eePosBs)
    % If eePosBs is not defined or empty, create an empty array
    if nargin < 4 || isempty(eePosBs)
        eePosBs = [];
    end
    % Visualize the robot, bottle, table, and reachable points
    
    % Create figure
    figure
    ax = axes; view(3); axis equal; grid on;
    hold on

    % Show robot
    show(robotB, 'PreservePlot', false, 'Parent', ax);

    % Show bottle by finding the new patch handle
    patches_before_bottle = findall(ax, 'Type', 'Patch');
    show(bottleInB.collision, 'Parent', ax);
    patches_after_bottle = findall(ax, 'Type', 'Patch');
    h_bottle = setdiff(patches_after_bottle, patches_before_bottle);
    if ~isempty(h_bottle)
        set(h_bottle, 'FaceColor', [0 0.4470 0.7410], 'EdgeColor', 'none'); % Blue color for bottle
    end
    plotTransforms(tform2trvec(bottleInB.graspPose), tform2quat(bottleInB.graspPose), 'FrameSize', 0.05, 'Parent', ax);

    % Show table by finding the new patch handle
    patches_before_table = findall(ax, 'Type', 'Patch');
    show(tableBox, 'Parent', ax);
    patches_after_table = findall(ax, 'Type', 'Patch');
    h_table = setdiff(patches_after_table, patches_before_table);
    if ~isempty(h_table)
        set(h_table, 'FaceColor', [0.8500 0.3250 0.0980], 'EdgeColor', 'none'); % Wood-like color for table
    end

    % Show reachable points
    if ~isempty(eePosBs)
        scatter3(ax, eePosBs(:,1), eePosBs(:,2), eePosBs(:,3), 2, 'b', 'filled');
    end

    % Add labels and legend
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Robot, Bottle, Table, and Reachable Points');
    legend('Robot', 'Bottle', 'Grasp Pose', 'Table', 'Reachable Points');
end
