function visualizeDualRobotJointTrajectories(robotA, qA_traj, robotB, qB_traj)
    % --- Pad robotA trajectory to match robotB length ---
    numStepsA = size(qA_traj, 1);
    numStepsB = size(qB_traj, 1);

    if numStepsA < numStepsB
        pad_steps = numStepsB - numStepsA;
        pad_block = repmat(qA_traj(1,:), pad_steps, 1);
        qA_traj = [pad_block; qA_traj];
    end

    numSteps = size(qB_traj, 1);  % Now both should be equal
    numJointsA = size(qA_traj, 2);
    numJointsB = size(qB_traj, 2);

    % qA and qB are all for robot joint's angles, but not each joint's
    % cartisan coordinates. show


    % --- Setup animation figure ---
    figure('Name','Dual Robot Animation','Position',[100 100 1200 600])
    tiledlayout(1,2);

    axA = nexttile(1); title('Robot A');
    axB = nexttile(2); title('Robot B');

    % axis(axA, [-1 1 -1 1 0 1.5]); view(axA, 3);
    % axis(axB, [-1 1 -1 1 0 1.5]); view(axB, 3);

    % --- Animation Loop ---
    for t = 1:numSteps
        cfgA = structFromVector(robotA, qA_traj(t,:));
        % axis(axA, 'auto');
        % show(robotA, cfgA, 'PreservePlot', false, 'Frames','off', 'Parent', axA);
        show(robotA,cfgA,'Parent', axA);
        view(axA, 3)

        cfgB = structFromVector(robotB, qB_traj(t,:));
        % axis(axB, 'auto');
        % show(robotB, cfgB, 'PreservePlot', false, 'Frames','off', 'Parent', axB);
        show(robotB,cfgB,'Parent', axB)
        view(axA, 3)

        sgtitle(sprintf('Step %d / %d', t, numSteps));
        drawnow;
    end
    


    % --- Joint Position Plot ---
    figure('Name','Joint Position Comparison');
    numSubplots = max(numJointsA, numJointsB);
    for j = 1:numSubplots
        subplot(ceil(numSubplots/2),2,j);
        hold on;
        if j <= numJointsA
            plot(rad2deg(qA_traj(:,j)), 'b-', 'DisplayName','robotA');
        end
        if j <= numJointsB
            plot(rad2deg(qB_traj(:,j)), 'r--', 'DisplayName','robotB');
        end
        xlabel('Time Step'); ylabel('Angle (deg)');
        title(sprintf('Joint %d', j));
        legend; grid on;
    end
end

function configStruct = structFromVector(robot, qvec)
    cfgTemplate = homeConfiguration(robot);
    for i = 1:numel(cfgTemplate)
        cfgTemplate(i).JointPosition = qvec(i);
    end
    configStruct = cfgTemplate;
end