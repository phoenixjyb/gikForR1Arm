%% create a bottle so that robot can reach
addpath(fullfile(pwd, 'backupFiles'));
addpath(fullfile(pwd, 'backupFiles', 'reachableWorkspace'));
robotB = importrobot('fineUrdfs/r1_v2_1_0.urdf'); % Use full-body URDF for consistency
robotB.DataFormat = 'struct';
robotB.Gravity = [0 0 -9.81];
endEffectorNameB = 'left_gripper_link';

% Calculate reachable workspace points using the unified function
leftArmJoints = {'left_arm_joint1','left_arm_joint2','left_arm_joint3','left_arm_joint4','left_arm_joint5','left_arm_joint6'};
eePosBs = sampleReachableWorkspace(robotB, endEffectorNameB, 5000, false, leftArmJoints);

[bottleInB,tableBox,cardbox] = generateBottleAndTable(eePosBs,robotB); 
save ('bottle.mat','bottleInB');
save ('table.mat','tableBox');
save ('cardbox.mat','cardbox');

% Check if bottle's grasp pose is reachable
graspPos = tform2trvec(bottleInB.graspPose);
dists = vecnorm(eePosBs - graspPos, 2, 2);
if all(dists > 0.05)
    warning('Bottle grasp pose is NOT within 5 cm of any sampled reachable workspace point!');
    % Move bottle to the closest reachable point
    [~, idxClosest] = min(dists);
    newBottlePos = eePosBs(idxClosest, :);
    fprintf('Trying to move bottle to closest reachable point: (%.3f, %.3f, %.3f)\n', newBottlePos(1), newBottlePos(2), newBottlePos(3));
    % Update bottle pose and grasp pose
    bottleCandidate = bottleInB;
    bottleCandidate.pose = trvec2tform(newBottlePos);
    T_graspLocal = bottleCandidate.pose \ bottleCandidate.graspPose;
    bottleCandidate.graspPose = bottleCandidate.pose * T_graspLocal;
    % Try IK for the new grasp pose
    gik = generalizedInverseKinematics('RigidBodyTree', robotB, 'ConstraintInputs', {'pose','joint'});
    poseTgt = constraintPoseTarget(endEffectorNameB);
    poseTgt.TargetTransform = bottleCandidate.graspPose;
    poseTgt.Weights = [1,1];
    jointTgt = constraintJointBounds(robotB);
    initialGuess = homeConfiguration(robotB);
    [qSol, solutionInfo] = gik(initialGuess, poseTgt, jointTgt);
    if strcmpi(solutionInfo.Status, 'success')
        disp('IK succeeded for new bottle position. Bottle updated.');
        bottleInB = bottleCandidate;
        save('bottle.mat','bottleInB');
        save('table.mat','tableBox');
    else
        warning('IK failed for the closest reachable point. Bottle not updated.');
    end
else
    disp('Bottle grasp pose is reachable by the arm.');
end

% this file is only for saving bottle and table as object 
