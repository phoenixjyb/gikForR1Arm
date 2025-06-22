load ("mugs.mat")
robotB = importrobot ('r1_leftArm_only_arm_down.urdf');
ik = inverseKinematics('RigidBodyTree', robotB);
weights = [0.5 0.5 0.5 1 1 1];   % tune these if needed
initialGuess = homeConfiguration(robotB);

EEname = 'left_gripper_link';
[qGrasp, solInfo] = ik(EEname, mugInB.graspPose, weights, initialGuess);

close all
figure; hold on; axis equal; view(3); grid on;
show(robotB, qGrasp, 'PreservePlot', false);
show(mugInB.collision);
plotTransforms(tform2trvec(mugInB.graspPose), tform2quat(mugInB.graspPose), 'FrameSize', 0.05);

approachOffset = trvec2tform([0, 0, -0.1]);  % approach from above
approachPose = mugInB.graspPose * approachOffset;
[qApproach, info2] = ik('left_gripper_link', approachPose, weights, qGrasp);

%% animate process of ik 
n = 50;  % number of interpolation steps
traj = zeros(n, numel(qGrasp));
for i = 1:numel(qGrasp)
    traj(:,i) = linspace(qApproach(i).JointPosition, qGrasp(i).JointPosition, n);
end

figure;
for t = 1:n
    qStep = qGrasp;
    for j = 1:numel(qStep)
        qStep(j).JointPosition = traj(t,j);
    end
    show(robotB, qStep, 'PreservePlot', false);
    drawnow;
end

