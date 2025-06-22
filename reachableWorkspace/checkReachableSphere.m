robotB = importrobot('r1_leftArm_only_arm_down.urdf');
robotB.DataFormat = 'row';
eeNameB = 'left_gripper_link';  % ← your actual EE link
[eePoints] = computeReachableWorkspace(robotB, eeNameB, 10000);
hold on;
show(robot);

