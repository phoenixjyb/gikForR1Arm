%% create a bottle so that robot can reach
addpath(fullfile(pwd, 'backupFiles', 'reachableWorkspace'));
robotB = importrobot('fineUrdfs/r1_v2_1_0.urdf'); % Use full-body URDF for consistency
robotB.DataFormat = 'struct';
robotB.Gravity = [0 0 -9.81];
endEffectorNameB = 'left_gripper_link';
numSamples = 5000;
eePosBs = computeReachableWorkspace(robotB,endEffectorNameB,numSamples);
[x,y,z,bottleInB,tableBox] = createBottleAndTable(eePosBs,robotB); 
save ('bottle.mat','bottleInB');
save ('table.mat','tableBox');

% this file is only for saving bottle and table as object 
