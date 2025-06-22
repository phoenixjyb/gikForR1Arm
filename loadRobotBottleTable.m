%% create a bottle so that robot can reach
robotB = importrobot('r1_leftArm_only_arm_down.urdf'); % it seems that arms from down position as initial states can 
robotB.DataFormat = 'struct';
robotB.Gravity = [0 0 -9.81];
endEffectorNameB = robotB.BodyNames{end};
numSamples = 5000;
eePosBs = computeReachableWorkspace(robotB,endEffectorNameB,numSamples);
[x,y,z,bottleInB,tableBox] = createBottleAndTable(eePosBs,robotB); 
save ('bottle.mat','bottleInB');
save ('table.mat','tableBox');

% this file is only for saving bottle and table as object 
