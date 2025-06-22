function updatedHomePos = updateHomePositionforR1_rightArm(loadedHomePos)

updatedHomePos = loadedHomePos;

% below are for left arm of galaxea r1;
% these values are specified in a urdf file but heed that urdf file
% contains both origin's rpy values and axis (with negative and positive signs)
% it is necessary to double check rpy value and axis sign when giving
% initial values

updatedHomePos(1).JointPosition = -1.56; % already manually specified with radian
updatedHomePos(2).JointPosition = 2.94; % already manually specified with radian
updatedHomePos(3).JointPosition = -2.54; % already manually specified with radian
updatedHomePos(4).JointPosition = 0; % already manually specified with radian
updatedHomePos(5).JointPosition = 0; % already manually specified with radian
updatedHomePos(6).JointPosition = 0; % already manually specified with radian

% right arm only changes the first entry, i.e., -1.56 rather than 1.56

end