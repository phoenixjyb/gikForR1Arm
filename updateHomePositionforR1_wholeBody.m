function updatedHomePos = updateHomePositionforR1_wholeBody(loadedHomePos)

updatedHomePos = loadedHomePos;

% Define left arm values in radians
leftArmVals = [1.56,    % joint1
               2.94,    % joint2
               -2.54,   % joint3
               0,       % joint4
               0,       % joint5
               0];      % joint6

% Define right arm values in radians
rightArmVals = [-1.56,  % joint1
                2.94,   % joint2
                -2.54,  % joint3
                0,      % joint4
                0,      % joint5
                0];     % joint6

% Define arm joint names
leftArmJoints = {'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3', ...
                 'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6'};
             
rightArmJoints = {'right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3', ...
                  'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6'};

% Copy input configuration
updatedHomePos = loadedHomePos;

% Update left arm joints
for i = 1:length(leftArmJoints)
    for j = 1:length(updatedHomePos)
        if strcmp(updatedHomePos(j).JointName, leftArmJoints{i})
            updatedHomePos(j).JointPosition = leftArmVals(i);
            break;
        end
    end
end

% Update right arm joints
for i = 1:length(rightArmJoints)
    for j = 1:length(updatedHomePos)
        if strcmp(updatedHomePos(j).JointName, rightArmJoints{i})
            updatedHomePos(j).JointPosition = rightArmVals(i);
            break;
        end
    end
end

% updatedHomePos(1).JointPosition = 1.56; % already manually specified with radian
% updatdedHomePos(2).JointPosition = 2.94; % already manually specified with radian
% updatedHomePos(3).JointPosition = -2.54; % already manually specified with radian
% updatedHomePos(4).JointPosition = 0; % already manually specified with radian
% updatedHomePos(5).JointPosition = 0; % already manually specified with radian
% updatedHomePos(6).JointPosition = 0; % already manually specified with radian

% % below are for right arm of galaxea r1;

% updatedHomePos(1).JointPosition = -1.56; % already manually specified with radian
% updatedHomePos(2).JointPosition = 2.94; % already manually specified with radian
% updatedHomePos(3).JointPosition = -2.54; % already manually specified with radian
% updatedHomePos(4).JointPosition = 0; % already manually specified with radian
% updatedHomePos(5).JointPosition = 0; % already manually specified with radian
% updatedHomePos(6).JointPosition = 0; % already manually specified with radiani

% right arm only changes the first entry, i.e., -1.56 


end