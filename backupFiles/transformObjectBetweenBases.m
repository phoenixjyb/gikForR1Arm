function [T_B, mugInB] = transformObjectBetweenBases(robotA, robotB, target_obj, baseA, baseB, mugInA)
%TRANSFORMTRAJECTORYBETWEENBASES Transforms trajectory T_A from RobotA's base frame
% to RobotB's base frame using each robot's URDF and home configuration.
%
% Inputs:
%   robotA - RigidBodyTree of Robot A
%   robotB - RigidBodyTree of Robot B
%   T_A    - 4xN points in terms of positions in baseA frame (Robot A's base frame)
%   baseA  - string name of Robot A's arm base link (e.g., 'torso_link')
%   baseB  - string name of Robot B's arm base link (e.g., 'left_arm_base_link')
%
% Output:
%   T_B    - 4x4xN array of poses transformed into baseB frame

    % Get home configurations
    cfgA = homeConfiguration(robotA);
    cfgB = homeConfiguration(robotB);

    % Get transforms from world to each robot's base
    T_world_to_A = getTransform(robotA, cfgA, baseA);
    T_world_to_B = getTransform(robotB, cfgB, baseB);

    % Compute the transform from baseA → baseB
    T_A_to_B = T_world_to_A \ T_world_to_B;
    T_B = T_A_to_B * target_obj;

% now shape B structure 
mugInB = mugInA ; % initialization
mugInB.pose = T_A_to_B * mugInA.pose;
mugInB.collision.Pose = T_A_to_B * mugInA.collision.Pose;
mugInB.graspPose = T_A_to_B * mugInA.graspPose;

end
