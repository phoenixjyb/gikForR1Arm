function publishJointTrajectory(node, topicName, q_traj, jointNames)
% Publish joint trajectory to a ROS2 topic
% q_traj: NxJ matrix
% jointNames: 1xJ cell array of joint names
   

    pub = ros2publisher(node, topicName, 'trajectory_msgs/JointTrajectory');
    msg = ros2message(pub);

    msg.JointNames = jointNames;
    numPoints = size(q_traj, 1);

    for i = 1:numPoints
        pt = ros2message('trajectory_msgs/JointTrajectoryPoint');
        pt.Positions = q_traj(i, :);
        pt.TimeFromStart = duration(0, 0, i * 0.1);  % hours, minutes, seconds
        % pt.TimeFromStart = builtin('duration', 0, 0, i * 0.1);  % 0.1s per step
        msg.Points(i) = pt;
    end

    send(pub, msg);
    disp("Published joint trajectory with " + numPoints + " steps.");
end