function publishSourceTrajectory(node, topicName, T_all)
    pub = ros2publisher(node, topicName, 'geometry_msgs/PoseArray');
    msg = transformTrajectoryToPoseArrayMsg(T_all, 'torso_link');
    send(pub, msg);
    disp("Published EE trajectory with " + size(T_all,3) + " steps.");
end