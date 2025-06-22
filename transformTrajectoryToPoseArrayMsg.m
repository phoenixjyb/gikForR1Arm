function msg = transformTrajectoryToPoseArrayMsg(T_all, frameId)
    numSteps = size(T_all, 3);
    msg = ros2message('geometry_msgs/PoseArray');
    msg.Header.FrameId = frameId;
    % msg.Header.Stamp = ros2time(datetime('now'));
    % Correctly set time stamp
    msg.Header.Stamp = ros2time(seconds(datetime('now') - datetime(1970,1,1)));
    poses(numSteps) = ros2message('geometry_msgs/Pose');
    for i = 1:numSteps
        T = T_all(:,:,i);
        pos = tform2trvec(T);
        quat = rotm2quat(tform2rotm(T));
        poses(i).Position.X = pos(1); poses(i).Position.Y = pos(2); poses(i).Position.Z = pos(3);
        poses(i).Orientation.W = quat(1); poses(i).Orientation.X = quat(2);
        poses(i).Orientation.Y = quat(3); poses(i).Orientation.Z = quat(4);
    end
    msg.Poses = poses;
end