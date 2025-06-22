function eeTrajectory = receiveEEPoseTrajectory(node, topicName, timeoutSec)
% Receive a sequence of end-effector poses from a ROS2 topic
% node: ros2node object
% topicName: string, e.g., '/robotA/ee_trajectory'
% timeoutSec: wait time in seconds

    sub = ros2subscriber(node, topicName, 'DataFormat', 'struct');
    disp('Waiting for EE trajectory...');
    msg = receive(sub, timeoutSec);

    if isempty(msg)
        error('No message received on %s within %d seconds.', topicName, timeoutSec);
    end

    % Assume msg.Poses is an array of PoseStamped (or custom 4x4 packed form)
    numPoses = numel(msg.Poses);
    eeTrajectory = zeros(4,4,numPoses);
    
    for i = 1:numPoses
        p = msg.Poses(i).Pose;
        R = quat2rotm([p.Orientation.W p.Orientation.X p.Orientation.Y p.Orientation.Z]);
        T = trvec2tform([p.Position.X p.Position.Y p.Position.Z]) * rotm2tform(R);
        eeTrajectory(:,:,i) = T;
    end

    disp("Received EE trajectory of length " + numPoses);
end