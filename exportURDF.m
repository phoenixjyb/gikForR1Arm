function exportURDF(robot, filename)
%EXPORTURDF Export a rigidBodyTree to a URDF file.
%   exportURDF(robot, filename) writes a basic URDF describing the
%   kinematic structure of the input robot to the given filename.

    fid = fopen(filename, 'w');
    if fid == -1
        error('Cannot open file: %s', filename);
    end

    fprintf(fid, '<?xml version="1.0" ?>\n');
    fprintf(fid, '<robot name="%s">\n', inputname(1));

    % Export each body
    for i = 1:numel(robot.Bodies)
        body = robot.Bodies{i};
        joint = body.Joint;

        % Write link
        fprintf(fid, '  <link name="%s"/>\n', body.Name);

        % Skip fixed base parent (already covered)
        if strcmp(robot.BaseName, body.Parent.Name)
            parentName = robot.BaseName;
        else
            parentName = body.Parent.Name;
        end

        % Write joint
        fprintf(fid, '  <joint name="%s" type="%s">\n', joint.Name, joint.Type);
        fprintf(fid, '    <parent link="%s"/>\n', parentName);
        fprintf(fid, '    <child link="%s"/>\n', body.Name);

        % Get joint transform
        tf = joint.JointToParentTransform;
        rpy = rotm2eul(tf(1:3,1:3), 'XYZ');
        xyz = tf(1:3,4);

        fprintf(fid, '    <origin xyz="%.6f %.6f %.6f" rpy="%.6f %.6f %.6f"/>\n', ...
            xyz(1), xyz(2), xyz(3), rpy(1), rpy(2), rpy(3));

        % Axis (for revolute or prismatic)
        if ~strcmp(joint.Type, 'fixed')
            axis = joint.JointAxis;
            fprintf(fid, '    <axis xyz="%.6f %.6f %.6f"/>\n', axis(1), axis(2), axis(3));
        end

        fprintf(fid, '  </joint>\n');
    end

    % Base link
    fprintf(fid, '  <link name="%s"/>\n', robot.BaseName);

    fprintf(fid, '</robot>\n');
    fclose(fid);

    fprintf('✅ URDF exported to %s\n', filename);
end