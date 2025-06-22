function jointLim = setJointPositionBounds(jointLim, jointName, limits)
    % Sets joint position bounds for a given joint name
    boundsArray = jointLim.Bounds;
    for k = 1:numel(boundsArray)
        try
            name = boundsArray(k).JointName;  % if it's an object
        catch
            name = boundsArray{k}.JointName;  % if it's a struct inside cell
        end
        if strcmp(name, jointName)
            jointLim.Bounds(k).JointPositionBounds = limits;
            return;
        end
    end
    warning("Joint %s not found in joint constraints.", jointName);
end