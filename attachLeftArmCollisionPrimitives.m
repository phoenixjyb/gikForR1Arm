function robot = attachLeftArmCollisionPrimitives(robot, meshDir, primitiveType)
% attachLeftArmCollisionPrimitives attaches collision primitives to left arm links
%   robot = attachLeftArmCollisionPrimitives(robot, meshDir, primitiveType)
%   - robot: rigidBodyTree object
%   - meshDir: directory containing STL files
%   - primitiveType: 'box' (default) or 'cylinder'
%   This function attaches either collisionBox or collisionCylinder to each left arm link
%   based on the STL geometry in meshDir.

if nargin < 3
    primitiveType = 'box';
end

sizeCoeff = 0.5; % Coefficient to scale primitive size (e.g., 0.5 for half size )
% this coefficient is put on purpose to make the collision primitives smaller
% than the original mesh, so that the collision check is more accurate  

leftArmLinks = {
    'left_arm_link1', 'left_arm_link2', 'left_arm_link3', ...
    'left_arm_link4', 'left_arm_link5', 'left_arm_link6'
};

for i = 1:numel(leftArmLinks)
    linkName = leftArmLinks{i};
    stlFile = fullfile(meshDir, [linkName, '.STL']);
    if exist(stlFile, 'file')
        % Robust STL reading (try triangulation, then fallback)
        try
            TR = stlread(stlFile);
            vertices = TR.Points;
        catch
            warning(['Failed to read STL for ', linkName, ', skipping.']);
            continue;
        end
        minV = min(vertices, [], 1);
        maxV = max(vertices, [], 1);
        boxSize = maxV - minV;
        boxCenter = (minV + maxV) / 2;
        if numel(boxCenter) < 3
            boxCenter(1,3) = 0;
        end
        boxCenter = boxCenter(1:3);
        % Find body index
        bodyIdx = find(strcmp(cellfun(@(b) b.Name, robot.Bodies, 'UniformOutput', false), linkName));
        if isempty(bodyIdx)
            warning(['Link ', linkName, ' not found in robot model!']);
            continue;
        end
        % Attach the chosen primitive
        switch lower(primitiveType)
            case 'box'
                cb = collisionBox(boxSize(1)*sizeCoeff, boxSize(2)*sizeCoeff, boxSize(3)*sizeCoeff);
                cb.Pose = trvec2tform(boxCenter);
                addCollision(robot.Bodies{bodyIdx}, cb, cb.Pose);
            case 'cylinder'
                zmin = min(vertices(:,3));
                zmax = max(vertices(:,3));
                cylLength = zmax - zmin;
                xy = vertices(:,1:2);
                center = mean(xy,1);
                radii = sqrt(sum((xy - center).^2,2));
                cylRadius = max(radii);
                cylCenter = [center, (zmin+zmax)/2];
                if numel(cylCenter) < 3
                    cylCenter(1,3) = 0;
                end
                cylCenter = cylCenter(1:3);
                if cylRadius > 0 && cylLength > 0
                    cyl = collisionCylinder(cylRadius*sizeCoeff, cylLength*sizeCoeff);
                    cyl.Pose = trvec2tform(cylCenter);
                    addCollision(robot.Bodies{bodyIdx}, cyl, cyl.Pose);
                else
                    warning('Skipping %s: computed cylinder radius or length not positive (radius=%.4f, length=%.4f)', linkName, cylRadius, cylLength);
                end
            otherwise
                error('Unknown primitiveType: %s. Use "box" or "cylinder".', primitiveType);
        end
    else
        warning(['STL file not found for ', linkName]);
    end
end
end 