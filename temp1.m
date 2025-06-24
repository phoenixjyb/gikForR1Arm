% Create a new rigidBodyTree
robotTest = rigidBodyTree;

% Create a new rigid body and joint
body = rigidBody('test_link');
jnt = rigidBodyJoint('test_joint','fixed');
body.Joint = jnt;

% Add the body to the robot
addBody(robotTest, body, robotTest.BaseName);

% Create a collision object
cb = collisionBox(0.1, 0.1, 0.1);

% Store the collision object in a map, keyed by body name
collisionMap = containers.Map;
collisionMap('test_link') = cb;

% Show the robot and the collision object together
ax = show(robotTest);
hold(ax, 'on');
show(cb, "Parent", ax, "Pose", trvec2tform([0 0 0])); % Adjust pose as needed
hold(ax, 'off');

% Access and manipulate the collision object later
myCollisionObj = collisionMap('test_link');
disp(myCollisionObj);

class(cb)
methods(cb)