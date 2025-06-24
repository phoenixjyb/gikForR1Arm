% Create a new rigidBodyTree
robotTest = rigidBodyTree;

% Create a new rigid body and joint
body = rigidBody('test_link');
jnt = rigidBodyJoint('test_joint','fixed');
body.Joint = jnt;

% Add a collision box to the body
addCollision(body, collisionBox(0.1, 0.1, 0.1));

% Add the body to the robot
addBody(robotTest, body, robotTest.BaseName);

% Display the Collisions property
disp('Collisions for test_link:');
disp(body.Collisions);

% Check the class of the first collision object
if ~isempty(body.Collisions)
    disp(['Collision object class: ', class(body.Collisions{1})]);
end