robot = loadrobot('kinovaGen3', 'DataFormat', 'row');

% Define obstacles (empty for minimal example)
env = {};

% Create the manipulatorRRT planner
planner = manipulatorRRT(robot, env);
planner.MaxConnectionDistance = 0.1;
planner.MaxIterations = 3000;

% Define start and goal configurations
qStart = homeConfiguration(robot);
qGoal = qStart;
qGoal(2) = qGoal(2) + 1.0; % Example goal

% Plan a path
[pathObj, solnInfo] = plan(planner, qStart, qGoal);

disp('Planner created and plan attempted!');