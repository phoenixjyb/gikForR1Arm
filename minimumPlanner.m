robot = loadrobot('kinovaGen3', 'DataFormat', 'row');
ss = manipulatorStateSpace(robot);
sv = manipulatorCollisionBodyValidator(ss);
planner = manipulatorRRT(ss, sv);
disp('Planner created successfully!');