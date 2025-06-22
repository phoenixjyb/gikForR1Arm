function cfgStruct = vectorToConfig(robot, qVec)
    cfgTemplate = homeConfiguration(robot);
    for i = 1:numel(cfgTemplate)
        cfgTemplate(i).JointPosition = qVec(i);
    end
    cfgStruct = cfgTemplate;
end