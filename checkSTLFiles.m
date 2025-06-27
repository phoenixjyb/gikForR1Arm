% checkSTLFiles.m
% Check existence and readability of left arm STL files in R1Meshes

leftArmLinks = {
    'left_arm_link1', 'left_arm_link2', 'left_arm_link3', ...
    'left_arm_link4', 'left_arm_link5', 'left_arm_link6'
};

meshDir = 'R1Meshes';
missingFiles = {};
unreadableFiles = {};

for i = 1:numel(leftArmLinks)
    linkName = leftArmLinks{i};
    stlFile = fullfile(meshDir, [linkName, '.STL']);
    fprintf('Checking %s... ', stlFile);
    if exist(stlFile, 'file')
        fprintf('Exists. ');
        try
            TR = stlread(stlFile);
            if isempty(TR)
                fprintf('Unreadable (empty output).\n');
                unreadableFiles{end+1} = stlFile;
            else
                fprintf('Readable.\n');
                disp(TR);
            end
        catch ME
            fprintf('Unreadable (error: %s).\n', ME.message);
            unreadableFiles{end+1} = stlFile;
        end
    else
        fprintf('Missing!\n');
        missingFiles{end+1} = stlFile;
    end
end

fprintf('\nSummary:\n');
if isempty(missingFiles)
    fprintf('All STL files exist.\n');
else
    fprintf('Missing files:\n');
    disp(missingFiles');
end
if isempty(unreadableFiles)
    fprintf('All STL files are readable.\n');
else
    fprintf('Unreadable files:\n');
    disp(unreadableFiles');
end 