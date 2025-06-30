% stitch_videos.m
% Stitch three robot grasp videos side-by-side and delete originals

% Define input and output filenames
vidFiles = {'robot_grasp_animation.mp4', 'robot_grasp_topview.mp4', 'robot_grasp_frontview.mp4'};
outFile = 'robot_grasp_stitched.mp4';

% Create VideoReader objects
vids = cellfun(@VideoReader, vidFiles, 'UniformOutput', false);

% Check frame rates and sizes
frameRate = vids{1}.FrameRate;
frameSize = size(readFrame(vids{1}));
vids{1} = VideoReader(vidFiles{1});

for k = 2:3
    if vids{k}.FrameRate ~= frameRate
        error('Frame rates do not match!');
    end
    sz = size(readFrame(vids{k}));
    vids{k} = VideoReader(vidFiles{k});
    if ~isequal(sz, frameSize)
        error('Frame sizes do not match!');
    end
end

% Create VideoWriter for output
outputVideo = VideoWriter(outFile, 'MPEG-4');
outputVideo.FrameRate = frameRate;
open(outputVideo);

% Stitch frames
while all(cellfun(@hasFrame, vids))
    frames = cellfun(@readFrame, vids, 'UniformOutput', false);
    stitchedFrame = cat(2, frames{:});
    writeVideo(outputVideo, stitchedFrame);
end

close(outputVideo);
disp(['Stitched video saved as ', outFile]);

% Delete original videos
for k = 1:3
    if exist(vidFiles{k}, 'file')
        delete(vidFiles{k});
        disp(['Deleted ', vidFiles{k}]);
    end
end 