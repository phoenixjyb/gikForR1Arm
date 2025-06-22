function egHelperPlotCupAndTable(cupHeight, cupRadius, cupPosition)
%exampleHelperPlotCupAndTable Adds a cup to the current axes

%   Copyright 2016-2020 The MathWorks, Inc.

% Create points for visualizing a cup
[X,Y,Z] = cylinder(cupRadius*linspace(0,1,50).^0.125);
% Scale the Z coordinates
Z = cupHeight*Z - cupHeight/2;
% Translate to the specified position
X = X + cupPosition(1);
Y = Y + cupPosition(2);
Z = Z + cupPosition(3);
% Add the cup to the figure and configure lighting
s = patch(surf2patch(X,Y,Z));
s.FaceColor = 'blue';
s.FaceLighting = 'gouraud';
s.EdgeAlpha = 0;
% Move the light so that the side of the cup is illuminated
lightObj = findobj(gca,'Type','Light');
for i = 1:length(lightObj)
    lightObj(i).Position = [1,1,1];
end

% Create points for visualizing a table
X = [0.3,-0.7,-0.7,0.3];
Y = [0.7,0.7,-0.3,-0.3];
Z = [0,0,0,0];
% Add the table to the figure
patch(X,Y,Z,0.8*[1,1,1]);

% Adjust the camera and axis limits
axis([-0.85, 0.45, -0.45, 0.85, 0, 1.35]);
campos([7.84, 8.17,2.05]); camva(6.9); camtarget([-0.16, 0.18, 0.55]);