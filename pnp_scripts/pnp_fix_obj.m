% This script pick and place object from Position_1 and Position_5 (these
% are fixed objects)
% The position of object's are preloaded

% open gripper:
% gripGoal=packGripGoal(0.01,gripGoal);
% Z-value is these are pick points/ to move incease the height to 0.4.(IMP)
% Note: Align the gripper and then decrease the height to pick and then
% again increse the height to drop.

% Objects to pick on position_1(region_1):
% 1.gripperTranslation = [0.46 -0.07 0.24]; gripperRotation = [pi/2 -pi 0];
% 1.gripGoal=packGripGoal(0.517,gripGoal);
% 2.gripperTranslation = [0.657 0.017 0.25]; gripperRotation = [pi/2 -pi 0];
% 2.gripGoal=packGripGoal(0.223,gripGoal);
% 3.gripperTranslation = [0.657 0.017 0.14]; gripperRotation = [pi/2 -pi 0];
% 3.gripGoal=packGripGoal(0.223,gripGoal);

% Object to pick on position_5(region_5):
% 1.gripperTranslation = [-0.36 -0.01 0.26]; gripperRotation = [pi/2 0 -pi];
% 1.gripGoal=packGripGoal(0.223,gripGoal);
% 2.gripperTranslation = [-0.50 0.395 0.14]; gripperRotation = [pi/2 0 -pi];
% 2.gripGoal=packGripGoal(0.223,gripGoal);
% 3.gripperTranslation = [-0.62 0.295 0.088]; gripperRotation = [0 0 -pi];
% 3.gripGoal=packGripGoal(0.209,gripGoal);
% 4. still 4th position in not determined


% Region 3
% the marker is moving (so execute this first)
% Obj 5 (marker)
trans = [0.0219 0.413 0.40];
rot = [pi-pi/2 -pi 0]; %[z ,y ,x]
g_val = 0.01;
run moveTo.m;
rgbImage = rosReadImage(rgbsub.LatestMessage);
[bboxes,scores,labels] = detect(detector_2, rgbImage); % marker detector_2
close all;
visDetection(rgbImage,bboxes,labels,scores); % visualize detection
if (find(labels=="marker"))
    initialIKGuess(2).JointPosition = -0.3;
    trans = [0.0219 0.413 0.07];
    rot = [pi-pi/2 -pi 0]; %[z ,y ,x]
    g_val = 0.68;
    run moveTo.m;
    trans = [0.0219 0.413 0.4]; % move z
    run moveTo.m;
    initialIKGuess(2).JointPosition = 0;
    [trans,rot,g_val] = moveTobluebin(); % drop zone
    run moveTo.m;
end


% Region_1:
[trans, rot, g_val] = generate_vars([0.46 -0.07 0.44], [pi/2 -pi 0], 0.01);
run moveTo.m

[trans, rot, g_val] = generate_vars([0.46 -0.07 0.24], [pi/2 -pi 0], 0.516);
run moveTo.m
pause(0.5);

[trans, rot, g_val] = moveTobluebin();
run moveTo.m

[trans, rot, g_val] = generate_vars([0.657 0.017 0.45], [pi/2 -pi 0], 0.01);
run moveTo.m

[trans, rot, g_val] = generate_vars([0.657 0.017 0.25], [pi/2 -pi 0], 0.223);
run moveTo.m
pause(0.5);

[trans, rot, g_val] = generate_vars([0.657 0.017 0.45], [pi/2 -pi 0], 0.223);
run moveTo.m

[trans, rot, g_val] = moveTogreenbin();
run moveTo.m

[trans, rot, g_val] = generate_vars([0.657 0.017 0.34], [pi/2 -pi 0], 0.01);
run moveTo.m

[trans, rot, g_val] = generate_vars([0.657 0.017 0.14], [pi/2 -pi 0], 0.223);
run moveTo.m
pause(0.5);

[trans, rot, g_val] = generate_vars([0.657 0.017 0.34], [pi/2 -pi 0], 0.223);
run moveTo.m

[trans, rot, g_val] = moveTogreenbin();
run moveTo.m

% stacked blocks (region 1) -> d_pouch
% first push the stacked cubes
%trans = [0.75 -0.09 0.1]; rot = [pi/2 -pi 0] ; g_val = 0.6;
%run moveTo.m


% function moveTobins:
function [trans, rot, g_val] = generate_vars(T, R, G_V)
    trans = T;
    rot = R;
    g_val = G_V;
end
function [trans, rot, g_val] = moveTobluebin()
    [trans, rot, g_val] = generate_vars([0.47 -0.37 0.45], [pi/2 -pi 0], 0.01);
end
function [trans, rot, g_val] = moveTogreenbin()
    [trans, rot, g_val] = generate_vars([-0.5 -0.4 0.48], [pi+90 pi 0], 0.01);
end

function visDetection(rgbImage,bboxes,labels,scores)
    annotatedImage = insertObjectAnnotation(im2uint8(rgbImage), 'Rectangle',...
        bboxes, string(labels)+":"+string(scores),'Color','r');
    figure, imshow(annotatedImage);
end


%{
% block_1 (purple)
[trans, rot, g_val] = generate_vars([0.691 -0.08 0.34], [pi/2 -pi 0], 0.01);
run moveTo.m
[trans, rot, g_val] = generate_vars([0.691 -0.08 0.14], [pi/2 -pi 0], 0.67);
run moveTo.m
pause(0.5);
[trans, rot, g_val] = generate_vars([0.691 -0.08 0.34], [pi/2 -pi 0], 0.67);
run moveTo.m
[trans, rot, g_val] = moveTogreenbin();
run moveTo.m
%}