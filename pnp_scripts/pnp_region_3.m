% This script -> pick and place objects in region 3
% trans = [-0.033 0.225 0.5];
% rot = [pi -pi 0]; %[z ,y ,x]
% g_val = 0.01;
% run moveTo.m;   % the robot moves to region
% at this point the "tform" is used for "Tb_c" calculation(IMP)
% run perception script: (we get objPose's in the particular region)
%run perception.m; % return objPose


% Obj 1,2,3,4 -> will be either bottle or can [IMP]
% Obj 1 (up right)
trans = [-0.228 0.174 0.30];
rot = [pi -pi 0]; %[z ,y ,x]
g_val = 0.01;
run moveTo.m;
rgbImage = rosReadImage(rgbsub.LatestMessage);
[bboxes,scores,labels] = detect(detector_1, rgbImage, Threshold = 0.7);
close all;
visDetection(rgbImage,bboxes,labels,scores); % visualize detection
if (find(labels=="bottle"))
    initialIKGuess(2).JointPosition = -0.3;
    trans = [-0.228 0.174 0.24];
    rot = [pi -pi 0]; %[z ,y ,x]
    g_val = 0.516;
    run moveTo.m;
    trans = [-0.228 0.174 0.4]; % move z
    run moveTo.m;
    initialIKGuess(2).JointPosition = 0;
    [trans,rot,g_val] = moveTobluebin(); % drop zone
    run moveTo.m;
elseif (find(labels=="can"))
    initialIKGuess(2).JointPosition = -0.3;
    trans = [-0.228 0.174 0.14];
    rot = [pi -pi 0]; %[z ,y ,x]
    g_val = 0.224;
    run moveTo.m;
    trans = [-0.228 0.174 0.4]; % move z
    run moveTo.m;
    initialIKGuess(2).JointPosition = 0;
    [trans,rot,g_val] = moveTogreenbin(); % drop zone
    run moveTo.m;
end

% Obj 2 (up right)
trans = [-0.150 0.406 0.30];
rot = [pi -pi 0]; %[z ,y ,x]
g_val = 0.115;
run moveTo.m;
rgbImage = rosReadImage(rgbsub.LatestMessage);
[bboxes,scores,labels] = detect(detector_1, rgbImage, Threshold = 0.7);
close all;
visDetection(rgbImage,bboxes,labels,scores); % visualize detection
if (find(labels=="bottle"))
    initialIKGuess(2).JointPosition = -0.3;
    trans = [-0.150 0.406 0.24];
    rot = [pi -pi 0]; %[z ,y ,x]
    g_val = 0.516;
    run moveTo.m;
    trans = [-0.150 0.406 0.4]; % move z
    run moveTo.m;
    initialIKGuess(2).JointPosition = 0;
    [trans,rot,g_val] = moveTobluebin(); % drop zone
    run moveTo.m;
elseif (find(labels=="can"))
    initialIKGuess(2).JointPosition = -0.3;
    trans = [-0.150 0.406 0.14];
    rot = [pi -pi 0]; %[z ,y ,x]
    g_val = 0.224;
    run moveTo.m;
    trans = [-0.150 0.406 0.4]; % move z
    run moveTo.m;
    initialIKGuess(2).JointPosition = 0;
    [trans,rot,g_val] = moveTogreenbin(); % drop zone
    run moveTo.m;
end

% Obj 3 and 4 (lying down) (old method)
trans = [-0.033 0.225 0.5];
rot = [pi -pi 0]; %[z ,y ,x]
g_val = 0.01;
run moveTo.m;   % the robot moves to region
close all;
run perception.m; % return objPose

% objPose -> [x ,y ,z, theta_z,label,width]
numObjects = height(objPose); % number of objects detected

%initialIKGuess(2).JointPosition = -0.3; % initialguess is modified in order for the ik to work better(IMP)

% move(x,y,theta) -> find g_val -> move(z) -> grab -> move(z=0.4) -> moveTobin -> drop -> repeat
for i = 1:numObjects
    initialIKGuess(2).JointPosition = -0.3;
    curPose = objPose(i,:);% curPose -> [x ,y ,z, theta_z,label,width]
    %[x,y,z,thz,labels] = curPose;
    trans = [curPose(1) curPose(2) 0.4];
    rot(1) = pi - deg2rad(curPose(4)); % converting to radian 
    g_val = 0.01;
    run moveTo.m; % move(x,y,Theta)
    initialIKGuess(2).JointPosition = 0;

    g_val = findG_val(curPose(6),curPose(5)); % g_val is computed (find g_val)
    if (g_val == 0.514)
        % manually find (x,y) of bottle
        trans(1) = (ptCloud_vec{i}.XLimits(1) + ptCloud_vec{i}.XLimits(2))/2;
        trans(2) = (ptCloud_vec{i}.YLimits(1) + ptCloud_vec{i}.YLimits(2))/2;
        trans = Tb_c(1:3,1:3)*trans(1,:)' + Tb_c(1:3,4); % camera frame to robot frame
        trans(3) = 0.24; %standing bottle
        %rot(1) = pi; % manually use pi as rot_z 
    else
        trans(3) = curPose(3);
    end
    if (curPose(5) == 2)  % g_val for lying can
        g_val = 0.228;
    end
    run moveTo.m; % move(z) and grab;

    initialIKGuess(2).JointPosition = -0.3;
    trans(3) = 0.4;
    run moveTo.m; % move(z=0.4)
    initialIKGuess(2).JointPosition = 0;

    if (curPose(5) == 1) % moveTobin
        [trans,rot,g_val] = moveTobluebin();
        run moveTo.m;
    elseif (curPose(5) == 2 || curPose(5) == 3)
        [trans,rot,g_val] = moveTogreenbin();
        run moveTo.m;
    end
    % trans = [-0.033 0.225 0.47];
    % rot = [pi -pi 0];
    % g_val = 0.01;
    % run moveTo.m;   % the robot moves to region
end

%initialIKGuess(2).JointPosition = 0; % initialguess is modified(IMP)
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


% Helper Functions:
function [trans,rot,g_val] = moveTobluebin()
    % default value of g_val in drop zone is 0.01
    trans = [0.47 -0.37 0.45]; rot = [pi/2 -pi 0]; g_val = 0.01; 
end
function [trans,rot,g_val] = moveTogreenbin()
    trans = [-0.5 -0.4 0.48]; rot = [pi+90 pi 0]; g_val = 0.01;
end

function visDetection(rgbImage,bboxes,labels,scores)
    annotatedImage = insertObjectAnnotation(im2uint8(rgbImage), 'Rectangle',...
        bboxes, string(labels)+":"+string(scores),'Color','r');
    figure, imshow(annotatedImage);
end