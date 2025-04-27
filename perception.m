% perception.m contains script to detect objects in image and find their
% pose and xyz coordinates with respect to the robot base.
% From this "theta" and "xyz" of the objects are calculated as objPose:
% This script computes the width of the object for gripper value.

disp("Running perception")

% Base to Gripper transformation: (for each region) (V.IMP)
Tb_g = tform; % (using IK)

% Base to Camera transformation:
Tb_c = Tb_g * Tg_c;

% Reading input from ROS:
% rgb image:
curImage = receive(rgbsub,5);
rgbImage = rosReadImage(rgbsub.LatestMessage);
% depth image:
curDepth = receive(depthsub,5);
depthImage = rosReadImage(depthsub.LatestMessage);
% Point Cloud:
sub = receive(ptsub);
latmsg = ptsub.LatestMessage;
ptCloud = rosReadXYZ(sub,"PreserveStructureOnRead",true);
ptCloud = pointCloud(ptCloud);

% Object Detection:
%trainedYoloNet = pretrained.detector; %YOLOv4 network
if det_mark_s_can
    [bboxes,scores,labels] = detector_2.detect(rgbImage); 
else
    [bboxes,scores,labels] = detectObjectsYoloNet(detector_1, rgbImage); % detection
end

if (showImg) % visualize image detection:
    annotatedImage = insertObjectAnnotation(im2uint8(rgbImage), 'Rectangle',...
        bboxes, string(labels)+":"+string(scores),'Color','r');
    figure, imshow(annotatedImage);
end

% filter detected objects:
if (pnp_cubes == false && det_mark_s_can == false)
    valid_scores = scores >= yoloScoreTh;
    valid_labels = labels ~= "d_pouch";
    valid_idx = valid_labels == valid_scores;
elseif (det_mark_s_can)      % do not detect spam cans
    valid_scores = scores >= 0.5;
    valid_labels = labels ~= "spam_can";
    valid_idx = valid_labels == valid_scores;
else
    valid_idx = scores >= yoloScoreTh;
end
bboxes = bboxes(valid_idx, :);
scores = scores(valid_idx);
labels = labels(valid_idx);
numObjects = size(bboxes,1);

% Detect plane and remove plane from Point cloud
[param, planeIdx, nonPlaneIdx] = pcfitplane(ptCloud, PlanrThickness, normalvector, maxPlaneTilt);
plane = select(ptCloud, planeIdx);
nonPlane = select(ptCloud, nonPlaneIdx);

%if(showImg) % Visualize pointcloud:
    %figure,pcshow(nonPlane,'ViewPlane','XY');axis on;hold on;
    %view([0 -90]);
    %hold off;
%end

% Separating nonPlaneMask (getting only non plane pointclouds)
[m,n,~] = size(rgbImage);
nonPlaneMask = zeros(m,n);
nonPlaneMask =nonPlaneMask(:);
nonPlaneMask(nonPlaneIdx)= 1;


% Estimate object poses
[xyz,theta,ptCloud_vec,ptCloudParameterVector] = findObjectPoses(ptCloud,rgbImage, bboxes,gridDownsample, nonPlaneMask);
if(showImg)
    figure;
    for idx = 1: numObjects
        U = ptCloudParameterVector{idx}.UVW(:,1);
        V = ptCloudParameterVector{idx}.UVW(:,2);
        W = ptCloudParameterVector{idx}.UVW(:,3);
        center = ptCloudParameterVector{idx}.centroid;
        nexttile;
        pcshow(ptCloud_vec{idx},'ViewPlane','XY');
        hold on;
        quiver3(center(1), center(2), center(3), U(1), V(1), W(1), 'r');
        quiver3(center(1), center(2), center(3), U(2), V(2), W(2), 'g');
        quiver3(center(1), center(2), center(3), U(3), V(3), W(3), 'b');
        hold off;
    end
end

% Compute object orientation with respect to robot coordinate system using the rotation matrix.
thetaNew = zeros(numObjects,1);
for idx = 1:numObjects
    U = ptCloudParameterVector{idx}.UVW(:,1);
    V = ptCloudParameterVector{idx}.UVW(:,2);
    W = ptCloudParameterVector{idx}.UVW(:,3);
    majorAxis = [U(1), V(1), W(1)];
    majorAxis = (Tb_c(1:3,1:3)*majorAxis')';
    %This calculates the angle between the positive y axis ([ 0 1 0]) and the major axis of the object in an anti-clockwise direction
    thetaNew(idx) = atan2d(dot([0 0 1],cross([  0 1 0],majorAxis)),dot([ 0 1 0],majorAxis));
    if (thetaNew(idx)<0)
        thetaNew(idx) = 180 + thetaNew(idx);
    end
end

% compute final object pose (objPose):
objPose = zeros(numObjects,6);     % objPose -> [x,y,z,theta,label,width]
for i=1:length(ptCloudParameterVector)
    objPose(i,1:3) = Tb_c(1:3,1:3)*xyz(i,:)' + Tb_c(1:3,4);
    objPose(i,4) = thetaNew(i);
end
% adding label to objPose:
objPose(1:numObjects,5) = labels; % labels ->(bottle-1;can-2;pouches-3)

% compute object width for gripper grasp value:
for i=1:length(ptCloudParameterVector)
    th = objPose(i,4);

    % change the theta of pouches: (WIP)
    % if (objPose(i,5) == 3)
    %     if (th > 10 && th < 60)
    %         th = th - 30; % this 30 is given by me for getting approximate angle (might be wrong)
    %         objPose(i,4) = th;
    %     end
    % end

    % transform the pointcloud where the object is parallel to xy axis
    rotationAngles = [0 0 th]; translation = [0 0 0];
    tf = rigidtform3d(rotationAngles,translation);
    ptCloudOut = pctransform(ptCloud_vec{i},tf);
    ydiff = abs(ptCloudOut.YLimits(1) - ptCloudOut.YLimits(2));
    xdiff = abs(ptCloudOut.XLimits(1) - ptCloudOut.XLimits(2));
    objPose(i,6) = min(xdiff, ydiff); % width of object

    % This is only for pouches, finding correct theta for pouches: (IMP)
    if (objPose(i,5) == 3)
        for ang = 0:90
            rotationAngles = [0 0 ang]; translation = [0 0 0];
            tf = rigidtform3d(rotationAngles,translation);
            ptCloudOut = pctransform(ptCloud_vec{i},tf);
            ydiff = abs(ptCloudOut.YLimits(1) - ptCloudOut.YLimits(2));
            xdiff = abs(ptCloudOut.XLimits(1) - ptCloudOut.XLimits(2));
            m = min(xdiff,ydiff); % width of object
            if (m <= 0.0302)
                objPose(i,4) = ang;  % changing angle (pi -> is current gripper position)
                objPose(i,6) = m;
                break;
            end
        end
    end

    % no visualizing script (ToDo)
    % nexttile;
    % pcshow(ptCloudOut,'ViewPlane','XY');
end 