% perception.m contains script to detect objects in image and find their
% pose and xyz coordinates with respect to the robot base.
% From this "theta" and "xyz" of the objects are calculated as objPose:
% This script computes the width of the object for gripper value.

disp("Running perception")

% Base to Gripper transformation: (for each region) (V.IMP)
Tb_g = tform; % (using IK)

% Base to Camera transformation:
%Tg_c = getTransform(ur_robot, ur_robot.homeConfiguration, "camera", "tool0");
Tg_c = load("Tg_c.mat").Tg_c;
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
if (det_mark_s_can)                         % choose detector
    yolodetector = detector_2;
else
    yolodetector = detector_1;
end
[bboxes,scores,labels] = detect(yolodetector, rgbImage);

if (showImg) % visualize image detection:
    annotatedImage = insertObjectAnnotation(im2uint8(rgbImage), 'Rectangle',...
        bboxes, string(labels)+":"+string(scores),'Color','r');
    figure, imshow(annotatedImage);
end

% filter detected objects:
if (pnp_cubes == false)
    valid_scores = scores >= yoloScoreTh;
    valid_labels = labels ~= "d_pouch";
    valid_idx = valid_labels == valid_scores;
else
    valid_idx = scores >= yoloScoreTh;
end
bboxes = bboxes(valid_idx, :);
scores = scores(valid_idx);
labels = labels(valid_idx);
numObjects = size(bboxes,1);


%%%%
% trying different approach this year 2025

% loop over all the objects and perform pose estimation
% Step 1: crop the point-cloud for each object using bbox
% Step 2: remove the plane from the croped point -cloud
% Step 3: ICP to find the `tf` from camera to object (using GT pointclouds)
% Step 4: transform multiplication for Tb_o (IMP: Tb_o = Tb_c * Tc_o)
% Step 5: store transform for each object detected in an array
% Step 6: update `trans` and `rot(1)` only z-rot for pnp
% NOTE:
%   - use z-height of point-cloud and label to find the grasp and acccurate
%   depth for pnp

% removing plane from original point cloud 
[~, ~, nonPlaneIdx] = pcfitplane(ptCloud, PlanrThickness, normalvector, maxPlaneTilt);
nonPlaneMask = zeros(480,640);
nonPlaneMask(nonPlaneIdx)= 1;
xyz = ptCloud.Location;
xyz(~nonPlaneMask) = NaN;
ptCloud = pointCloud(xyz);
%pcshow(ptCloud);

objPose = zeros(numObjects, 5);     % objPose -> [x, y, z, theta, label]
m = ceil(sqrt(numObjects));
figure;
for idx = 1:numObjects
    % crop an objects pointcloud for finding pose
    box = double(round(bboxes(idx,:)));
    x_start = box(1);
    y_start = box(2);
    x_end = x_start + box(3);
    y_end = y_start + box(4);
    % cx, cy
    cx = x_start + round(box(3) / 2);
    cy = y_start + round(box(4) / 2);

    % crop
    point = ptCloud.Location(y_start:y_end, x_start:x_end, :);
    pc = pointCloud(point);
    %pcshow(pc);

    % get label idx in classes list
    label = string(labels(idx));
    if (det_mark_s_can)
        obj_idx = find(classes_2 == label);
    else
        obj_idx = find(classes_1 == label);
    end
    % get pcd of the selected label
    if (det_mark_s_can)
        obj_pc = pc_list_2(obj_idx);
    else
        obj_pc = pc_list_1(obj_idx);
    end
    
    % use ICP to find pose [IMP] -> pose estimation
    pc_tform = pcregistericp(pc, obj_pc);
    % Tc_o
    Tc_o = invert(pc_tform).A;
    
    % update z-distance
    z_dist = depthImage(cy, cx);
    Tc_o(3, 4) = z_dist;
    
    % Transform to robot frame (Tb_o)
    Tb_o = Tb_c * Tc_o;

    % append values
    objPose(idx, 1) = Tb_o(1, 4);       % x
    objPose(idx, 2) = Tb_o(2, 4);       % y
    objPose(idx, 3) = Tb_o(3, 4);       % z
    eul_rot = rotm2eul(Tb_o(1:3, 1:3)); % [z, y, x]
    objPose(idx, 4) = eul_rot(1);       % theta (z-rot)
    objPose(idx, 5) = obj_idx;          % label index -> can be accessed using `classes` list
    
    % plot the trasformed pc
    t_pc = pctransform(obj_pc, invert(pc_tform));
    subplot(m, m, idx);
    pcshowpair(pc, t_pc);

end


%%%%

% old code
%{
% Detect plane and remove plane from Point cloud
[param, planeIdx, nonPlaneIdx] = pcfitplane(ptCloud, PlanrThickness, normalvector, maxPlaneTilt);
plane = select(ptCloud, planeIdx);
nonPlane = select(ptCloud, nonPlaneIdx);

if(showImg) % Visualize pointcloud:
    %figure,pcshow(nonPlane,'ViewPlane','XY');axis on;hold on;
    %view([0 -90]);
    %hold off;
end

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
    objPose(i,6) = min(xdiff,ydiff); % width of object

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
%}


% convert from 2D coordinate(image plane) to 3D camera coordinate
%{ 
% z from depth image
Z_start = double(depthImage(x_start, y_start));
Z_end = double(depthImage(x_end, y_end));

% inverse projection
X_s = round((x_start - cx) * Z_start / fx);
Y_s = round((y_start - cy) * Z_start / fy);
X_s = double(X_s / 1);
Y_s = double(Y_s / 1);
X_e = round((x_end - cx) * Z_start / fx);
Y_e = round((y_end - cy) * Z_start / fy);
X_e = double(X_e / 1);
Y_e = double(Y_e / 1);
%}