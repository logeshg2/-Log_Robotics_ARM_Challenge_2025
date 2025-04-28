% This script -> pick and place objects in region 3
trans = [0.4 0.45 0.5];
rot = [pi/2 -pi 0]; %[z ,y ,x]
g_val = 0.01;
run moveTo.m;   % the robot moves to region
% at this point the "tform" is used for "Tb_c" calculation(IMP)
% run perception script: (we get objPose's in the particular region)
run perception.m; % return objPose

% objPose -> [x ,y ,z, theta_z,label,width]
numObjects = height(objPose); % number of objects detected

% number of objects
if (det_mark_s_can)
    num_objects = 1;
else
    num_objects = 5;
end

% move(x,y,theta) -> find g_val -> move(z) -> grab -> move(z=0.4) -> moveTobin -> drop -> repeat
for i = 1:num_objects
    % if no objects then stop
    if (numObjects == 0)
        break;
    end

    initialIKGuess(2).JointPosition = 0.3; % modifying initial guess for pnp

    curPose = objPose(1,:);% curPose -> [x ,y ,z, theta_z,label,width] % taking only 1'st element in th detected objects
    
    % execute pnp of bottle at last
    if (curPose(5) == 1 && i ~= numObjects)
        try % check for index
            curPose = objPose(2,:); % take the second object from detection (IMP)
        catch
            % Do nothing
        end
    end

    %[x,y,z,thz,labels] = curPose;
    trans = [curPose(1) curPose(2) 0.4];
    rot(1) = deg2rad(curPose(4)); % converting to radian (-) is removed here (region 2)
    g_val = 0.01;
    run moveTo.m; % move(x,y,Theta)
    
    g_val = findG_val(curPose(6),curPose(5)); % g_val is computed (find g_val)
    if (g_val == 0.516)
        % manually find (x,y) of bottle
        trans(1) = (ptCloud_vec{i}.XLimits(1) + ptCloud_vec{i}.XLimits(2))/2;
        trans(2) = (ptCloud_vec{i}.YLimits(1) + ptCloud_vec{i}.YLimits(2))/2;
        trans = Tb_c(1:3,1:3)*trans(1,:)' + Tb_c(1:3,4); % camera frame to robot frame
        trans(3) = 0.24; %standing bottle
        %rot(1) = pi; % manually use pi as rot_z 
    else
        trans(3) = curPose(3);
    end
    if (g_val == 0.209)
        g_val = 0.211;
    end
    if (g_val == 0.230 && curPose(3) > 0.1) % adjusting g_val
        g_val = 0.227;
        rot(1) = pi/2;
    end
    % for marker
    if (g_val == 0.68)
        trans(3) = 0.086;    
    end

    run moveTo.m; % move(z) and grab;

    trans(3) = 0.4;
    run moveTo.m; % move(z=0.4)

    initialIKGuess(2).JointPosition = 0; % change initialIkguess to default

    if (curPose(5) == 1) % moveTobin
        [trans,rot,g_val] = moveTobluebin();
        run moveTo.m;
    elseif (curPose(5) == 2 || curPose(5) == 3)
        [trans,rot,g_val] = moveTogreenbin();
        run moveTo.m;
    end
    trans = [0.45 0.45 0.5];
    rot = [pi/2 -pi 0];
    g_val = 0.01;
    run moveTo.m;   % the robot moves to region

    close all;
    run perception.m; % perception
end


% Helper Functions:
function [trans,rot,g_val] = moveTobluebin()
    % default value of g_val in drop zone is 0.01
    trans = [0.47 -0.37 0.45]; rot = [pi/2 -pi 0]; g_val = 0.01; 
end
function [trans,rot,g_val] = moveTogreenbin()
    trans = [-0.5 -0.4 0.48]; rot = [pi+90 pi 0]; g_val = 0.01;
end