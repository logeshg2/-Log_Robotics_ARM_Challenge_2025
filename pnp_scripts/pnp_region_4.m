% move to obj 1 and perform pnp:
% trans = [0.16 0.779 0.30];
% rot = [pi -pi 0]; %[z ,y ,x]
% g_val = 0.01;
% run moveTo.m;


trans = [0.0 0.68 0.5];
rot = [pi -pi 0]; %[z ,y ,x]
g_val = 0.01;
run moveTo.m;
run perception.m;
numObjects = height(objPose);
for i = 1:numObjects % pnp loop in this region

    initialIKGuess(2).JointPosition = 0.4; % modifying initial guess for pnp
    
    curPose = objPose(i,:);% curPose -> [x, y, z, theta_z, label, width]
    trans = [curPose(1) curPose(2) 0.4];
    rot(1) = pi - deg2rad(curPose(4)); % converting to radian 
    if (curPose(5) == 3) % modify rot variable for pouches
        rot(1) = -rot(1);
    elseif (curPose(5) == 2 && curPose(3) > 0.09)  % for spam can
        rot(1) = -rot(1);
    end
    g_val = 0.01;
    run moveTo.m; % move(x,y,Theta)
    
    g_val = findG_val(curPose(6),curPose(5)); % g_val is computed (find g_val)
    if (g_val == 0.516)
        % manually find (x,y) of bottle
        if (curPose(1) <= 0.2 && curPose(1) >= 0.1)
            if (curPose(2) <= 0.82 && curPose(2) >= 0.71)
                trans = [0.16 0.779 0.30];
                %rot(1) = pi;
                g_val = 0.01;
            end
            run moveTo.m;
        end
        g_val = 0.516;
        trans(3) = 0.24; % standing bottle
    elseif (curPose(5) == 3) % pouches
        trans(3) = 0.066;
    else
        trans(3) = curPose(3);
    end
    if (g_val == 0.209) % increase grip value of lying bottle
        g_val = 0.211;
    end
    if (g_val == 0.230 && curPose(3) > 0.1) % adjusting g_val
        g_val = 0.228;
    elseif (curPose(5) == 2 && curPose(3) > 0.09)  % for spam can
        trans(3) = 0.1;
        g_val = 0.32;
    end

    run moveTo.m; % move(z) and grab;
    

    % proper gripping of pouches (release gripper and grasp again)
    if (curPose(5) == 3) % only for pouches (IMP)
        g_val = 0.45;
        run moveTo.m;
        g_val = 0.54;
        run moveTo.m;
    end

    trans(3) = 0.4;
    run moveTo.m; % move(z=0.4)
    
    initialIKGuess(2).JointPosition = 0; % change initialIkguess to default
    
    % Task for pouches: (IMP)
    % place the pouch on the weight machine and pnp: 
    % if (curPose(5) == 3)
    %     run execute_pouch_task.m
    % end

    if (curPose(5) == 1) % moveTobin
        [trans,rot,g_val] = moveTobluebin();
        run moveTo.m;
    elseif (curPose(5) == 2 || curPose(5) == 3)
        [trans,rot,g_val] = moveTogreenbin();
        run moveTo.m;
    end
    % trans = [0.0 0.68 0.45];
    % rot = [pi -pi 0];
    % g_val = 0.01;
    % run moveTo.m;   % the robot moves to region
end

% Helper Functions:
function [trans,rot,g_val] = moveTobluebin()
    % default value of g_val in drop zone is 0.01
    trans = [0.47 -0.37 0.45]; rot = [pi/2 -pi 0]; g_val = 0.01; 
end
function [trans,rot,g_val] = moveTogreenbin()
    trans = [-0.5 -0.4 0.48]; rot = [pi+90 pi 0]; g_val = 0.01;
end