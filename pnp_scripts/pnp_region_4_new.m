% Pick and place region 4 (perception based pick and place)

if (det_mark_s_can)
    trans = [-0.15 0.72 0.4];
else
    trans = [0.0 0.68 0.5];
end
rot = [pi -pi 0]; % [z ,y ,x]
g_val = 0.01;
run moveTo.m;


% run perception to find pose -> updates objPose variable
run perception.m;
numObjects = height(objPose);
for i = 1:numObjects

    initialIKGuess(2).JointPosition = 0.4;  % modifying initial guess for pnp

    curPose = objPose(i,:);                 % curPose -> [x, y, z, theta_z, label_idx]
    if (det_mark_s_can)
        label = classes_2(curPose(5));
    else
        label = classes_1(curPose(5));
    end

    trans = [curPose(1) curPose(2) 0.4];
    rot(1) = pi - curPose(4);
    
    % modify rot variable for pouches
    if (label == "d_pouch")                   
        rot(1) = -rot(1);
    elseif (label == "spam_can")
        %rot(1) = -rot(1);
        rot(1) = rot(1) - pi/2;
    elseif (label == "bottle" && curPose(3) >= 0.12)   % standing bottle
        rot(1) = pi;
    end

    % move(x,y,Theta)
    g_val = 0.01;
    run moveTo.m;                           
    
    % below logic is for estimating grip_val and z_dist
    % compute the g_val based on curPose z-value and the label
    % NOTE: we could use point cloud to find the object from camera to find
    % it orientation (standing or lying)

    if (label == "bottle")
        if (curPose(3) >= 0.12)         % denotes standing bottle
            trans(3) = 0.24;
            trans(2) = trans(2) - 0.008;
            g_val = 0.516;
        else                            % denotes lying bottle
            trans(3) = 0.084;
            g_val = 0.211;
        end
    elseif (label == "can")
        if (curPose(3) >= 0.084)         % denotes standing can
            trans(3) = 0.14;
            g_val = 0.226;
        else                            % denotes lying can
            trans(3) = 0.083;
            g_val = 0.228;
        end
    elseif (label == "marker")          % marker allways in lying position
        trans(3) = 0.07;
        g_val = 0.68;
    elseif (label == "spam_can")
        if (curPose(3) > 0.065)           % only for upright position
            trans(2) = trans(2) + 0.02;
            trans(1) = trans(1) - 0.02;
            trans(3) = 0.1;
            g_val = 0.25;
            run moveTo.m
            g_val = 0.32;
        end
    elseif (label == "d_pouch")
        % fixed z distance
        trans(3) = 0.067;
        g_val = 0.54;
        run moveTo.m;
        g_val = 0.6;
        run moveTo.m
        trans(3) = 0.2;
        run moveTo.m
    end
    
    % move to pick 
    run moveTo.m;
    
    % back home
    trans = [0.0 0.68 0.4];
    run moveTo.m;

    % drop sequence
    if (label == "bottle" || label == "marker")
        [trans,rot,g_val] = moveTobluebin();
        run moveTo.m;
    elseif (label == "can" || label == "spam_can")
        [trans,rot,g_val] = moveTogreenbin();
        run moveTo.m;
    end
    
    % for color cubes
    % color sbould be identified first
    % place in scale and identify the color


end


% Helper Functions:
function [trans,rot,g_val] = moveTobluebin()
    trans = [0.47 -0.37 0.45]; rot = [pi/2 -pi 0]; g_val = 0.01; 
end
function [trans,rot,g_val] = moveTogreenbin()
    trans = [-0.5 -0.4 0.48]; rot = [pi+90 pi 0]; g_val = 0.01;
end


%{
%%%% old method
    g_val = findG_val(curPose(6),curPose(5)); % g_val is computed (find g_val)
    if (g_val == 0.514)
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
        trans(3) = 0.24; %standing bottle
    elseif (curPose(5) == 3) % pouches
        trans(3) = 0.066;
    else
        trans(3) = curPose(3);
    end
    if (g_val == 0.209) % increase grip value of lying bottle
        g_val = 0.211;
    end
    if (g_val == 0.230 && curPose(3) > 0.1) % adjusting g_val
        g_val = 0.227;
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
    %%%%
%}