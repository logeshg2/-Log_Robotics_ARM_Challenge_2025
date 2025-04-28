% Region_5:

[trans, rot, g_val] = generate_vars([-0.62 0.295 0.288], [0 0 -pi], 0.01);
run moveTo.m

[trans, rot, g_val] = generate_vars([-0.62 0.295 0.087], [0 0 -pi], 0.209);
run moveTo.m
pause(0.5);

[trans, rot, g_val] = generate_vars([-0.62 0.295 0.288], [0 0 -pi], 0.209);
run moveTo.m

[trans, rot, g_val] = moveTobluebin();
run moveTo.m

[trans, rot, g_val] = generate_vars([-0.50 0.265 0.34], [pi/2 0 -pi], 0.01);
run moveTo.m

[trans, rot, g_val] = generate_vars([-0.50 0.265 0.14], [pi/2 0 -pi], 0.228);
run moveTo.m
pause(0.5);

[trans, rot, g_val] = generate_vars([-0.50 0.265 0.34], [pi/2 0 -pi], 0.228);
run moveTo.m

[trans, rot, g_val] = moveTogreenbin();
run moveTo.m

%%%
% this part contatins 1 spam can
trans = [-0.50 0.27 0.4];
rot = [pi -pi 0];
g_val = 0.01;
run moveTo.m
det_mark_s_can = false;
run perception.m;
numObjects = height(objPose);
for i = 1:numObjects % pnp loop in this region

    initialIKGuess(2).JointPosition = 0.4; % modifying initial guess for pnp
    
    curPose = objPose(i,:);% curPose -> [x, y, z, theta_z, label, width]
    trans = [curPose(1) curPose(2) 0.4];
    rot(1) = pi - deg2rad(curPose(4)); % converting to radian 
    if (curPose(5) == 2 && curPose(3) > 0.09)  % for spam can
        rot(1) = -rot(1);
    end
    g_val = 0.01;
    run moveTo.m; % move(x,y,Theta)
    
    g_val = findG_val(curPose(6),curPose(5)); % g_val is computed (find g_val)
    
    if (curPose(5) == 2 && curPose(3) > 0.09)  % for spam can
        trans(3) = 0.1;
        g_val = 0.32;
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
end
%%%


[trans, rot, g_val] = generate_vars([-0.36 -0.01 0.36], [pi/2 0 -pi], 0.01);
run moveTo.m

[trans, rot, g_val] = generate_vars([-0.36 -0.01 0.25], [pi/2 0 -pi], 0.223);
run moveTo.m
pause(0.5);

[trans, rot, g_val] = generate_vars([-0.36 -0.01 0.36], [pi/2 0 -pi], 0.223);
run moveTo.m

[trans, rot, g_val] = moveTogreenbin();
run moveTo.m

initialguess(1).JointPosition = 1.5; % to move in a good manner(IMP)
[trans, rot, g_val] = generate_vars([-0.525 -0.06 0.4], [0 0 -pi], 0.01);
run moveTo.m

[trans, rot, g_val] = generate_vars([-0.52 -0.06 0.087], [0 0 -pi], 0.211);
run moveTo.m
pause(0.5);

[trans, rot, g_val] = generate_vars([-0.52 -0.06 0.4], [0 0 -pi], 0.211);
run moveTo.m

initialIKGuess(1).JointPosition = 0; % back to normal initialguess(IMP)
[trans, rot, g_val] = moveTobluebin();
run moveTo.m

% marker 
initialguess(1).JointPosition = 1.5; % to move in a good manner(IMP)
[trans, rot, g_val] = generate_vars([-0.67 -0.02 0.34], [0-pi/3 0 -pi], 0.01);
run moveTo.m

[trans, rot, g_val] = generate_vars([-0.67 -0.02 0.07], [0-pi/3 0 -pi], 0.68);
run moveTo.m
pause(0.5);

[trans, rot, g_val] = generate_vars([-0.67 -0.02 0.34], [0-pi/3 0 -pi], 0.68);
run moveTo.m

initialIKGuess(1).JointPosition = 0; % back to normal initialguess(IMP)
[trans, rot, g_val] = moveTobluebin();
run moveTo.m
%

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


%{
% 2025 old

% fixed spam can
[trans, rot, g_val] = generate_vars([-0.48 0.418 0.34], [pi/2+pi/4 0 -pi], 0.01);
run moveTo.m

[trans, rot, g_val] = generate_vars([-0.48 0.418 0.1], [pi/2+pi/4 0 -pi], 0.32);
run moveTo.m
pause(0.5);

[trans, rot, g_val] = generate_vars([-0.48 0.418 0.34], [pi/2+pi/4 0 -pi], 0.32);
run moveTo.m

[trans, rot, g_val] = moveTogreenbin();
run moveTo.m
%
%}