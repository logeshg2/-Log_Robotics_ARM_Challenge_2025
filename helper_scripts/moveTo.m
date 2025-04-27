% This script is "moveTo"

% NOTE:
% trans = [x y z]; % [X Y Z] in meter
% rot = [zrad yrad xrad]; %  [Z Y X] in radians}

% first compute the transforms tf
tform = eul2tform(rot); 
tform(1:3,4) = trans';       % set translation in homogeneous transform

% compute Ik from base to the target tform
[configSoln, ~] = ik_solver('tool0', tform, ik_weights, initialguess);

% compute the trajectory goals and send to ur_robot
trajGoal = packTrajGoal(configSoln, trajGoal);
sendGoalAndWait(trajAct,trajGoal);              % for movement

gripGoal = packGripGoal(g_val, gripGoal);
sendGoalAndWait(gripAct, gripGoal);  % for gripper

% for adaptive gripping
%{
if g_val <= 0.1
    gripGoal = packGripGoal(g_val, gripGoal);
    sendGoalAndWait(gripAct, gripGoal);  % for gripper
else
    msg = receive(ftSub, 1);
    ftmsg = ftSub.LatestMessage;
    start = 0.1;
    while ftmsg.Wrench.Force.Z < 12.5
        gripGoal = packGripGoal(start, gripGoal);
        sendGoalAndWait(gripAct, gripGoal);  % for gripper
        
        ftmsg = ftSub.LatestMessage;
        disp(start);
        
        start = start + 0.04;
        if start > g_val
            disp("Adaptive gripping over");
            break;
        end
    end
end
%}