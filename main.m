% Main script that revoks other required function and scripts.
% Run this file to perform pick and place task.
%clear;
clc;

% Cans & Spam -> Green bin
% Bottle & Markers -> Blue bin
% Green & Purple cubes -> Green bin
% Blue & Red cubes -> Blutrae bin

% adding required files path
addpath("helper_scripts/")
addpath("mat_files/")
addpath("pnp_scripts/")
%addpath("pointclouds/")

% Robot connection -> using ROS-IP with the gazebo simulator
% IP Address of robot
rosIP = "192.168.1.2"; % "10.70.250.237"; "192.168.56.101"
rosshutdown; % shut down existing connection to ROS
rosinit(rosIP,11311);
disp('ROS-init over');

% gipper rosactionclient 
[gripAct,gripGoal] = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory');
gripAct.FeedbackFcn = [];
gripAct.ResultFcn = [];

% robot trajectory control rosactionclient
[trajAct,trajGoal] = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory'); 
trajAct.FeedbackFcn = []; 
trajAct.ResultFcn = []; 

% UR5e robot model declaration
ur_robot = loadrobot('universalUR5e');%, 'DataFormat','row');

% Adjusting the robot's position as given in the gazebo simulator (Competition start position)
tform=ur_robot.Bodies{3}.Joint.JointToParentTransform;
ur_robot.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));
tform=ur_robot.Bodies{4}.Joint.JointToParentTransform;
ur_robot.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));
tform=ur_robot.Bodies{7}.Joint.JointToParentTransform;
ur_robot.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));
% after the above change in transform -> initialguess remains the same

% set joint limits for proper movement and to avoid collision
% ur_robot.Bodies{4}.Joint.PositionLimits = [deg2rad(0) deg2rad(180)];

% initialize parameters
disp("Initializing Paramters")
run initialize.m;  
disp("Initialization Completed")
% TODO: for initialize.m
% >> use getTranform() to get the tf from base to camera for pnp

% Inverse kinematics solver
ik_solver = inverseKinematics("RigidBodyTree",ur_robot);
ik_solver.SolverParameters.AllowRandomRestart = false;
ik_weights = [0.25 0.25 0.25 0.1 0.1 0.1];   % configuration weights for IK solver [Translation Orientation]
initialguess = ur_robot.homeConfiguration; % contains (JointName ; JointPosition)
% the ik_weights above were given in the startup code

initialguess(3).JointPosition = 0.3; %initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialguess,trajGoal);
sendGoal(trajAct,trajGoal);
% set gripper to initial position
gripPos = 0.01;   % 0.0 is open, ~0.4 is close
goalMsg = packGripGoal(gripPos, gripGoal);
sendGoal(gripAct,goalMsg);

% pick and place of fixed objects
disp(newline + "Pick and place on fixed objects")
run pnp_fix_obj.m
% need to update color cubes position and grip value -> fixed object area

% autonomous pick and place (pnp)
disp(newline + "Autonomous Pick and Place");

initialguess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialguess,trajGoal);
sendGoalAndWait(trajAct,trajGoal);

% Position_3:
disp(newline + "Pick and place on Region 3");
run pnp_region_3.m;
close all;

% Position 4:
disp(newline + "Pick and place on Region 4");
% first pnp no cubes
pnp_cubes = false;
run pnp_region_4.m;
close all;
% first pnp no cubes
pnp_cubes = true;
run pnp_region_4.m;
close all;
pnp_cubes = false;

% Position 2:
disp(newline + "Pick and place on Region 2");
% pick bottles and cans
det_mark_s_can = false;
run pnp_region_2.m;
close all;
% pick markers
det_mark_s_can = true;
run pnp_region_2.m;
close all;
det_mark_s_can = false;

% region 5
disp(newline + "Pick and place on fixed objects")
run pnp_fix_r_5.m

% shutting down ros connection:
disp(newline + "Pick and Place Complete");
rosshutdown;

% NOTE:
% check all the pnp_region_X.m files !!!
% for any clarification check original main.m file