% Initializing Parameters:
 
% Camera msg setup:
rgbsub = rossubscriber("/camera/rgb/image_raw","sensor_msgs/Image","DataFormat","struct"); % rgb camera
%pause(2);
depthsub = rossubscriber('/camera/depth/image_raw',"sensor_msgs/Image","DataFormat","struct"); % depth camera
%pause(2);
ptsub = rossubscriber('/camera/depth/points','DataFormat','struct'); % pointcloud msg
%pause(2);
ftSub = rossubscriber('/ft_sensor/raw', 'geometry_msgs/WrenchStamped'); % force feed back

% add camera rigidbody to the ur_robot
cam = rigidBody("camera");
cam_tf = trvec2tform([0, -0.055 0]) * eul2tform([0, 0, 0], 'ZYX');   % [0, -pi/2, pi/2]
jnt = rigidBodyJoint('jnt','fixed');
setFixedTransform(jnt,cam_tf)
cam.Joint = jnt;
% add body to ur_robot rigidbody tree
addBody(ur_robot,cam,"tool0");

Tg_c = load("Tg_c.mat").Tg_c;


%{
% add gripper rigidbody to the ur_robot [133.7 mm - 147.2 mm]
% value of z-offset (tool frame for gripper) -> so calculate Ik from gripper tool (not "tool0")
grip = rigidBody("gripper");
grip_tf = trvec2tform([0, 0, 0.1337]) * eul2tform([0, 0, 0], 'XYZ');
jnt1 = rigidBodyJoint('jnt1','fixed');
setFixedTransform(jnt1,grip_tf)
grip.Joint = jnt1;
% add body to ur_robot rigidbody tree
addBody(ur_robot, grip, "tool0");
%}


% Perception parameters:
showImg = true;
det_mark_s_can = false;             % turn `on` when marker or spam can must be detected
pnp_cubes = false;                  % turn `on` when you need to pick color blocks
yoloScoreTh = 0.7;                  % YOLO score threshold
gridDownsample = 0.001;             % Measured point downsample grid size
PlanrThickness = 0.01;
normalvector = [0 0 1];
maxPlaneTilt = 10;                   % in degrees
% yolov4 model (objecte detection model)
detector_1 = load("best2.mat").detector;           % yolo model gets loaded
detector_2 = load("marker_can.mat").detector;
classes_1 = ["bottle", "can", "d_pouch"];          % "marker", "spam_can", "color_blocks"];
classes_2 = ["marker", "spam_can"];
% camera intrinsics
K = [[554.38 0 320.5] ; [0 554.38 240.5] ; [0 0 1]];
% or
fx = 554.38 / 1000;
fy = 554.38 / 1000;
cx = 320.5; 
cy = 240.5;


% Position for reference:
% x,y,z,Rx,Ry,Rz values for different regions:
Position_1 = [0.5,0.0,0.45,pi/2,-pi,0];      % fixed objects
Position_2 = [0.45,0.45,0.45,pi/2,-pi,0];    % position and orientation change ( difficult to pnp)
Position_3 = [-0.033,0.225,0.47,pi,-pi,0];   % only items change
Position_4 = [0.0,0.68,0.45,pi,-pi,0];       % same position but orientation change
Position_5 = [-0.4,0.16,0.48,pi/2,0,-pi];    % fixed objects
blueBin = [0.47,-0.37,0.45,pi/2,-pi,0];
redBin = [-0.5,-0.4,0.48,pi+90,pi,0];


% initial camera check:
% This will produce error early when there is no camera feed
% If error comes re-run the main script
curImage = receive(rgbsub,5);
rgbImage = rosReadImage(rgbsub.LatestMessage);

% depth image:
curDepth = receive(depthsub,5);
depthImage = rosReadImage(depthsub.LatestMessage);

% Point Cloud:
sub = receive(ptsub, 10);
latmsg = ptsub.LatestMessage;
ptCloud = rosReadXYZ(sub,"PreserveStructureOnRead",true);
ptCloud = pointCloud(ptCloud);

%{
% load objects pointcloud
bottle_pc = pcread("bottle.pcd");
can_pc = pcread("can.pcd");
cube_pc = pcread("cube.pcd");
marker_pc = pcread("marker.pcd");
spam_can_pc = pcread("spam_can.pcd");
pc_list_1 = [bottle_pc, can_pc, cube_pc];
pc_list_2 = [marker_pc, spam_can_pc];
%}