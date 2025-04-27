% this script contains the experiments that I done on the camera transform from 'tool0' to 'camera_joint'
% transform for gripper as well
% xyz and rpy were from:
% "file:///home/robot/src/arm_gazebo/launch/urdf/ur5e_2f85.xacro"
% "file:///home/robot/src/arm_gazebo/launch/urdf/ur5e_2f85_macro.xacro"

% parent link is: 'tool0' and child link is: 'camera_joint'
% <origin xyz="0 -0.055 0" rpy="${pi/2.0} -${pi/2.0} 0"/>

ur_robot = loadrobot('universalUR5e');
tform=ur_robot.Bodies{3}.Joint.JointToParentTransform;
ur_robot.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));
tform=ur_robot.Bodies{4}.Joint.JointToParentTransform;
ur_robot.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));
tform=ur_robot.Bodies{7}.Joint.JointToParentTransform;
ur_robot.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

% camera rigidbody
cam = rigidBody("camera");
cam_tf = trvec2tform([0, -0.055 0]) * eul2tform([0, -pi/2 ,pi/2], 'ZYX');
jnt = rigidBodyJoint('jnt','fixed');
setFixedTransform(jnt,cam_tf)
cam.Joint = jnt;
% add body to ur_robot rigidbody tree
addBody(ur_robot,cam,"tool0");

home_cfg = ur_robot.homeConfiguration;

% check the pose of camera by ploting the ur_robot
ur_robot.show(home_cfg);

% disp using getTransform
% computes the transform that converts points from the source body frame 
% to the target body frame, using the specified robot configuration.
cam_2_tool0_tf = getTransform(ur_robot, home_cfg, "camera", "tool0");
cam_2_baselink_tf = getTransform(ur_robot, home_cfg, "camera", "base_link");
disp("Camera to tool0:");
disp(cam_2_tool0_tf); 
disp("Camera to base_link:");
disp(cam_2_baselink_tf);


% gipper part (add links)
% refer this also: https://in.mathworks.com/help/robotics/ref/rigidbodytree.addsubtree.html
% https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190503.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/6.%20Specifications.htm
% gripper as tool -> can use different z-distance from tool0 but =>> it depends on the object to pick. [IMP]

% so value (z - distance from tool0 to gripper tip [where where grasp takes place])
% 133.7 mm - 147.2 mm
% take between this range

% example
% gripper rigidbody
grip = rigidBody("gripper");
grip_tf = trvec2tform([0, 0, 0.1337]) * eul2tform([0, 0, 0], 'XYZ');
jnt1 = rigidBodyJoint('jnt1','fixed');
setFixedTransform(jnt1,grip_tf)
grip.Joint = jnt1;
% add body to ur_robot rigidbody tree
addBody(ur_robot, grip, "tool0");

home_cfg = ur_robot.homeConfiguration;

% check the pose of camera by ploting the ur_robot
ur_robot.show(home_cfg);

%%
% movement script
trans = [0 0.35 0.40];
rot = [pi -pi 0]; %[z ,y ,x]
g_val = 0.115;
run moveTo.m;
%%
% image acquisition for yolov4 training
z_rots = [pi/4 pi/2 pi -pi/2 -pi/4];
start = 10;
for i = 1:length(z_rots)
    rot(1) = z_rots(i);
    run moveTo.m
    rgbImage = rosReadImage(rgbsub.LatestMessage);
    filename = "./datasets/img" + start + ".png";
    imwrite(rgbImage, filename)
    start = start + 1;
end
%%
% simple image write part
idx = 42;
rgbImage = rosReadImage(rgbsub.LatestMessage);
filename = "./datasets/img" + idx + ".png";
imwrite(rgbImage, filename)

%%