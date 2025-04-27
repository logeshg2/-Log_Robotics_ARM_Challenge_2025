### RoboCup ARM Challenge 2025

This is the main repo that contains the execution scripts for robocup arm challenge 2025

- [ ] Positions global declaration + ros / gazebo connection -> basically initialization
- [ ] Inverse kinematics with gripper control.
- [ ] Update UR5e robot's joint limits for collision free motion -> more specifically joint 2 or 3.
- [X] Camera to base_link transform -> (no need calibration) but we can add new link (need to research)
- [ ] Same as applied to set transform from base_link to gripper (same as above) [!IMP].
- [ ] Trajectory and collision avoidance for robot pick and place (using RRT algorithm).
- [ ] Adaptive gripper + force control for proper grasp.
- [ ] Object detection ( or segmenation for extracting pcd) + Pose estimation (using ICP registers)
- [ ] Need add-on for solov2 detector