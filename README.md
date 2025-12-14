# ros_pub
For build:
cd ~/ros2_ws
colcon build --packages-select pose_pubsub
. install/setup.bash

Instructions for one csv:
~/ros2_ws$ ros2 run pose_pubsub pose_to_path

~/ros2_ws$ ros2 run pose_pubsub pose_publisher --ros-args -p csv_path:=/home/helen/ros2_ws/src/pose_pubsub/data/all_gt_pose_aria.csv

~/ros2_ws$ rviz2 -d ~/ros2_ws/src/pose_pubsub/config/rviz/slam_player.rviz

Instructions for two csv files:
~/ros2_ws$ rviz2 # then add poses

~/ros2_ws$ ros2 launch pose_pubsub two_csv_viz.launch.py   csv1:=/home/helen/ros2_ws/src/pose_pubsub/data/all_gt_poses_exo2.csv   csv2:=/home/helen/ros2_ws/src/pose_pubsub/data/all_gt_poses_aria_new.csv   stride:=20     # Can change the colour of ARIA to blue after
