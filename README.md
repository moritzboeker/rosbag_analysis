# rosbag_analysis
## Analyses rosbags and informs the user about possible errors
Usage:
1. Create folder 'data' within 'analyse_rosbag'
2. Copy ROS bag file of your choice into 'data'
3. In launch-file edit the parameter 'rosbag_filepath' with the corresponding path to your bag file
4. Launch via $ roslaunch analyse_rosbag analyse_rosbag.launch

This ROS package analyses and evaluates the topics
- `/ipa_log` concerning trusty or untrusty localization, it plots the localization confidence
- `/tf` concerning transforms with frame_id 'odom_combined', it plots the xy position and yaw angle
