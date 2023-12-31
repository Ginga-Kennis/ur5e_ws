# SETUP ur5e_ws
- git clone https://github.com/Ginga-Kennis/ur5e_ws.git
- cd $HOME/ur5e_ws
- rosdep update
- rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src
- catkin init
- catkin build
- source devel/setup.bash

## RUN REAL ROBOT
- ur5e moveit
  - roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.56.101
  - start external control program on TP
  - roslaunch ur5e_moveit_config moveit_planning_execution.launch
  - rviz
- robotiq gripper
  - rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
  - rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
  - rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py
- realsense camera
  - roslaunch realsense2_camera rs_rgbd.launch
  
## RUN SIMULATION
- roslaunch ur5e_control ur5e_sim_moveit.launch
