# SETUP ur5e_ws
## RUN REAL ROBOT
- roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.56.101
- start external control program on TP
- roslaunch ur5e_moveit_config moveit_planning_execution.launch
- rviz
  
## RUN SIMULATION
- roslaunch ur5e_control ur5e_sim_moveit.launch
