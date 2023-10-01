#!/usr/bin/env python3
# coding: UTF-8
import sys
import rospy
import tf
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("joint_planner")
    
    # center of the scene
    center_x = 0.2
    center_y = 0.35
    center_z = 0.15
    
    ############ OBJECTS ###############
    # scene
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    
    p1 = PoseStamped()
    p1.header.frame_id = robot.get_planning_frame()
    p1.pose.position.x = center_x
    p1.pose.position.y = center_y
    p1.pose.position.z = center_z
    scene.add_box("scene", p1, (0.2, 0.2, 0.2))
    
    # ground
    p2 = PoseStamped()
    p2.header.frame_id = robot.get_planning_frame()
    p2.pose.position.x = 0.0
    p2.pose.position.y = 0.0
    p2.pose.position.z = 0.0
    scene.add_plane("ground", p2, normal=(0, 0, 1), offset=0)
    ####################################
    
    ########## PLANNING ################
    group_name = "ur5e"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # camera configuration
    r = 0.5
    r_xy = r * np.sin(np.radians(30))
    r_z = r * np.cos(np.radians(30))
    
    # HOME30
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = np.deg2rad(91.27)
    joint_goal[1] = np.deg2rad(-70.92)
    joint_goal[2] = np.deg2rad(-77.90)
    joint_goal[3] = np.deg2rad(-271.08)
    joint_goal[4] = np.deg2rad(-89.58)
    joint_goal[5] = np.deg2rad(1.41)
    
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    print("position1 done")
    time.sleep(3.0)
    
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = np.deg2rad(124.30)
    joint_goal[1] = np.deg2rad(-64.08)
    joint_goal[2] = np.deg2rad(-89.06)
    joint_goal[3] = np.deg2rad(-119.50)
    joint_goal[4] = np.deg2rad(60.42)
    joint_goal[5] = np.deg2rad(-84.89)
    
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    print("position2 done")
    time.sleep(3.0)
    
    
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = np.deg2rad(101.23)
    joint_goal[1] = np.deg2rad(-98.97)
    joint_goal[2] = np.deg2rad(-55.23)
    joint_goal[3] = np.deg2rad(-136.72)
    joint_goal[4] = np.deg2rad(68.19)
    joint_goal[5] = np.deg2rad(-44.68)
    
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    print("position3 done")
    time.sleep(3.0)
    
    
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = np.deg2rad(77.23)
    joint_goal[1] = np.deg2rad(-126.72)
    joint_goal[2] = np.deg2rad(-11.08)
    joint_goal[3] = np.deg2rad(-161.66)
    joint_goal[4] = np.deg2rad(83.83)
    joint_goal[5] = np.deg2rad(-11.31)
    
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    print("position4 done")
    
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = np.deg2rad(55.68)
    joint_goal[1] = np.deg2rad(-112.75)
    joint_goal[2] = np.deg2rad(-35)
    joint_goal[3] = np.deg2rad(-149.88)
    joint_goal[4] = np.deg2rad(102.75)
    joint_goal[5] = np.deg2rad(22.43)
    
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    print("position5 done")
    time.sleep(3.0)
    
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = np.deg2rad(43.81)
    joint_goal[1] = np.deg2rad(-79.89)
    joint_goal[2] = np.deg2rad(-77.21)
    joint_goal[3] = np.deg2rad(-122.14)
    joint_goal[4] = np.deg2rad(119.01)
    joint_goal[5] = np.deg2rad(71.46)
    
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    print("position6 done")
    time.sleep(3.0)
        
    
   
    
if __name__ == "__main__":
    main()
    