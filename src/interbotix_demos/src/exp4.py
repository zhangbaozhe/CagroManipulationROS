#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians

class MoveAttachedObjectDemo:
    def __init__(self):
        # init move group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # init ros node
        rospy.init_node('moveit_attached_object_demo')
        
        # init scene
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                                
        # init arm group
        arm = MoveGroupCommander('interbotix_arm')
        
        # get the name of the end effector
        end_effector_link = arm.get_end_effector_link()
        
        # arm error
        arm.set_goal_joint_tolerance(0.001)

        # max velocity and acceleration
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        
        
        # remove the objects 
        scene.remove_attached_object(end_effector_link, 'tool')
        scene.remove_world_object('table') 

        # set the height of the table
        table_ground = 0.5
        
        # set the 3D size of the table and the tool
        table_size = [1.5*0.1, 1.5*0.3, 1.5*0.02]
        tool_size = [0.2, 0.02, 0.02]
        
        # set the position of the tool
        p = PoseStamped()
        p.header.frame_id = end_effector_link
        
        p.pose.position.x = tool_size[0] - 0.03
        p.pose.position.y = 0
        p.pose.position.z = -0.015
        p.pose.orientation.w = 1
        
        # attach the tool to the end effector
        scene.attach_box(end_effector_link, 'tool', p, tool_size)

        # add table to the scene 
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = 0.4
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        # below rotate the table along y-axis 90 degrees
        table_pose.pose.orientation.y = 0.707
        table_pose.pose.orientation.w = 0.707

        scene.add_box('table', table_pose, table_size)
        
        rospy.sleep(2)  

        # arm to home position
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)

        # set the plan interval to 10s
        arm.set_planning_time(10)

       
        # set the target pos of the arm
        # joint_positions = [0.00153, -0.37429, 0.38502, 0.012271, -0.86209, -0.00153]
        # arm.set_joint_value_target(joint_positions)
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'world'
        target_pose.pose.position.x = 0.15
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = table_ground + table_size[2] / 2.0
        # target_pose.pose.position.z = table_ground 
        target_pose.pose.orientation.w = 0.924
        target_pose.pose.orientation.y = -0.383
        # target_pose.pose.orientation.w = 1

        # set the current state as the start state
        arm.set_start_state_to_current_state()

        # set the pose of the end effector
        arm.set_pose_target(target_pose, end_effector_link)
        # plan
        plan_success, traj, planning_time, error_code = arm.plan()

        # move
        arm.execute(traj)
        rospy.sleep(5)
        
        arm.set_named_target('Home')
        arm.go()
        rospy.sleep(1)

        arm.set_named_target('Sleep')
        arm.go()
        rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveAttachedObjectDemo()

    
