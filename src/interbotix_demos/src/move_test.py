#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from math import pi

HOLD_JOINTS = {
    'waist': 0 / 180 * pi, 
    'shoulder': -29 / 180 * pi, 
    'elbow': -36 / 180 * pi, 
    'forearm_roll': 0, 
    'wrist_angle': -83 / 180 * pi, 
    'wrist_rotate': 0
}

RED_BUCKET_JOINTS = {
    'waist': 90 / 180 * pi, 
    'shoulder': -26 / 180 * pi, 
    'elbow': -38 / 180 * pi, 
    'forearm_roll': 0, 
    'wrist_angle': -77 / 180 * pi, 
    'wrist_rotate': 0
}

BLUE_BUCKET_JOINTS = {
    'waist': 120 / 180 * pi, 
    'shoulder': -19 / 180 * pi, 
    'elbow': -33 / 180 * pi, 
    'forearm_roll': 0, 
    'wrist_angle': -76 / 180 * pi, 
    'wrist_rotate': 30 / 180 * pi
}

GREEN_BUCKET_JOINTS = {
    'waist': 65 / 180 * pi, 
    'shoulder': -21 / 180 * pi, 
    'elbow': -35 / 180 * pi, 
    'forearm_roll': 0, 
    'wrist_angle': -76 / 180 * pi, 
    'wrist_rotate': -25 / 180 * pi
}

def main():
    # initialize the API from move_group
    moveit_commander.roscpp_initialize(sys.argv)

    # init ros node
    rospy.init_node("exp3_grasp")

    # get arm group
    arm = moveit_commander.MoveGroupCommander("interbotix_arm")
    # get gripper group
    gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
                
    # get the name of the end effector link
    end_effector_link = arm.get_end_effector_link()
                    
    # set the reference frame as world
    reference_frame = 'world'
    arm.set_pose_reference_frame(reference_frame)
            
    # allow re-planning
    arm.allow_replanning(True)
    
    # set allowed errors
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.001)
    
    # set max velocity and accelation
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    # set gripper error
    gripper.set_goal_joint_tolerance(0.001)

    # set max velocity and acceleration
    gripper.set_max_acceleration_scaling_factor(0.3)
    gripper.set_max_velocity_scaling_factor(0.3)

    # open the gripper
    gripper.set_named_target('Open')
    gripper.go()
    rospy.sleep(1)

    # let the robot arm to the initial point
    # step 1
    rospy.loginfo("In step 1")
    arm.set_named_target('Home')
    arm.go()
    rospy.sleep(1)

    # step 2
    # let the arm move above the target object and open the gripper
    # target pose [0.3, 0.0, 0.0]
    # target size [0.05, 0.05, 0.05]
    rospy.loginfo("In step 2")
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()     
    target_pose.pose.position.x = 0.22
    target_pose.pose.position.y = -0.1
    target_pose.pose.position.z = 0.075
    target_pose.pose.orientation.w = 0.7071068
    target_pose.pose.orientation.y = 0.7071068

    # set the current state as the start state
    arm.set_start_state_to_current_state()

    # set the pose of the end effector
    arm.set_pose_target(target_pose, end_effector_link)

    # plan
    plan_success, traj, planning_time, error_code = arm.plan()

    # move
    arm.execute(traj)
    rospy.sleep(1)

    arm.set_joint_value_target(HOLD_JOINTS)
    arm.go()
    rospy.sleep(1)

    position = arm.get_current_pose()
    print(position)



    # close the gripper
    rospy.loginfo("In step 4")
    gripper.set_named_target('Closed')
    # joinnt_positions = [0.03375, -0.03375]  # TODO: force to move?

    # gripper.set_joint_value_target(joinnt_positions)
    gripper.go()
    rospy.sleep(1)
    
    arm.set_named_target('Home')
    arm.go()
    rospy.sleep(1)

    joints = arm.get_joints()
    print(joints)

    arm.set_joint_value_target(RED_BUCKET_JOINTS)
    arm.go()
    rospy.sleep(1)

    gripper.set_named_target("Open")
    gripper.go()
    rospy.sleep(1)

    rospy.loginfo("In step 1")
    arm.set_named_target('Home')
    arm.go()
    rospy.sleep(1)

    rospy.loginfo("In step 2")
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()     
    target_pose.pose.position.x = 0.28
    target_pose.pose.position.y = 0
    target_pose.pose.position.z = 0.075
    target_pose.pose.orientation.w = 0.7071068
    target_pose.pose.orientation.y = 0.7071068

    # set the current state as the start state
    arm.set_start_state_to_current_state()

    # set the pose of the end effector
    arm.set_pose_target(target_pose, end_effector_link)

    # plan
    plan_success, traj, planning_time, error_code = arm.plan()

    # move
    arm.execute(traj)
    rospy.sleep(1)

    # close the gripper
    rospy.loginfo("In step 4")
    gripper.set_named_target('Closed')
    # joinnt_positions = [0.03375, -0.03375]  # TODO: force to move?

    # gripper.set_joint_value_target(joinnt_positions)
    gripper.go()
    rospy.sleep(1)
    
    arm.set_named_target('Home')
    arm.go()
    rospy.sleep(1)

    joints = arm.get_joints()
    print(joints)

    arm.set_joint_value_target(BLUE_BUCKET_JOINTS)
    arm.go()
    rospy.sleep(1)

    gripper.set_named_target("Open")
    gripper.go()
    rospy.sleep(1)

    rospy.loginfo("In step 1")
    arm.set_named_target('Home')
    arm.go()
    rospy.sleep(1)

    rospy.loginfo("In step 2")
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()     
    target_pose.pose.position.x = 0.245
    target_pose.pose.position.y = 0.1
    target_pose.pose.position.z = 0.075
    target_pose.pose.orientation.w = 0.7071068
    target_pose.pose.orientation.y = 0.7071068

    # set the current state as the start state
    arm.set_start_state_to_current_state()

    # set the pose of the end effector
    arm.set_pose_target(target_pose, end_effector_link)

    # plan
    plan_success, traj, planning_time, error_code = arm.plan()

    # move
    arm.execute(traj)
    rospy.sleep(1)

    # close the gripper
    rospy.loginfo("In step 4")
    gripper.set_named_target('Closed')
    # joinnt_positions = [0.03375, -0.03375]  # TODO: force to move?

    # gripper.set_joint_value_target(joinnt_positions)
    gripper.go()
    rospy.sleep(1)
    
    arm.set_named_target('Home')
    arm.go()
    rospy.sleep(1)

    joints = arm.get_joints()
    print(joints)

    arm.set_joint_value_target(GREEN_BUCKET_JOINTS)
    arm.go()
    rospy.sleep(1)

    gripper.set_named_target("Open")
    gripper.go()
    rospy.sleep(1)

    arm.set_joint_value_target(HOLD_JOINTS)
    arm.go()
    rospy.sleep(1)

    position = arm.get_current_pose()
    print(position)

    # # step 3
    # # move to the grasping position
    # rospy.loginfo("In step 3")
    # target_pose = PoseStamped()
    # target_pose.header.frame_id = reference_frame
    # target_pose.header.stamp = rospy.Time.now()     
    # target_pose.pose.position.x = 0.23
    # target_pose.pose.position.y = 0
    # target_pose.pose.position.z = 0.15
    # target_pose.pose.orientation.w = 1.0

    # # set the current state as the start state
    # arm.set_start_state_to_current_state()

    # # set the pose of the end effector
    # arm.set_pose_target(target_pose, end_effector_link)

    # # plan
    # plan_success, traj, planning_time, error_code = arm.plan()

    # # move
    # arm.execute(traj)
    # rospy.sleep(1)

    # # step 4
    # # close the gripper
    # rospy.loginfo("In step 4")
    # gripper.set_named_target('Closed')
    # # joinnt_positions = [0.03375, -0.03375]  # TODO: force to move?

    # # gripper.set_joint_value_target(joinnt_positions)
    # gripper.go()
    # rospy.sleep(1)


    # # step 5
    # # move above
    # rospy.loginfo("In step 5")
    # target_pose = PoseStamped()
    # target_pose.header.frame_id = reference_frame
    # target_pose.header.stamp = rospy.Time.now()     
    # target_pose.pose.position.x = 0.3
    # target_pose.pose.position.y = 0
    # target_pose.pose.position.z = 0.2
    # target_pose.pose.orientation.w = 1.0

    # # set the current state as the start state
    # arm.set_start_state_to_current_state()

    # # set the pose of the end effector
    # arm.set_pose_target(target_pose, end_effector_link)

    # # plan
    # plan_success, traj, planning_time, error_code = arm.plan()

    # # move
    # arm.execute(traj)
    # rospy.sleep(1)



    # # step 6
    # # open the gripper
    # rospy.loginfo("In step 6")
    # gripper.set_named_target('Open')
    # gripper.go()
    # rospy.sleep(1)

    # # step 7
    # rospy.loginfo("In step 7")
    # arm.set_named_target('Home')
    # arm.go()
    # rospy.sleep(1)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)




if __name__ == '__main__':
    main()