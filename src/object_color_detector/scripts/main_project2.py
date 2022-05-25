#!/usr/bin/env python3

# Author: Baozhe
# Usage: main file of the project 2

import rospy, sys
from object_color_detector.srv import DetectObjectSrv, DetectObjectSrvRequest, DetectObjectSrvResponse
from object_color_detector.srv import PickPlaceSrv, PickPlaceSrvRequest, PickPlaceSrvResponse
from geometry_msgs.msg import PoseStamped, Pose
import moveit_commander
from math import pi


#############################################################
## Global Variables #########################################
#############################################################
TASK_STACK = []
TASK_HOLD = 0
TASK_SORT = 1

TARGET_POSITIONS = {
    "A": (0.30, 0.00, 0.75),
    "B": (0.22, -0.10, 0.75),
    "C": (0.22, 0.10, 0.75),
    "D": (0.16, 0.00, 0.75),
    "E": (0.22, 0.00, 0.75)
}

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

# If the three are all empty then arm will hold
RED_OBJ_STACK = []
BLUE_OBJ_STACK = []
GREEN_OBJ_STACK = []

# Global MoveIt
ARM = 0
GRIPPER = 0
REFERENCE_FRAME = 0
END_EFFECTOR_LINK = 0

# Global detection service 
OBJECT_COLOR_DETECT = 0

# Global pick and place service
PICK_PLACE = 0
#############################################################


def pickPlaceCallBack(req):
    global ARM, GRIPPER

    if req.color == 0:
        # move to the red bucket
        target_joints = RED_BUCKET_JOINTS
    elif req.color == 1:
        # move to the green bucket
        target_joints = GREEN_BUCKET_JOINTS
    elif req.color == 2:
        # move to the blue bucket
        target_joints = BLUE_BUCKET_JOINTS
    else:
        # no such color 
        # error
        rospy.logerr("At [pickPlaceCallBack]: no such color")
        return PickPlaceSrvResponse(False)
    
    target_pose = req.targetPose # a PoseStamped

    try: 

        # set the current state as the start state
        ARM.set_start_state_to_current_state()

        # set the pose of the end effector
        ARM.set_pose_target(target_pose, END_EFFECTOR_LINK)

        # plan 
        plan_success, traj, planning_time, error_code = ARM.plan()

        # move
        ARM.execute(traj)
        rospy.sleep(0.5)

        # close the gripper
        GRIPPER.set_named_target('Closed')
        GRIPPER.go()
        rospy.sleep(0.5)

        # go to hold position
        ARM.set_joint_value_target(HOLD_JOINTS)
        ARM.go()
        rospy.sleep(0.5)

        # move to the bucket 
        ARM.set_joint_value_target(target_joints)
        ARM.go()
        rospy.sleep(0.5)

        # open the gripper
        GRIPPER.set_named_target("Open")
        GRIPPER.go()
        rospy.sleep(0.5)

        # reset 
        ARM.set_joint_value_target(HOLD_JOINTS)
        ARM.go()
        rospy.sleep(0.5)

        return PickPlaceSrvResponse(True)
    except: 
        return PickPlaceSrvResponse(False)

def cameraPointTransformToWorld(x_cam, y_cam, x_yellow, y_yellow, width_yellow, height_yellow):
    # transform from the camera pixel to the real world coordinates
    y_world = 0.15 - (x_cam - x_yellow) / width_yellow * 0.3
    x_world = 0.31 - (y_cam - y_yellow) / height_yellow * 0.18
    return (x_world, y_world)




def timerCommandCallBack(event):
    global TASK_STACK, ARM, GRIPPER
    global OBJECT_COLOR_DETECT, PICK_PLACE
    global RED_OBJ_STACK, BLUE_OBJ_STACK, GREEN_OBJ_STACK

    if len(TASK_STACK) == 0 and len(RED_OBJ_STACK) == 0 and \
        len(GREEN_OBJ_STACK) == 0 and len(BLUE_OBJ_STACK) == 0:
        TASK_STACK.append(TASK_HOLD)

    if TASK_STACK[0] == TASK_HOLD:
        # on this mode, we can do the detection
        res = OBJECT_COLOR_DETECT(0)
        print(res)
        x_yellow = res.yellowRectX
        y_yellow = res.yellowRectY
        width_yellow = res.yellowRectXWidth
        height_yellow = res.yellowRectYHeight
        if width_yellow == 0 or height_yellow == 0:
            return

        for pose in res.redObjList:
            x_world, y_world = \
                cameraPointTransformToWorld(
                    pose.position.x, 
                    pose.position.y, 
                    x_yellow, 
                    y_yellow, 
                    width_yellow, 
                    height_yellow)
            temp_pose = PoseStamped()
            temp_pose.header.frame_id = REFERENCE_FRAME
            temp_pose.header.stamp = rospy.Time.now()
            temp_pose.pose.position.x = x_world
            temp_pose.pose.position.y = y_world
            temp_pose.pose.position.z = 0.075
            temp_pose.pose.orientation.w = 0.7071068
            temp_pose.pose.orientation.y = 0.7071068
            RED_OBJ_STACK.append(temp_pose)

        for pose in res.greenObjList:
            x_world, y_world = \
                cameraPointTransformToWorld(
                    pose.position.x, 
                    pose.position.y, 
                    x_yellow, 
                    y_yellow, 
                    width_yellow, 
                    height_yellow)
            temp_pose = PoseStamped()
            temp_pose.header.frame_id = REFERENCE_FRAME
            temp_pose.header.stamp = rospy.Time.now()
            temp_pose.pose.position.x = x_world
            temp_pose.pose.position.y = y_world
            temp_pose.pose.position.z = 0.075
            temp_pose.pose.orientation.w = 0.7071068
            temp_pose.pose.orientation.y = 0.7071068
            GREEN_OBJ_STACK.append(temp_pose)

        for pose in res.blueObjList:
            x_world, y_world = \
                cameraPointTransformToWorld(
                    pose.position.x, 
                    pose.position.y, 
                    x_yellow, 
                    y_yellow, 
                    width_yellow, 
                    height_yellow)
            temp_pose = PoseStamped()
            temp_pose.header.frame_id = REFERENCE_FRAME
            temp_pose.header.stamp = rospy.Time.now()
            temp_pose.pose.position.x = x_world
            temp_pose.pose.position.y = y_world
            temp_pose.pose.position.z = 0.075
            temp_pose.pose.orientation.w = 0.7071068
            temp_pose.pose.orientation.y = 0.7071068
            BLUE_OBJ_STACK.append(temp_pose)
        
        # detection done
        TASK_STACK.pop()
        TASK_STACK.append(TASK_SORT)
        return 

    if TASK_STACK[0] == TASK_SORT:
        if len(RED_OBJ_STACK) == 0 and \
        len(GREEN_OBJ_STACK) == 0 and len(BLUE_OBJ_STACK) == 0:
            # no target
            rospy.logwarn("In SORT mode, but no target")
            TASK_STACK.pop()
            return
        
        # first do the red part
        for target in RED_OBJ_STACK:
            # 0 for red
            status = PICK_PLACE(target, 0)
            if not status:
                rospy.logerr("Error")

        for target in GREEN_OBJ_STACK:
            status = PICK_PLACE(target, 1)
            if not status:
                rospy.logerr("Error")
        
        for target in BLUE_OBJ_STACK:
            status = PICK_PLACE(target, 2)
            if not status:
                rospy.logerr("Error")
        
        # done 
        RED_OBJ_STACK = []
        GREEN_OBJ_STACK = []
        BLUE_OBJ_STACK = []
        TASK_STACK.pop()
        return

        
        









    # pick_place = rospy.ServiceProxy('pick_place', PickPlaceSrv)
    # target_pose = PoseStamped()
    # target_pose.header.frame_id = REFERENCE_FRAME
    # target_pose.header.stamp = rospy.Time.now()     
    # target_pose.pose.position.x = 0.28
    # target_pose.pose.position.y = 0
    # target_pose.pose.position.z = 0.075
    # target_pose.pose.orientation.w = 0.7071068
    # target_pose.pose.orientation.y = 0.7071068

    # target_color = 2

    # pick_place(target_pose, target_color)


def main():
    global ARM, GRIPPER, REFERENCE_FRAME, END_EFFECTOR_LINK
    global OBJECT_COLOR_DETECT, PICK_PLACE

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("warehouse_sort")
    ARM = moveit_commander.MoveGroupCommander("interbotix_arm")
    GRIPPER = moveit_commander.MoveGroupCommander("interbotix_gripper")

    OBJECT_COLOR_DETECT = rospy.ServiceProxy('/object_detect', DetectObjectSrv)

    # get the name of the end effector link
    END_EFFECTOR_LINK = ARM.get_end_effector_link()
                    
    # set the reference frame as world
    REFERENCE_FRAME = 'world'
    ARM.set_pose_reference_frame(REFERENCE_FRAME)
            
    # allow re-planning
    ARM.allow_replanning(True)
    
    # set allowed errors
    ARM.set_goal_position_tolerance(0.001)
    ARM.set_goal_orientation_tolerance(0.001)
    
    # set max velocity and accelation
    ARM.set_max_acceleration_scaling_factor(1.0)
    ARM.set_max_velocity_scaling_factor(1.0)

    # set gripper error
    GRIPPER.set_goal_joint_tolerance(0.001)

    # set max velocity and acceleration
    GRIPPER.set_max_acceleration_scaling_factor(1.0)
    GRIPPER.set_max_velocity_scaling_factor(1.0)

    # open the gripper
    GRIPPER.set_named_target('Open')
    GRIPPER.go()
    rospy.sleep(0.5)

    # go to hold position
    ARM.set_joint_value_target(HOLD_JOINTS)
    ARM.go()
    rospy.sleep(0.5)

    pick_place_service = rospy.Service('pick_place', PickPlaceSrv, pickPlaceCallBack)

    rospy.wait_for_service('pick_place')
    PICK_PLACE = rospy.ServiceProxy('pick_place', PickPlaceSrv)

    rospy.Timer(rospy.Duration(1), timerCommandCallBack)
    rospy.spin()

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass