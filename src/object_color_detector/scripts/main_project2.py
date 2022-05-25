#!/usr/bin/env python3

# Author: Baozhe
# Usage: main file of the project 2

import rospy, sys
from object_color_detector.srv import DetectObjectSrv, DetectObjectSrvRequest, DetectObjectSrvResponse
from object_color_detector.srv import PickPlaceSrv, PickPlaceSrvRequest, PickPlaceSrvResponse
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64
import moveit_commander
from math import pi


#############################################################
## Global Variables #########################################
#############################################################
TASK_STACK = []
TASK_HOLD = 0
TASK_SORT = 1


# Global camera parameters
K1 = 0
K2 = 0
B1 = 0
B2 = 0

TARGET_POSITIONS = {
    "A": (0.30, 0.00, 0.75),
    "B": (0.22, -0.10, 0.75),
    "C": (0.22, 0.10, 0.75),
    "D": (0.16, 0.00, 0.75),
    "E": (0.22, 0.00, 0.75)
}

HOLD_JOINTS = {
    'waist': 0 / 180 * pi, 
    'shoulder': -27 / 180 * pi, 
    'elbow': -3 / 180 * pi, 
    'forearm_roll': 0, 
    'wrist_angle': -115 / 180 * pi, 
    'wrist_rotate': 0
}

HOLD_POSE = PoseStamped()
HOLD_POSE.header.frame_id = 'world'
HOLD_POSE.pose.position.x = 0.15
HOLD_POSE.pose.position.y = 0.0
HOLD_POSE.pose.position.z = 0.3
HOLD_POSE.pose.orientation.y = 2**0.5/2.
HOLD_POSE.pose.orientation.w = 2**0.5/2.

RED_BUCKET_JOINTS = {
    'waist': 90 / 180 * pi, 
    'shoulder': -24 / 180 * pi, 
    'elbow': -28 / 180 * pi, 
    'forearm_roll': 0, 
    'wrist_angle': -86 / 180 * pi, 
    'wrist_rotate': 0
}

BLUE_BUCKET_JOINTS = {
    'waist': 115 / 180 * pi, 
    'shoulder': -24 / 180 * pi, 
    'elbow': -33 / 180 * pi, 
    'forearm_roll': 0, 
    'wrist_angle': -86 / 180 * pi, 
    'wrist_rotate': 0 / 180 * pi
}

GREEN_BUCKET_JOINTS = {
    'waist': 65 / 180 * pi, 
    'shoulder': -19 / 180 * pi, 
    'elbow': -25 / 180 * pi, 
    'forearm_roll': 0, 
    'wrist_angle': -84 / 180 * pi, 
    'wrist_rotate': -0 / 180 * pi
}

TARGET_Z = 0.085

# If the three are all empty then arm will hold
RED_OBJ_STACK = []
BLUE_OBJ_STACK = []
GREEN_OBJ_STACK = []

# Global MoveIt
ARM = 0
GRIPPER = 0
REFERENCE_FRAME = 0
END_EFFECTOR_LINK = 0

# Global gripper pub
GRIPPER_PUB = 0

# Global detection service 
OBJECT_COLOR_DETECT = 0

# Global pick and place service
PICK_PLACE = 0
#############################################################


def pickPlaceCallBack(req):
    global ARM, GRIPPER, GRIPPER_PUB

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
        # GRIPPER.set_named_target('Closed')
        # GRIPPER.go()
        GRIPPER_PUB.publish(Float64(-0.01))
        rospy.sleep(0.5)
        # GRIPPER.clear_pose_targets()

        # go to hold position
        # ARM.set_joint_value_target(HOLD_JOINTS)
        # ARM.go()
        # set the pose of the end effector
        ARM.set_pose_target(HOLD_POSE, END_EFFECTOR_LINK)
        # plan 
        plan_success, traj, planning_time, error_code = ARM.plan()
        # move
        rospy.sleep(0.5)

        # move to the bucket 
        ARM.set_joint_value_target(target_joints)
        ARM.go()
        rospy.sleep(0.5)

        # open the gripper
        # below will fail
        # GRIPPER.set_named_target("Open")
        # GRIPPER.go()
        # open raw
        GRIPPER_PUB.publish(Float64(-0.02))
        rospy.sleep(0.5)

        # reset 
        # ARM.set_joint_value_target(HOLD_JOINTS)
        # ARM.go()
        # set the pose of the end effector
        ARM.set_pose_target(HOLD_POSE, END_EFFECTOR_LINK)
        # plan 
        plan_success, traj, planning_time, error_code = ARM.plan()
        # move
        ARM.execute(traj)
        rospy.sleep(0.5)

        return PickPlaceSrvResponse(True)
    except: 
        return PickPlaceSrvResponse(False)

def cameraPointTransformToWorld(x_cam, y_cam, x_yellow, y_yellow, width_yellow, height_yellow):
    # transform from the camera pixel to the real world coordinates
    y_world = 0.15 - (x_cam - x_yellow) / width_yellow * 0.3
    x_world = 0.31 - (y_cam - y_yellow) / height_yellow * 0.18
    y_temp = K2 * x_cam + B2
    x_temp = K1 * y_cam + B1
    return ((x_world+x_temp)/2, (y_world+y_temp)/2)




def timerCommandCallBack(event):
    global TASK_STACK, ARM, GRIPPER, GRIPPER_PUB
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
            temp_pose.pose.position.z = TARGET_Z
            temp_pose.pose.orientation.w = 0.7071068
            temp_pose.pose.orientation.y = 0.7071068
            RED_OBJ_STACK.append(temp_pose)

        for pose in res.greenObjList:
            if pose.position.x < 100:
                continue
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
            temp_pose.pose.position.z = TARGET_Z
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
            temp_pose.pose.position.z = TARGET_Z
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



def main():
    global ARM, GRIPPER, GRIPPER_PUB, REFERENCE_FRAME, END_EFFECTOR_LINK
    global OBJECT_COLOR_DETECT, PICK_PLACE
    global K1, K2, B1, B2

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("warehouse_sort")
    
    K1 = rospy.get_param("/image/reg_x/K1")
    B1 = rospy.get_param("/image/reg_x/B1")
    K2 = rospy.get_param("/image/reg_y/K2")
    B2 = rospy.get_param("/image/reg_y/B2")

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
    ARM.set_max_acceleration_scaling_factor(0.5)
    ARM.set_max_velocity_scaling_factor(0.5)

    # set gripper error
    GRIPPER.set_goal_joint_tolerance(0.001)

    # set max velocity and acceleration
    GRIPPER.set_max_acceleration_scaling_factor(0.3)
    GRIPPER.set_max_velocity_scaling_factor(0.3)

    GRIPPER_PUB = rospy.Publisher('/wx250s/gripper/command', Float64, queue_size=1)
    while GRIPPER_PUB.get_num_connections() < 1:
        pass

    # open the gripper
    # GRIPPER.set_named_target('Open')
    # GRIPPER.go()
    GRIPPER_PUB.publish(Float64(-0.02))
    rospy.sleep(0.5)

    # go to hold position
    # ARM.set_joint_value_target(HOLD_JOINTS)
    # ARM.go()
    # set the pose of the end effector
    ARM.set_pose_target(HOLD_POSE, END_EFFECTOR_LINK)
    # plan 
    plan_success, traj, planning_time, error_code = ARM.plan()
    # move
    ARM.execute(traj)
    rospy.sleep(0.5)

    pick_place_service = rospy.Service('pick_place', PickPlaceSrv, pickPlaceCallBack)

    rospy.wait_for_service('pick_place')
    PICK_PLACE = rospy.ServiceProxy('pick_place', PickPlaceSrv)

    rospy.Timer(rospy.Duration(2), timerCommandCallBack)
    rospy.spin()

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass