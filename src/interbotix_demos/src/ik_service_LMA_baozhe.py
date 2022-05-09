#!/usr/bin/env python3

import rospy
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
import random


class GetIK(object):
    def __init__(self, group, ik_timeout=0.005, avoid_collisions=False):
        """
        :param str group: MoveIt! group name
        :param float ik_timeout: default timeout for IK
        :param int ik_attempts: default number of attempts
        :param bool avoid_collisions: if to ask for IKs that take
        into account collisions
        """
        rospy.loginfo("Initalizing GetIK...")
        self.group_name = group
        self.ik_timeout = ik_timeout
        self.avoid_collisions = avoid_collisions
        rospy.loginfo("Computing IKs for group: " + self.group_name)
        rospy.loginfo("With IK timeout: " + str(self.ik_timeout))
        rospy.loginfo("Setting avoid collisions to: " + str(self.avoid_collisions))
        self.ik_srv = rospy.ServiceProxy('/wx250s/compute_ik', GetPositionIK)
        rospy.loginfo("Waiting for compute_ik service...")
        self.ik_srv.wait_for_service()
        rospy.loginfo("Connected!")

    def get_ik(self, pose_stamped):
        """
        Do an IK call to pose_stamped pose.
        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        """
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(self.ik_timeout)
        req.ik_request.avoid_collisions = self.avoid_collisions
        req.ik_request.robot_state.joint_state.name = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        req.ik_request.robot_state.joint_state.position = [0., 0., 0., 0., 0., 0.]

        try:
            resp = self.ik_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code = 99999  # Failure
            return resp

if __name__ == '__main__':
    rospy.init_node('ik_service_demo', anonymous=True)

    ik_solver = GetIK("interbotix_arm")
    ITER_NUM = 1000
    average_time = 0.0
    success_rate = 0.0

    for i in range(ITER_NUM):
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()

        target.pose.position.x = random.uniform(0.0, 0.5)
        target.pose.position.y = random.uniform(0.0, 0.5)
        target.pose.position.z =  random.uniform(0.0, 0.5)
        target.pose.orientation.x = random.uniform(-0.5, 0.5)
        target.pose.orientation.y = random.uniform(-0.5, 0.5)
        target.pose.orientation.z = random.uniform(-0.5, 0.5)
        target.pose.orientation.w = 1.

        t_start = rospy.Time.now()
        res = ik_solver.get_ik(target)
        if res.error_code.val == 1:
            success_rate += 1
            print("Success")
        else:
            print("Failed")
            print(res)
        time_cost = (rospy.Time.now()-t_start).to_sec()/1000.
        average_time += time_cost
        print("Time cost:", time_cost, "ms")
    
    average_time /= ITER_NUM
    success_rate /= ITER_NUM
    print("AVG TIME:", average_time)
    print("SUCCESS RATE:", success_rate)
    with open("exp2_log_TRCKIK.csv", mode='a') as f:
        f.write(f"{average_time}, {success_rate}")
