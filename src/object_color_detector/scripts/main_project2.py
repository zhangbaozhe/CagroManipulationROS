#!/usr/bin/env python3
# Author: Baozhe
# Usage: main file of the project 2

import rospy
from object_color_detector.srv import DetectObjectSrvRequest, DetectObjectSrvResponse

#############################################################
## Global Variables #########################################
#############################################################
TASK_STACK = []
TASK_PEEK = 0
TASK_SORT = 1
TASK_HOLD = 2
TARGET_POSITIONS = {
    "A": (0.30, 0.00),
    "B": (0.22, -0.10),
    "C": (0.22, 0.10),
    "D": (0.16, 0.00),
    "E": (0.22, 0.00)
}

# If the three are all empty then arm will hold
RED_OBJ_STACK = []
BLUE_OBJ_STACK = []
GREEN_OBJ_STACK = []
#############################################################


def timerServiceCallBack(event):
    pass

def timerCommandCallBack(event):
    pass


def main():
    rospy.init_node("warehouse_sort")
    rospy.Timer(rospy.Duration(2), timerServiceCallBack)
    rospy.Timer(rospy.Duration(2), timerCommandCallBack)
    rospy.spin()


if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass