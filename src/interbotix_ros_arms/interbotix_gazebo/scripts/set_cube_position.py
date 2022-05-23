#!/usr/bin/env python3
# Author: Baozhe
# Usage: to random set the cubes or set a cube to a target position
import rospy 
import random
import sys
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

# Global positions
POSITIONS = {
    "A": (0.30, 0.00),
    "B": (0.22, -0.10),
    "C": (0.22, 0.10),
    "D": (0.16, 0.00),
    "E": (0.22, 0.00)
}

def main():
    rospy.init_node('gazebo_set_cube_pose')

    if len(sys.argv) == 1:
        # random    
        cube_pos = []
        while len(cube_pos) <= 3:
            new_pos = (random.uniform(0.14, 0.3), random.uniform(-0.14, 0.14))
            ok = True
            for pos in cube_pos:
                if ((pos[0] - new_pos[0])**2 + (pos[1] - new_pos[1])**2)**.5 < 0.06:
                    ok = False
                    break
            if ok:
                cube_pos.append(new_pos)

        state_msgs = []
        for pos, name in zip(cube_pos, ["red_cube", "green_cube", "blue_cube"]):
            state_msg = ModelState()
            state_msg.model_name = name
            state_msg.pose.position.x = pos[0]
            state_msg.pose.position.y = pos[1]
            state_msg.pose.position.z = 0.01
            state_msgs.append(state_msg)

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            for msg in state_msgs:
                set_state( msg )

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    elif len(sys.argv) == 3: 
        # target cube -> target pos
        if sys.argv[1] == 'r':
            target_cube = "red_cube"
        elif sys.argv[1] == 'g':
            target_cube = "green_cube"
        elif sys.argv[1] == 'b':
            target_cube = "blue_cube"
        else:
            rospy.logerr("Incorrect cube name. Choose from 'r', 'g', 'b' ")
            exit(1)
        
        target_pos = sys.argv[2]
        # error checking 
        if target_pos not in POSITIONS.keys():
            rospy.logerr("Incorrect cube position. Choose from %s", str(list(POSITIONS.keys())))
            exit(1)
        
        state_msg = ModelState()
        state_msg.model_name = target_cube
        state_msg.pose.position.x = POSITIONS[target_pos][0]
        state_msg.pose.position.y = POSITIONS[target_pos][1]
        state_msg.pose.position.z = 0.01
        
        rospy.wait_for_service('gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(state_msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    else:
        rospy.logerr("Incorrect command line arguments")
        exit(1)
        
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

