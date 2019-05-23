#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from robot_lib.msg import GripperAngles
from robot_lib.srv import MoveGripper
import math
import re
import time

desired_angles = GripperAngles()

def test(): 
    
    rospy.wait_for_service('/robot/gripper/move_fingers')
    try:
        g_c = rospy.ServiceProxy('/robot/gripper/move_fingers', MoveGripper)
        
        print("open")
        g_c("open")
        time.sleep(5)
        
        print("catch")
        g_c("catch")
        time.sleep(5)
        
        print("open")
        g_c("open")
        time.sleep(5)
        
        print("close")
        g_c("close")
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    
 

if __name__ == "__main__":
    test()