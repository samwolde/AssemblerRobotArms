#!/usr/bin/env python
import rospy

from scan_environment import *
from align_gripper import *
from kinematics import *
from arm_control import *
from gripper_control import *

def main():
    rospy.init_node('ArmControl')

    armControl = ArmControl()
    alignGripper = AlignGrip()
    kinematics = Kinematics()
    scanEnv = ScanEnvironment()
    gripperControl = GripperControl()

    print("Arm control node initiated")
    
    try:
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Arm control shutting down...")

if __name__=='__main__':
    main()