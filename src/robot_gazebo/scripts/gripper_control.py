#!/usr/bin/env python
import rospy

from robot_lib.msg import GripperAngles
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Bool

from robot_lib.srv import MoveArm, MoveArmResponse, MoveGripper, MoveGripperResponse

import constants as Constant
import math
import time

'''
fingerLB - max = 90 and min = 0 (neg) --> increase - open
fingerLF - max = 90 and min = 0 (pos) --> increase - close
fingerRB - max = 90 and min = 0 (pos) --> increase - open
fingerRF - max = 90 and min = 0 (neg) --> increase - close
'''


class GripperCollision:
    def __init__(self):
        self.base = False
        self.finger1B = False
        self.finger1F = False
        self.finger2B = False
        self.finger2F = False

    def reset(self):
        self.base = False
        self.finger1B = False
        self.finger1F = False
        self.finger2B = False
        self.finger2F = False

    def checkBackFingerCol(self):
        return self.finger1B and self.finger2B

    def checkFrontFingerCol(self):
        return self.finger1F and self.finger2F


class GripperControl:
    angles = None

    def __init__(self):
        self.gripperCollision = GripperCollision()
        
        self.gripperToggle = rospy.Service('/' + Constant.ROBOT_NAME + '/gripper/toggle', MoveGripper, self.toggleGrip)

        self.collisionCheckPub = rospy.Publisher('/' + Constant.ROBOT_NAME + '/gripper/collision/check', Bool, queue_size=10)
        self.gripperColSub = rospy.Subscriber('/'+ Constant.ROBOT_NAME + '/gripper/collision', String, self.checkCollision)
        self.moveFingersPub = rospy.Publisher('/' + Constant.ROBOT_NAME + '/gripper/move_fingers', GripperAngles, queue_size=10)

        self.default()

    def grip(self):
        self.angles.palm_finger1 -= 1
        self.angles.palm_finger2 += 1

        return self.angles

    def release(self):
        self.angles.palm_finger1 += 1
        self.angles.palm_finger2 -= 1

        return self.angles

    def belowLimit(self):
        return self.angles.palm_finger1 < -5 or self.angles.palm_finger2 > 5

    def beyondLimit(self):
        return self.angles.palm_finger1 > 30 or self.angles.palm_finger2 < -30

    def default(self):
        self.angles = GripperAngles(30, -30)
        self.gripperCollision.reset()

    def radToDeg(self, rad):
        return rad * 180 / math.pi

    def checkCollision(self, col):
        cols = col.data.split("=")
        # print(col)
        for i in range(len(cols)):
            if len(cols[i]) <= 0:
                continue
            else:
                objs = cols[i].split("-")
                if not objs[0].startswith(Constant.ROBOT_NAME):
                    if "finger_one" in objs[1]:
                        self.gripperCollision.finger1B = True

                    elif "finger_two" in objs[1]:
                        self.gripperCollision.finger2B = True

                    elif "finger_one_tip" in objs[1]:
                        self.gripperCollision.finger1F = True

                    elif "finger_two_tip" in objs[1]:
                        self.gripperCollision.finger2F = True

        if self.gripperCollision.finger1B or self.gripperCollision.finger2B:
            self.collisionCheckPub.publish(True)
        
        else:        
            self.collisionCheckPub.publish(False)

    def toggleGrip(self, grip):
        innerGripped = 0
        outerGripped = 0
        self.gripperCollision.reset()

        if grip.action == 'catch':
            while True:
                self.moveFingersPub.publish(self.grip())
                time.sleep(0.03)
                if not self.gripperCollision.checkBackFingerCol() and self.belowLimit():
                    return False

                if self.gripperCollision.checkBackFingerCol() or self.gripperCollision.checkFrontFingerCol():
                    if innerGripped > 6:
                        break

                    innerGripped += 1

            print "Gripped"
            return True

        else:
            print("Release called")
            while not self.beyondLimit():
                self.moveFingersPub.publish(self.release())
                time.sleep(0.05)

            return True

        return False
        


# def main():
#     gripperControl = GripperControl()

#     rospy.init_node('gripper_controller')    
#     print("Gripper control initiated")
#     rospy.spin()
    

# if __name__=='__main__':
#     main()