#!/usr/bin/env python
import rospy

from robot_lib.msg import ArmAngles
from geometry_msgs.msg import Vector3, Point
from gazebo_msgs.srv import GetJointProperties

from robot_lib.srv import MoveArm, MoveArmResponse, InverseKinematics, InverseKinematicsResponse,ForwardKinematics, GetArmAngles, ObserveEnv, ArmAnglesSrv, AlignGripper, MoveArmStraight
from sensor_msgs.msg import Range

import constants as Constant
import math
import time

import extra_lib as ext_lib
import threading


class AlignGrip:
    def __init__(self):
        self.armAngles = None
        self.called = False

        self.align = False
        self.gripperOnly = False
        self.aligningDone = False

        self.detectedObjLocSub = rospy.Subscriber("/" + Constant.ROBOT_NAME + '/detected_object_location', Point, self.tempObjDetCallback)
        # self.armAnglesCmdSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/angles_cmd/srv', ArmAnglesSrv)
        self.armAnglesCmdPub = rospy.Publisher('/' + Constant.ROBOT_NAME + '/arm/angles_cmd', ArmAngles, queue_size=10)

        self.getArmAnglesSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/get_arm_angles', GetArmAngles)
        self.ikSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/lib/ik_srv', InverseKinematics)
        self.fkSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/lib/fk_srv', ForwardKinematics)
        self.moveArmStraightSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/move_arm/straight', MoveArmStraight)

        self.alignGripper = rospy.Service('/' + Constant.ROBOT_NAME + '/gripper/align', AlignGripper, self.alignGripperCallback)

    def alignGripperCallback(self, msg):
        self.setArmAngles(None)
        self.gripperOnly = msg.onlyGripper
        self.align = True
        self.aligningDone = False
        while not self.aligningDone:
            pass

        return True

    def tempObjDetCallback(self, loc):
        if not self.called and self.align:
            print("Called")
            self.called = True
            self.objDetectedCallback(loc)
            # x = threading.Thread(target=self.objDetectedCallback, args=(loc,))
            # x.start()

    def objDetectedCallback(self, loc):
        # self.setArmAngles(None)
        print("start of alignment")
        print(self.armAngles)
        diff = self.getObjToCenterDistance(loc)
        print(diff)
        print(loc)
        if abs(diff[0]) < 10 and abs(diff[1]) < 10:
            self.called = False
            print("I am done")
            print("======================================")
            self.align = False
            self.gripperOnly = False
            self.aligningDone = True
            return

        angles = self.getArmAng()
        print("New angle")
        print(angles)
        newAngles = self.getNextAngles(angles, diff[0], diff[1], self.gripperOnly)

        if newAngles is None:
            self.moveArmStraightSrv(False)
            self.called = False
            return

        # change angles
        self.setArmAngles(newAngles)
        
        self.armAnglesCmdPub.publish(newAngles)
        # self.armAnglesCmdSrv(newAngles)
        self.called = False  

    def getObjToCenterDistance(self, loc):
        centerX = Constant.CAMERA_WIDTH/2
        centerY = Constant.CAMERA_HEIGHT/2

        xDiff = loc.x - centerX
        yDiff = loc.y - centerY

        return xDiff, -yDiff

    # hor = 1 - right, 0 - no movement, -1 - left  Direction of arm motion is horizontal
    def horAlign(self, angles, hor):
        step = Constant.HOR_STEP
        if hor != 0:
            if abs(hor) < 50:
                step = step/4
            angles.armBase_armBaseTop += -hor/abs(hor) * step      # negative for hor because the arm moves positively in counter clockwise direction 

        return angles

    # ver = 1 - up, 0 - no movement, -1 - down  Direction of arm motion is vertical
    def verAlign(self, angles, ver, gripperOnly=False):
        print("Vertical align")
        print(angles)
        step = 1
        if ver != 0:
            if abs(ver) < 50:
                step = 1

            if gripperOnly:
                angles.arm2_gripper += ver/abs(ver) * Constant.HOR_STEP * step    # negative for ver because the gripper moves positively in downwards 
                return angles

            else:
                angles = ext_lib.armAnglesToRad(angles)
                point = self.fkSrv(angles).point
                point.z += -ver/abs(ver) * Constant.VER_STEP * step       

                if point is None:
                    return [ext_lib.armAnglesToDeg(angles)]

                try:
                    return ext_lib.armAnglesToDeg(self.ikSrv(point).angles[0])
            
                except Exception:
                    return None

        return angles
    
    def makeGripperHor(self, newAngles):
        finGripAngle = -(newAngles.armBaseTop_arm1-90) - newAngles.arm1_arm2
        gripperAngleDiff = finGripAngle - newAngles.arm2_gripper
        newAngles.arm2_gripper += gripperAngleDiff * Constant.GRIPPER_STEP

        return newAngles

    def getNextAngles(self, angles, hor, ver, gripperOnly=False):
        angles = self.horAlign(angles, hor)
        print("Gripper only: ",gripperOnly)
        angles = self.verAlign(angles, ver, gripperOnly)
        
        if angles is not None:
            if not gripperOnly:
                return self.makeGripperHor(angles)

            return angles

        return None

    def getArmAng(self):
        if self.armAngles is not None:
            return self.armAngles
        else:
            return self.getArmAnglesSrv(True).angles

    def setArmAngles(self, angles):
        self.armAngles = angles