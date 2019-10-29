#!/usr/bin/env python
import rospy

from robot_lib.msg import ArmAngles
from geometry_msgs.msg import Vector3, Point
from gazebo_msgs.srv import GetJointProperties

from robot_lib.srv import MoveArm, MoveArmResponse, InverseKinematics, InverseKinematicsResponse,ForwardKinematics, GetArmAngles, ObserveEnv, ArmAnglesSrv, AlignGripper
from sensor_msgs.msg import Range

import constants as Constant
import math
import time

import extra_lib as ext_lib
import threading

class Kinematics:
    def __init__(self):
        self.ikSrv = rospy.Service('/' + Constant.ROBOT_NAME + '/lib/ik_srv', InverseKinematics, self.inverseKinematics)
        self.fkSrv = rospy.Service('/' + Constant.ROBOT_NAME + '/lib/fk_srv', ForwardKinematics, self.forwardKinematics)

    def inverseKinematics(self, endPose):
        l1 = Constant.ARM_1
        l2 = Constant.ARM_2
        # print(endPose)
        endPose = endPose.point
        ny = float(endPose.z)
        nx = math.sqrt(endPose.x**2 + endPose.y**2)

        gamma = math.atan2(ny, nx)
        try:
            beta = math.acos((l1**2 + l2**2 - nx**2 - ny**2)/(2*l1*l2))
            alpha = math.acos((nx**2 + ny**2 + l1**2 - l2**2)/(2*l1*math.sqrt(nx**2 + ny**2)))

            horizontalAngle = -math.atan2(endPose.x, endPose.y)     # negative sign used to reverse the direction of rotation
            
            return InverseKinematicsResponse([ArmAngles(horizontalAngle, (math.pi/2)-(gamma+alpha), -beta+math.pi, (gamma+alpha)-(-beta+math.pi))])
        
        except:
            return None

    def forwardKinematics(self, angles):
        angles = angles.angles
        angles.armBaseTop_arm1 -= math.pi/2
        angles.arm1_arm2 -= math.pi
        angles.armBaseTop_arm1 *= -1
        angles.arm1_arm2 *= -1

        l1 = Constant.ARM_1
        l2 = Constant.ARM_2
        try:
            l = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(angles.arm1_arm2))
            alpha = math.asin((l2*math.sin(angles.arm1_arm2)/l))
            nx = l * math.cos(angles.armBaseTop_arm1-alpha)
            ny = l * math.sin(angles.armBaseTop_arm1-alpha)

            x = nx * math.sin(-angles.armBase_armBaseTop)
            y = nx * math.cos(-angles.armBase_armBaseTop)
            z = ny

            return Point(x, y, z)

        except:
            return None