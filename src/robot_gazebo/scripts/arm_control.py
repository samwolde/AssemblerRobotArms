#!/usr/bin/env python
import rospy

from robot_lib.msg import ArmAngles
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import Bool
from gazebo_msgs.srv import GetJointProperties

from robot_lib.srv import MoveArm, MoveArmResponse, InverseKinematics, InverseKinematicsResponse,ForwardKinematics, GetArmAngles, ObserveEnv, ArmAnglesSrv, AlignGripper,MoveGripper, PickObj, MoveArmStraight, ObjDistanceSrv,MoveFrwd
from sensor_msgs.msg import Range

import constants as Constant
import math
import time

import extra_lib as ext_lib
import threading

class ArmControl:
    def __init__(self):
        self.armAngles = ArmAngles(0,20,45,45)
        self.colDetTime = 0
        self.currentDistance = 0

        self.moveArm = rospy.Service('/' + Constant.ROBOT_NAME + '/arm/move_arm', MoveArm, self.moveArm)
        self.moveArmSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/move_arm', MoveArm)
        self.getArmAnglesSrv = rospy.Service('/' + Constant.ROBOT_NAME + '/arm/get_arm_angles', GetArmAngles, self.getArmAngles)
        self.armAnglesCmdPub = rospy.Publisher('/' + Constant.ROBOT_NAME + '/arm/angles_cmd', ArmAngles, queue_size=10)
        self.armAnglesCmdSrv = rospy.Service('/' + Constant.ROBOT_NAME + '/arm/angles_cmd/srv', ArmAnglesSrv, self.pubArmAngles)
        
        self.moveArmStraight = rospy.Service('/' + Constant.ROBOT_NAME + '/arm/move_arm/straight', MoveArmStraight, self.moveArmStraight)
        self.moveArmStraightSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/move_arm/straight', MoveArmStraight)

        self.ikSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/lib/ik_srv', InverseKinematics)
        self.fkSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/lib/fk_srv', ForwardKinematics)
        self.getArmAnglesSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/get_arm_angles', GetArmAngles)
        self.alignGripperSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/gripper/align', AlignGripper)

        self.getJointProp = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)

        self.gripperColCheckSub = rospy.Subscriber('/'+ Constant.ROBOT_NAME + '/gripper/collision/check', Bool, self.checkCollision)
        self.toggleGripper = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/gripper/toggle', MoveGripper)
        self.objDistance = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/object/distance_srv', ObjDistanceSrv)

        self.loadObj = rospy.Service('/' + Constant.ROBOT_NAME + '/arm/pick', PickObj, self.pickObj)

        self.objDistance = rospy.Subscriber('/' + Constant.ROBOT_NAME + '/object/distance', Range, self.objDistanceCallback)
        
        self.moveFwd = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/object_pick/move_fwd', MoveFrwd)


    def pubArmAngles(self, message):
        try:
            self.armAnglesCmdPub.publish(message.angles)
            return True
        except Exception:
            return False

    def getArmAngles(self, req):
        angles = ArmAngles()
        angles.armBase_armBaseTop = ext_lib.radToDeg(self.getJointProp("armBase_armBaseTop").position[0])
        angles.armBaseTop_arm1 = ext_lib.radToDeg(self.getJointProp("armBaseTop_arm1").position[0])
        angles.arm1_arm2 = ext_lib.radToDeg(self.getJointProp("arm1_arm2").position[0])
        angles.arm2_gripper = ext_lib.radToDeg(self.getJointProp("palm_joint").position[0])

        self.armAngles = angles
        return angles

    def moveArm(self, request):
        if self.isPositionReachable(request):
            desiredPose = Point(request.x,request.y,request.z)
            finalArmAngles = self.changeArmPosition(desiredPose)
            print(finalArmAngles)
            if request.slow:
                if finalArmAngles is not None:
                    steps = self.getSteps(finalArmAngles)

                    for i in range(Constant.ARM_STEP):
                        self.increaseArmStep(steps)
                        self.armAnglesCmdPub.publish(self.armAngles)
                        time.sleep(0.05)

                        if i + 1 >= Constant.ARM_STEP:
                            return True
            else:
                self.armAnglesCmdPub.publish(finalArmAngles)
                self.armAngles = finalArmAngles
                return True

        return False

    def changeArmPosition(self, desiredPose):
        angles = self.ikSrv(desiredPose).angles
        leng = len(angles)

        if angles and leng>0:
            finArmAngles = ext_lib.armAnglesToDeg(self.getBestAngle(angles))
            if self.isAngleValid(finArmAngles):
                return finArmAngles
            else:
                print(finArmAngles)
                print("Invalid angle Position unreachable")

        return None

    def getSteps(self, fAng):
        iAng = self.getArmAngles(True)
        return ((fAng.armBase_armBaseTop-iAng.armBase_armBaseTop)/Constant.ARM_STEP, (fAng.armBaseTop_arm1-iAng.armBaseTop_arm1)/Constant.ARM_STEP,
                (fAng.arm1_arm2-iAng.arm1_arm2)/Constant.ARM_STEP, (fAng.arm2_gripper-iAng.arm2_gripper)/Constant.ARM_STEP)

    def increaseArmStep(self, steps):
        self.armAngles.armBase_armBaseTop += steps[0]
        self.armAngles.armBaseTop_arm1 += steps[1]
        self.armAngles.arm1_arm2 += steps[2]
        self.armAngles.arm2_gripper = (90-self.armAngles.armBaseTop_arm1) - self.armAngles.arm1_arm2

    def getBestAngle(self, angles):
        return angles[0]

    def isPositionReachable(self, finalPose):
        totArmLength = Constant.ARM_1 + Constant.ARM_2
        
        if finalPose.z > totArmLength - Constant.ARM_BASE_TOP:
            return False

        distance = math.sqrt(finalPose.x ** 2 + finalPose.y ** 2 + finalPose.z ** 2)

        if distance > totArmLength:
            return False

        return True

    def isAngleValid(self, angles, limit=Constant.ARM_ANGLE_LIMIT):
        angles = [angles.armBase_armBaseTop, angles.armBaseTop_arm1, angles.arm1_arm2, angles.arm2_gripper]
        for i in range(len(angles)):
            if angles[i] < limit[i][0] or angles[i] > limit[i][1]:
                return False
        
        return True


    def moveArmStraight(self, forward):
        print(forward)
        step = Constant.STRAIGHT_ARM_STEP
        if not forward.forward:
            step = -Constant.STRAIGHT_ARM_STEP
        z = 0
        for i in range(1):
            angles = self.getArmAnglesSrv(True).angles
            angles = ext_lib.armAnglesToRad(angles)
            point = self.fkSrv(angles).point
            z = point.z
            print("Old point")
            print(point)
            hyp = math.sqrt((point.x**2) + (point.y**2))
            nHyp = hyp + step
            
            sinVal = point.y/hyp
            cosVal = point.x/hyp
            
            nx = cosVal * nHyp
            ny = sinVal * nHyp

            point.x = nx
            point.y = ny
            point.z = z+0.01

            try:
                if self.isPositionReachable(point):
                    print("Reachable")
                    self.moveArmSrv(point.x, point.y, point.z, True)
                    time.sleep(0.05)
                    return True 

                return False
                # angles = self.ikSrv(point).angles
                print("New point")
                print(point)
                # angles = ext_lib.armAnglesToDeg(angles[0])
                # # self.setArmAngles(angles)

                # self.armAnglesCmdPub.publish(angles)

            except Exception, e:
                print(e)
                return False

    def checkCollision(self, msg):
        if msg:
            self.colDetTime = time.time()

    def objDistanceCallback(self, message):
        self.currentDistance = message.range

    def pickObj(self, msg):
        if msg.fromStart:
            self.moveArmSrv(0, 0.1, -0.1, True)

        self.alignGripperSrv(False)

        while True:
            if time.time() - self.colDetTime < 1:
                if self.toggleGripper('catch').reached:
                    # raw_input("Load stuff??")
                    time.sleep(0.5)
                    self.moveArmSrv(0, -0.1, 0.1, True)
                    self.toggleGripper('release')
                    print("Object loaded")
                    return True
                
                else:
                    self.toggleGripper('release')

            if self.currentDistance > 0.1:
                self.moveFwd(self.currentDistance)    # move the vehicle fwd by given distance

            else:
                if not self.moveArmStraightSrv(True).reached:
                    print("Maximum arm extension reached")
                    self.moveArmStraightSrv(False)      # First retract arm
                    self.moveFwd(self.currentDistance)    # move the vehicle fwd by given distance
                    print("Maximum arm extension reached: ", self.currentDistance)

        