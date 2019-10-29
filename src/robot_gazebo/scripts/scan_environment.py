#!/usr/bin/env python
import rospy

from robot_lib.msg import ArmAngles
from geometry_msgs.msg import Vector3, Point
from gazebo_msgs.srv import GetJointProperties

from robot_lib.srv import MoveArm, MoveArmResponse, InverseKinematics, InverseKinematicsResponse,ForwardKinematics, GetArmAngles, ObserveEnv, ArmAnglesSrv, AlignGripper, ObjDistanceSrv, ObjDistanceSrvResponse
from sensor_msgs.msg import Range

import constants as Constant
import math
import time

import extra_lib as ext_lib
import threading

class ScanEnvironment:
    def __init__(self):
        self.currentDistance = 0
        self.lastObjDetTime = 0
        self.objLoc = None

        self.objDistance = rospy.Subscriber('/' + Constant.ROBOT_NAME + '/object/distance', Range, self.objDistanceCallback)
        self.detectedObjLocSub = rospy.Subscriber("/" + Constant.ROBOT_NAME + '/detected_object_location', Point, self.objDetectedCallback)

        self.observeEnvSrv = rospy.Service('/' + Constant.ROBOT_NAME + '/observe', ObserveEnv, self.observeEnv)
        # self.objDistanceSrv = rospy.Service('/' + Constant.ROBOT_NAME + '/object/distance_srv', ObjDistanceSrv, self.objDistance)
        
        # self.armAnglesCmdSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/angles_cmd/srv', ArmAnglesSrv)
        self.armAnglesCmdPub = rospy.Publisher('/' + Constant.ROBOT_NAME + '/arm/angles_cmd', ArmAngles, queue_size=10)

        self.moveArmSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/move_arm', MoveArm)
        self.getArmAnglesSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/get_arm_angles', GetArmAngles)
        self.fkSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/lib/fk_srv', ForwardKinematics)


        self.alignGripper = rospy.ServiceProxy("/" + Constant.ROBOT_NAME + "/gripper/align", AlignGripper)
        # self.pickObject = rospy.ServiceProxy("/" + Constant.ROBOT_NAME + "/object_pick/angle", PickObject)

    def objDetectedCallback(self, message):
        self.lastObjDetTime = time.time()
        self.objLoc = message
        print("Obj detected")

    def objDistanceCallback(self, message):
        self.currentDistance = message.range

    def observeEnv(self, msg):
        import copy
        speed = 0.03
        # put to default position
        self.moveArmSrv(0, -0.1, 0.1, True)
        time.sleep(1)
        armAngles = self.getArmAnglesSrv(True).angles

        curAng = armAngles.armBase_armBaseTop

        detObjLoc = []
        angles = []
        distances = []

        lastDetLoc = Constant.CAMERA_HEIGHT/2
        lastDetAng = -190

        while curAng < 180:
            curAng += 1
            detTime = self.lastObjDetTime

            if time.time() - detTime <= 1 and self.objLoc.x > 200 and self.objLoc.x < lastDetLoc and curAng - 5 > lastDetAng:
                print("Obj detected and found")
                al = self.alignGripper(True)
                # raw_input("Need comment")
                print("aligned", al)
                lastDetLoc = Constant.CAMERA_HEIGHT/2     # camera_height because the image is inverted 90 deg couter clockwise

                # Get the new arm angles after the gripper is aligned
                armAngles = self.getArmAnglesSrv(True).angles
                curAng = armAngles.armBase_armBaseTop
                lastDetAng = copy.copy(curAng)
                print("Current angle: ", curAng-5)
                print("Last det angle: ", lastDetAng)
                # calculate and append the current straight line distance from the object and horizontal angle
                dist = self.currentDistance
                if len(detObjLoc) > 0:
                    if curAng - detObjLoc[-1][-1] > 10:
                        angles.append(round(ext_lib.degToRad(self.fixAngleAxis(curAng)), 3))
                        distances.append(round(self.getStraightLineDist(dist, armAngles), 3))
                else:
                    angles.append(round(ext_lib.degToRad(self.fixAngleAxis(curAng)), 3))
                    distances.append(round(self.getStraightLineDist(dist, armAngles), 3))

            else:
                armAngles.armBase_armBaseTop = curAng
                print("Current angle: ", curAng-5)
                self.armAnglesCmdPub.publish(armAngles)
            
            time.sleep(speed)

        # while curAng < 270:
        #     curAng += 1
        #     armAngles.armBase_armBaseTop = curAng
        #     self.armAnglesCmdPub.publish(armAngles)
        #     time.sleep(speed)

        time.sleep(0.1)
        while curAng > -180:
            curAng -= 1
            armAngles.armBase_armBaseTop = curAng
            self.armAnglesCmdPub.publish(armAngles)
            time.sleep(speed)

        return [angles, distances]
        # if len(detObjLoc) > 0:
        #     self.pickObject(detObjLoc)

    def fixAngleAxis(self, angle):
        if angle < 0:
            return 360 + angle
        
        else:
            return angle

    def getStraightLineDist(self, dist, angles):
        pt = self.fkSrv(ext_lib.armAnglesToRad(angles)).point
        print("Distance: ",dist)
        height = pt.z + Constant.ARM_BASE_INIT_HEIGHT
        fromGripperTipToObj = math.sqrt((dist**2) - (height**2))
        fromGripperTipToArmBase = math.sqrt((pt.x**2) + (pt.y**2))

        return fromGripperTipToArmBase + fromGripperTipToObj