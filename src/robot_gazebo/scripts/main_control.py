#!/usr/bin/env python
import rospy

from robot_lib.msg import ArmAngles
from geometry_msgs.msg import Vector3, Point
from gazebo_msgs.msg import LinkStates

import constants as Constant
import math
import time
from robot_lib.srv import MoveArm, MoveGripper, GoTo, GoToPlanned

from tf import transformations as tra

class MainControl:
    def __init__(self):
        pass

    def performMainAction(self):
        # self.moveVehicle(2)
        # self.moveVehicle(0)
        # self.controlVehicle()
        # self.moveVehicle(1)
        # self.controlArm()
        # self.controlGripper()
        # Arm - A-1,2,1
        # Gripper - G-C or G-R
        while True:
            act = raw_input("Enter action: ")
            act = act.split("|")
            part = act[0].strip()
            
            if part == "A":
                com = act[1].strip().split(",")
                com1 = [float(i.strip()) for i in com]
                
                com = tuple(com1)
                # com = self.transform1((0,0,0),(0,0,0), com, math.pi/2)
                print(com)

                self.controlArm(com)

            elif part == "G":
                com = act[1].strip()
                if com == "C":
                    com = "catch"
                elif com == "R":
                    com = "release"

                self.controlGripper(com)

            else:
                quit()
                
            # com = 
            # inp = raw_input("Enter choice + or -: ")
            # if inp == "+":
            #     gripperControl('catch')
            # else:
            #     gripperControl('open')
            #     print "Here"
            # print n
            # gripperControl(n)
        # while True:
        #     self.controlVehicle()
            
        #     # if arm can't reach the object return to control vehicle to move the vehicle closer
        #     self.controlArm()

        #     # if gripper can't grip the object return to the control arm function to fix arm location
        #     # self.controlGripper()

    # def moveVehicle(self, d):
    #     reached = planMotionSrv(Point(2, -5, d))
    #     print(reached)

    # def controlVehicle(self):
    #     global vehicleControl
    #     reached = vehicleControl(Point(0, -2, 0))
    #     print(reached)
    #     # find the current position of the vehicle
    #     # rotate it to face the direction of the destination
    #     # find the current position of the vehicle
    #     # move to the destination point leaving some distance for the arm to act
    #     # return true if succeed false if not.
    #     pass

    def controlArm(self, end):
        print(end)
        # global armControl
        
        # while getMainLoc() is None:
        #     pass
        # transformed = self.transform(Vector3(2.05-0.2, 0.368, 0.437))
        # print transformed
        reached = armControl(end[0], end[1], end[2], True)
        # reached = armControl(transformed.x, transformed.y, transformed.z)
        # print self.transform(Vector3(1.22, -1.31, 0.437))
        print(reached.reached)
        # print getMainLoc()
        # time.sleep(5)
        # transform the destination point to the arm's coordinate space
        # give the computed point to the arm controller
        # return true if arm reaches location false if not 
        

    def controlGripper(self, act):
        global gripperControl
        gripped = gripperControl(act)
        print gripped
        # get the state of the gripper (gripped object/ready to grip)
        # send the signal to gripper controller to grip/release the object
        # return true if object is gripped false if not
        pass

    def transform(self, point):
        import numpy as np
        armBaseLoc = getMainLoc()
        print point
        print armBaseLocr
        # return Vector3(point.x-armBaseLoc.position.x, point.y-armBaseLoc.position.y, point.z - armBaseLoc.position.z)
        angle = armBaseLoc.orientation.z
        translation = np.array([armBaseLoc.position.x, armBaseLoc.position.y, armBaseLoc.position.z])

        mz = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1]
        ])    

        transformed = mz.dot(np.array([point.x, point.y, point.z]) - translation)
        
        return Vector3(transformed[0], transformed[1], transformed[2])

    def transform1(self, worldBase, newLocBase, point, rotAngle, axis=(0, 0, 1)):
        translationVec = (worldBase[0]-newLocBase[0], worldBase[1]-newLocBase[1], worldBase[2]-newLocBase[2])
        # translationVec = (-worldBase[0]+newLocBase[0], -worldBase[1]+newLocBase[1], -worldBase[2]+newLocBase[2])
        
        # traMat = tra.translation_matrix(translationVec).dot(tra.rotation_matrix(rotAngle, axis))
        traMat = tra.rotation_matrix(rotAngle, axis).dot(tra.translation_matrix(translationVec))
        print(traMat)
        return tuple(traMat.dot([point[0], point[1], point[2], 1])[:3])

   
mainControl = MainControl()
vehicleControl = None
planMotionSrv = None
armControl = None
gripperControl = None
pub = None
mainLoc = None

armBaseHeight = 0
armBaseTopHeight = 0

def main():
    global pub, armControl, gripperControl, vehicleControl, planMotionSrv

    rospy.init_node('main_control')
    # rospy.wait_for_service('move_arm')
    # Constant.ROBOT_NAME = 'robot'
    vehicleControl = rospy.ServiceProxy('/wheely/nav/goto_srv', GoTo)
    armControl = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/move_arm', MoveArm)
    # gripperControl = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/gripper/toggle', ToggleGrip)
    gripperControl = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/gripper/toggle', MoveGripper)
    planMotionSrv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/motion_cmd_srv', GoToPlanned)
    # sub = rospy.Subscriber('/gazebo/link_states', LinkStates, updateMainLoc)

    mainControl.performMainAction()

    rospy.spin()
    
def callback(desiredPose):
    global pub

    # pub.publish(armControl.changeArmPosition(desiredPose))
    # armControl.changeArmPosition(desiredPose)

def updateMainLoc(links):
    # print "Here"
    # print links
    # time.sleep(5)
    names = links.name
    for i in range(len(names)):
        if Constant.ROBOT_NAME_1 + '::armBase' == names[i]:
            setMainLoc(links.pose[i])
            time.sleep(2)

def setMainLoc(loc):
    global mainLoc

    mainLoc = loc

    mainLoc.position.x = 0.8
    mainLoc.position.y = 0
    # mainLoc.position.z = 0.866
    mainLoc.position.z += Constant.ARM_BASE/2.0 + Constant.ARM_BASE_TOP
    
def getMainLoc():
    global mainLoc

    return mainLoc

if __name__=='__main__':
    main()

        
