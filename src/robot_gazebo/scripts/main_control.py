#!/usr/bin/env python
import rospy

import time
from robot_lib.srv import PickObj, ObserveEnv, BeginWork

'''
Steps of action
1. Make map of the current area and decide first destination
2. Scan the environment for any object
    until all seen object is loaded
    2.1 Move the vehicle to the first nearest object
    2.2 Grab and load object
'''

class MainControl:
    def __init__(self):
        # self.moveToDestination = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/move_to_destination', ObserveEnv)
        self.scanEnv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/observe', ObserveEnv)
        self.loadObj = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/pick', PickObj)
        # self.traverseObjs = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/object_pick/angle', PickObject)

        self.startMainControl = rospy.Service('/' + Constant.ROBOT_NAME + '/begin', BeginWork, self.startWork)

    def startWork(self, arg):
        pointsInMaze = []
        for i in range(len(pointsInMaze)):
            # self.moveToDestination(pointsInMaze[i][0], pointsInMaze[i][1])
            detObjLoc = self.scanEnv(True)
            
            if len(detObjLoc) > 0:
                self.traverseObjs(detObjLoc[0], detObjLoc[1])
                for i in range(len(detObjLoc[0]) - 1):
                    self.traverseObjs([], [])


def main():
    rospy.init_node('MainControl')

    mainControl = MainControl()

    print("Main control node initiated")

    try:
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Main control shutting down...")

if __name__=='__main__':
    main()

        
