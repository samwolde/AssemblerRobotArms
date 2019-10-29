#!/usr/bin/env python
import rospy

import time
from robot_lib.srv import PickObj, ObserveEnv, BeginWork, GoTo,PickObject
from geometry_msgs.msg import Point
import constants as Constant

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
        self.goto = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/nav/goto_srv', GoTo)
        self.scanEnv = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/observe', ObserveEnv)
        self.loadObj = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/arm/pick', PickObj)
        self.transformObjsLoc = rospy.ServiceProxy('/' + Constant.ROBOT_NAME + '/object_pick/angle', PickObject)
        self.startMainControl = rospy.Service('/' + Constant.ROBOT_NAME + '/begin', BeginWork, self.startWork)

    def startWork(self, arg):
        print("Starting...")
        pointsInMaze = Constant.MAZE_ROOMS
        for i in range(len(pointsInMaze)):
            finPt = Point(pointsInMaze[i][0], pointsInMaze[i][1], 0)
            
            self.goto([finPt],True)
            detObjLoc = self.scanEnv(True)
            
            if len(detObjLoc) > 0:
                traPts = self.transformObjsLoc(detObjLoc[0], detObjLoc[1])
                
                for i in range(len(traPts)):
                    self.goto([traPts[i]],True)
                    self.loadObj(True)
        print("Ending...")
        
        return True

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

        
