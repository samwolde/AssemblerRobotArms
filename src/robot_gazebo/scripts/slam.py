#!/usr/bin/env python
import rospy

from robot_lib.msg import ArmAngles
from robot_lib.srv import GoTo, GoToPlanned
from geometry_msgs.msg import Vector3, Point
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import constants as Constant
import math
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

import tf
import time
import numpy as np

import constants as Constant
import search
from threading import Lock

subSonar = None
subOdom = None
pubSonar = None
vehicleControlSrv = None
pose = Pose2D()
odomAngle = 0
sonarAngle = 0
initCord = (0,0,0)
j = 0
graph = None
graphLock = Lock()
poseLock = Lock()
doSweep = True

def main():
    global subSonar, subOdom, pubSonar, vehicleControlSrv, graph

    rospy.init_node('SLAM')
    
    graph = search.callCreateEmptyGraph()

    subSonar = rospy.Subscriber('/' + Constant.ROBOT_NAME + '/sonar', Range, rangeCallback)
    subOdom = rospy.Subscriber('/wheely/steering/odom', Odometry, odomCallback)
    pubSonar = rospy.Publisher('/' + Constant.ROBOT_NAME + '/sonar/angle_cmd', Float32, queue_size=10)
    vehicleControlSrv = rospy.ServiceProxy('/wheely/nav/goto_srv', GoTo)
    moveVehicleSrv = rospy.Service('/' + Constant.ROBOT_NAME + '/motion_cmd_srv', GoToPlanned, moveVehicle)

    print("SLAM control node initialized")

    rospy.spin()

def setPose(x, y, theta):
    global pose
    pose.x = x
    pose.y = y
    pose.theta = theta

def getPose():
    return pose

def degToRad(ang):
    return ang * math.pi / 180

def radToDeg(ang):
    return ang * 180 / math.pi

def spinSonar():
    global sonarAngle
    sonarAngle += 0.3
    pubSonar.publish(sonarAngle)

def rangeCallback(message):
    global walls, wallsN, j, graph
    print("CAlled")
    distance = message.range
    if doSweep:
        if distance < 30:
            rotNo = int(math.floor(sonarAngle/360)) % Constant.RANGE_SAMPLES
            sonarAngRad = degToRad(sonarAngle)

            x = math.cos(sonarAngRad) * distance
            y = math.sin(sonarAngRad) * distance
            
            poseLock.acquire()
            curCord = getPose()
            poseLock.release()
            
            #initCord[2] = 0

            sonarLoc = transform((distance,0,0), (0.05, -0.27, 0), sonarAngRad)
            obstacleLoc = transform(sonarLoc, (curCord.x, curCord.y, 0), curCord.theta - (math.pi/2))
            
            #obstacleLocM = transform((obstacleLoc[0], obstacleLoc[1],0), (curCord.x, curCord.y, 0), initCord, curCord.theta)
            # obstacleLoc = transform((x, y, 0), (curCord.x, curCord.y, 0), initCord, curCord.theta)
            # print(obstacleLoc)
            obstacleLocN = search.roundPoint(obstacleLoc, Constant.INTERVAL)

            if obstacleLoc[0] < 10 and obstacleLoc[1] < 10 and obstacleLoc[0] > -10 and obstacleLoc[1] > -10:
                if obstacleLocN[0] < 20 and obstacleLocN[1] < 20 and obstacleLocN[0] > -20 and obstacleLocN[1] > -20:
                    graphLock.acquire()
                    modifyGraph(graph, obstacleLocN, rotNo)
                    graphLock.release()
                    j += 1

                # if rotNo == 0 and j>1000:
                #     showMap()            

        spinSonar()

def modifyGraph(graph, obstacleLocN, rotNo):
    node = graph[obstacleLocN]
    node.setState(Constant.NODE_WALL, rotNo)
    node.createBuffer()

def showMap(pathArr=None):
    arr = search.convertGraphToPlotArr(graph, False)
    # pathArr = search.findPath(graph, (0,1), (5, 5))
    # pathArr = search.convertPathToPlotArr(pathArr)

    import matplotlib.pyplot as plt
    plt.plot(arr[0], arr[1], 'bo')
    if pathArr is not None:
        plt.plot(pathArr[0], pathArr[1], 'ro')
    plt.show()

def moveVehicle(request):
    dest = request.destination
    planAndMove(dest)

def planAndMove(dest):
    global doSweep
    if int(dest.z) == 0:
        doSweep = False

    elif int(dest.z) == 2:
        showMap()
    else:
        doSweep = True
    # showMap()
    return
    start = getPose() 

    graphLock.acquire()   
    pathArr = search.findPath(graph, (start.x, start.y), (dest.x, dest.y))
    graphLock.release()

    print(dest)
    if not pathArr:
        return False

    startPoint = 1
    while True:
        loc = pathArr[startPoint]
        vehicleControlSrv(Point(loc[0], loc[1], 0))
        
        startPoint += 1
        print(startPoint)
        graphLock.acquire()
        pathValid = checkPathValidity(pathArr[startPoint:])
        if startPoint % 3 == 0 and not pathValid:
            print('Next partition')
            start = getPose() 
            print(start)   
            # pathArr = search.findPath(graph, (start.x, start.y), (dest.x, dest.y))
            pathArr = search.findPath(graph, (start.x, start.y), (dest.x, dest.y))
            if pathArr is None or pathArr is False or len(pathArr) < 1:
                showMap()
                return False
            startPoint = 1
            # showMap()
            graphLock.release()

        else:
            graphLock.release()


        if startPoint > len(pathArr) - 1:
            return True
        
def checkPathValidity(pathArr):
    for i in pathArr:
        print(graph[i].getName(), " = Buffer -", graph[i].isBuffer(), "Avail -", graph[i].isAvailable())
        if not graph[i].isAvailable():
            print(i)
            return False

    return True

def odomCallback(message):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    orientationEuler = tf.transformations.euler_from_quaternion(quaternion)
    
    poseLock.acquire()
    setPose(position.x, position.y, orientationEuler[2])
    poseLock.release()

def transform(point, traVec, rotAng):
    import numpy as np
    from tf import transformations as tra
    
    mt = tra.translation_matrix(np.multiply(traVec, -1))
    mr = tra.rotation_matrix(rotAng, (0, 0, 1))
    mat = mt.dot(mr)
    point = [point[0], point[1], point[2], 1]

    #transformed = mr.dot(point) + traVec
    #print(transformed)
    return tuple(mat.dot(point)[:3])

if __name__=='__main__':
    main()

#print(transform([3,6,0,1], (3,5,0), 0))