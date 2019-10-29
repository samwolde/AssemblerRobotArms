import numpy as np
from tf import transformations as tra
import math

def transform(point, traVec, rotAng):
    mt = tra.translation_matrix(traVec)
    mr = tra.rotation_matrix(rotAng, (0, 0, 1))
    print(mt)
    print(mr)
    print(mr+mt)

    # point = [point[0], point[1], point[2], 1]
    #transformed = mr.dot(point)
    #print(transformed)

    #return tuple(transformed)

# print(transform((1,2,0), (3, 2, 0), 20))

def transform1(worldBase, newLocBase, point, rotAngle, axis=(0, 0, 1)):
    translationVec = (worldBase[0]-newLocBase[0], worldBase[1]-newLocBase[1], worldBase[2]-newLocBase[2])
    # translationVec = (-worldBase[0]+newLocBase[0], -worldBase[1]+newLocBase[1], -worldBase[2]+newLocBase[2])

    # traMat = tra.translation_matrix(translationVec).dot(tra.rotation_matrix(rotAngle, axis))
    traMat = tra.rotation_matrix(rotAngle, axis).dot(tra.translation_matrix(translationVec))
    print(traMat)
    return traMat.dot([point[0], point[1], point[2], 1])

# print(transform1((0,0,0), (5,5,0),(5,6,0),-math.pi/2))

import math
import constants as Constant


class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def inverseKinematics(endPose):
    finalAngles = []

    l1 = Constant.ARM_1
    l2 = Constant.ARM_2
    ny = float(endPose.z)
    nx = math.sqrt(endPose.x**2 + endPose.y**2)

    cosTheta2 = (nx**2 + ny**2 - (l1**2 + l2**2))/(2*l1*l2)
    sinTheta2 = math.sqrt(1 - cosTheta2**2)
    
    theta2 = [math.atan2(sinTheta2, cosTheta2), math.atan2(-sinTheta2, cosTheta2)]

    alpha = math.atan2(ny, nx)
    sinGamma = [l2*math.sin(math.pi-theta)/math.sqrt(nx**2+ny**2) for theta in theta2]
    gamma = [math.atan2(sinGam, math.sqrt(1-sinGam**2)) for sinGam in sinGamma]
    angles = []
    
    for i in range(len(theta2)):
        if theta2[i] < 0:
            angles.append([alpha+gamma[len(theta2)-1 - i], theta2[i]])
        else:
            angles.append([alpha-gamma[len(theta2)-1 - i], theta2[i]])

    theta0 = [math.atan2(endPose.x, endPose.y), math.atan2(endPose.x, endPose.y) - 2*math.pi]

    for angle in angles:
        for theta in theta0:
            ang = [theta]
            ang.extend(angle)
            finalAngles.append(tuple(ang))

    return finalAngles

def radToDeg(ang):
    return ang *180/math.pi

def changeAll(angles):
    finAngles = []
    for i in angles:
        finAngles.append((radToDeg(i[0]), radToDeg(i[1]), radToDeg(i[2])))

    return finAngles

# print(changeAll(inverseKinematics(Point(0.1,0.15,0.3))))

def inverseKinematics2(endPose):
    l1 = Constant.ARM_1
    l2 = Constant.ARM_2
    ny = float(endPose.z)
    nx = math.sqrt(endPose.x**2 + endPose.y**2)
    print(nx, ny)

    gamma = math.atan2(ny, nx)
    beta = math.acos((l1**2 + l2**2 - nx**2 - ny**2)/(2*l1*l2))
    alpha = math.acos((nx**2 + ny**2 + l1**2 - l2**2)/(2*l1*math.sqrt(nx**2 + ny**2)))

    return [(math.atan2(endPose.x, endPose.y), gamma+alpha, beta-math.pi, 0)]

def forwardKin(theta1, theta2):
    # theta1 = theta1 * math.pi/180
    # theta2 = theta2 * math.pi/180
    l1 = Constant.ARM_1
    l2 = Constant.ARM_2
    x = (l1 * math.cos(theta1)) + (l2*math.cos(theta1+theta2))
    y = (l1 * math.sin(theta1)) + (l2*math.sin(theta1+theta2))

    return (x, y)

# print(inverseKinematics2(Point(0.1,0.1,0)))
# print(forwardKin(1.0799136485055851, -2.1598272970111703))

x=raw_input("Retract arm") 
print("Retracted")
y =input("Are we done?")
print("Bye")