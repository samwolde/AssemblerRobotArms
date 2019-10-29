import math
import numpy
from robot_lib.msg import ArmAngles
from geometry_msgs.msg import Point
import constants as Constant

def transform(self, worldBase, newLocBase, point, rotAngle, axis=(0, 0, 1)):
    translationVec = (worldBase[0]-newLocBase[0], worldBase[1]-newLocBase[1], worldBase[2]-newLocBase[2])
    # translationVec = (-worldBase[0]+newLocBase[0], -worldBase[1]+newLocBase[1], -worldBase[2]+newLocBase[2])
    
    # traMat = tra.translation_matrix(translationVec).dot(tra.rotation_matrix(rotAngle, axis))
    traMat = tra.rotation_matrix(rotAngle, axis).dot(tra.translation_matrix(translationVec))

    return tuple(traMat.dot([point[0], point[1], point[2], 1])[:3])

def radToDeg(rad):
    return rad * 180 / math.pi

def degToRad(deg):
    return deg * math.pi/180

def armAnglesToDeg(finArmAngles):
    finalArmAngles = ArmAngles()
    finalArmAngles.armBase_armBaseTop = radToDeg(finArmAngles.armBase_armBaseTop)
    finalArmAngles.armBaseTop_arm1 = radToDeg(finArmAngles.armBaseTop_arm1)
    finalArmAngles.arm1_arm2 = radToDeg(finArmAngles.arm1_arm2)
    finalArmAngles.arm2_gripper = radToDeg(finArmAngles.arm2_gripper)          

    return finalArmAngles

def armAnglesToRad(finArmAngles):
    finalArmAngles = ArmAngles()
    finalArmAngles.armBase_armBaseTop = degToRad(finArmAngles.armBase_armBaseTop)
    finalArmAngles.armBaseTop_arm1 = degToRad(finArmAngles.armBaseTop_arm1)
    finalArmAngles.arm1_arm2 = degToRad(finArmAngles.arm1_arm2)
    finalArmAngles.arm2_gripper = degToRad(finArmAngles.arm2_gripper)          

    return finalArmAngles

def forwardKinematics(angles):
    angles.armBaseTop_arm1 -= math.pi/2
    angles.arm1_arm2 -= math.pi
    angles.armBaseTop_arm1 *= -1
    angles.arm1_arm2 *= -1

    l1 = Constant.ARM_1
    l2 = Constant.ARM_2

    l = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(angles.arm1_arm2))
    alpha = math.asin((l2*math.sin(angles.arm1_arm2)/l))
    nx = l * math.cos(angles.armBaseTop_arm1-alpha)
    ny = l * math.sin(angles.armBaseTop_arm1-alpha)

    x = nx * math.sin(-angles.armBase_armBaseTop)
    y = nx * math.cos(-angles.armBase_armBaseTop)
    z = ny

    return Point(x, y, z)

def inverseKinematics(endPose):
    l1 = Constant.ARM_1
    l2 = Constant.ARM_2
    ny = float(endPose.z)
    nx = math.sqrt(endPose.x**2 + endPose.y**2)

    gamma = math.atan2(ny, nx)
    beta = math.acos((l1**2 + l2**2 - nx**2 - ny**2)/(2*l1*l2))
    alpha = math.acos((nx**2 + ny**2 + l1**2 - l2**2)/(2*l1*math.sqrt(nx**2 + ny**2)))

    horizontalAngle = -math.atan2(endPose.x, endPose.y)     # negative sign used to reverse the direction of rotation
    
    return [ArmAngles(horizontalAngle, (math.pi/2)-(gamma+alpha), -beta+math.pi, (gamma+alpha)-(-beta+math.pi))]


# hor = 1 - right, 0 - no movement, -1 - left  Direction of arm motion is horizontal
# ver = 1 - up, 0 - no movement, -1 - down  Direction of arm motion is vertical
def alignGripperWithObject(angles, hor, ver):
    point = None
    if hor != 0:
        angles.armBase_armBaseTop += hor * Constant.HOR_STEP

    if ver != 0:
        point = forwardKinematics(angles)
        point.z += ver * Constant.VER_STEP

    return point 
# a = inverseKinematics(Point(0.15,0.1,0.2))[0]
# print(a)
# print(forwardKinematics(a))