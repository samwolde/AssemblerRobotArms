#!/usr/bin/env python
import rospy

from robot_lib.msg import ArmAngles
from geometry_msgs.msg import Vector3, Point

from robot_lib.srv import MoveArm, MoveArmResponse

import constants as Constant
import math

class ArmControl:
    def changeArmPosition(self, desiredPose):
        angles = self.inverseKinematics2(desiredPose)
        # finArmAngles = self.moveArm(desiredPose)

        # if finArmAngles:
        #     finArmAngles = self.getBestAngles(desiredPose, finArmAngles)
        # else:
        #     return False
        if angles and len(angles)>0:
            finArmAngles = angles[0]
            print(finArmAngles)
            finalArmAngles = ArmAngles()
            finalArmAngles.armBase_armBaseTop = self.radToDeg(finArmAngles[0])
            finalArmAngles.armBaseTop_arm1 = self.radToDeg(finArmAngles[1])
            finalArmAngles.arm1_arm2 = self.radToDeg(finArmAngles[2])
            finalArmAngles.arm2_gripper = 0

            return finalArmAngles

        return None

    def getBestAngles(self, finalPose, candidateAngles):
        if abs(candidateAngles[0][0]) > math.pi:
            minBase = candidateAngles[0][1]
        else:
            minBase = candidateAngles[0][0]
        # minBase - 90 deg --> since rotation starts from y-axis this moves the start to x-axis
        return (minBase - (math.pi/2), -((math.pi/2)-candidateAngles[1][1][0]), -candidateAngles[1][1][1], -(math.sqrt(candidateAngles[1][1][2]**2)))

    def isPositionReachable(self, finalPose):
        totArmLength = Constant.ARM_1 + Constant.ARM_2
        
        if finalPose.z > totArmLength - Constant.ARM_BASE_TOP:
            return False

        distance = math.sqrt(finalPose.x ** 2 + finalPose.y ** 2 + finalPose.z ** 2)

        if distance > totArmLength:
            return False

        return True

    def getInitHeight(self):
        x = Constant.MAIN_LENGTH

        return x/10 + x/6 + x/6 + x/20 + x/16 

    def getBestBaseAng(self, finalPose, finalBaseAngle):
        if finalPose.x >= 0 and finalPose.y >= 0:
            return finalBaseAngle

        elif finalPose.x < 0 and finalPose.y >= 0:
            return math.pi - finalBaseAngle

        elif finalPose.x < 0 and finalPose.y < 0:
            return math.pi + finalBaseAngle

        else:
            return -1 * finalBaseAngle

    def getArmBaseAng(self, finalPose):
        # return self.getBestBaseAng(finalPose, math.atan2(finalPose.y, finalPose.x))
        return math.atan2(finalPose.y, finalPose.x)

    def moveArm(self, finalPose):
        print("CAlled")
        if self.isPositionReachable(finalPose):
            armBaseAng = self.getArmBaseAng(finalPose)

            ny = float(finalPose.z)
            nx = math.sqrt(finalPose.x**2 + finalPose.y**2)

            # beta = theta2
            beta = math.acos(((Constant.ARM_1 ** 2) + (Constant.ARM_2 ** 2) - (nx ** 2) - (ny**2))/(2.0 * Constant.ARM_1 * Constant.ARM_2))
            # alpha = theta1
            alpha = math.acos(((nx**2) + (ny**2) + (Constant.ARM_1**2) - (Constant.ARM_2**2))/(2.0 * Constant.ARM_1 * math.sqrt((nx**2) + (ny**2))))
            gamma = math.atan2(ny, nx)

            if armBaseAng < 0:
                baseAngles = [armBaseAng + (2*math.pi), armBaseAng]
            else:
                baseAngles = [armBaseAng, armBaseAng - (2*math.pi)]
            
            armAngles = [(gamma-alpha, beta-math.pi, (gamma-alpha+beta-math.pi)), (gamma+alpha, math.pi-beta, (math.pi-beta - gamma-alpha))]

            return (baseAngles, armAngles)
        
        else:
            print("Position unreachable")
            return False

    def radToDeg(self, rad):
        return rad * 180 / math.pi

    
    def inverseKinematics(self, endPose):
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

    def inverseKinematics1(self, endPose):
        finalAngles = []

        width = 0.3
        depth =  width/6
        wheel_radius = width/4
        suspension_depth = wheel_radius * 1.3
        b = depth + suspension_depth + wheel_radius

        l1 = Constant.ARM_1
        l2 = Constant.ARM_2
        ny = float(endPose.z) + b
        nx = math.sqrt(endPose.x**2 + endPose.y**2)

        thetaT = math.acos(nx/math.sqrt((nx**2)+(ny**2)))
        theta1 = math.acos(((l1**2)+(nx**2)+(ny**2)-(l2**2))/(2*l1*math.sqrt((nx**2)+(ny**2)))) + thetaT

        theta2 = math.pi - math.acos(((l1**2)+(l2**2)-((nx**2)+(ny**2)))/(2*l1*l2))
        
        theta0 = [math.atan2(endPose.x, endPose.y), math.atan2(endPose.x, endPose.y) - 2*math.pi]

        return [(theta, theta1, theta2, 0) for theta in theta0]

    def inverseKinematics2(self, endPose):
        l1 = Constant.ARM_1
        l2 = Constant.ARM_2
        ny = float(endPose.z)
        nx = math.sqrt(endPose.x**2 + endPose.y**2)

        gamma = math.atan2(ny, nx)
        beta = math.acos((l1**2 + l2**2 - nx**2 - ny**2)/(2*l1*l2))
        alpha = math.acos((nx**2 + ny**2 + l1**2 - l2**2)/(2*l1*math.sqrt(nx**2 + ny**2)))

        horizontalAngle = -math.atan2(endPose.x, endPose.y)     # negative sign used to reverse the direction of rotation
        return [(horizontalAngle, (math.pi/2)-(gamma+alpha), -beta+math.pi, (gamma+alpha)-(-beta+math.pi))]

armAngles = ArmAngles(0,20,45,45)

armControl = ArmControl()
pub = None

def main():
    global pub
    
    alignGripper = AlignGripper()

    rospy.init_node('IK')
    
    service = rospy.Service('/' + Constant.ROBOT_NAME + '/arm/move_arm', MoveArm, moveArm)
    # getArmAnglesSrv = rospy.Service('/' + Constant.ROBOT_NAME + '/arm/get_arm_angles', GetArmAngles, getArmAngles)
    pub = rospy.Publisher('/' + Constant.ROBOT_NAME + '/arm/angles_cmd', ArmAngles, queue_size=10)
    print("Arm control initiated")
    rospy.spin()
    
def defaultPose():
    global pub, armAngles
    armAngles = ArmAngles(0, 20, 45, 45)

    pub.publish(armAngles)

def getSteps(fAng):
    iAng = armAngles
    return ((fAng.armBase_armBaseTop-iAng.armBase_armBaseTop)/Constant.ARM_STEP, (fAng.armBaseTop_arm1-iAng.armBaseTop_arm1)/Constant.ARM_STEP,
            (fAng.arm1_arm2-iAng.arm1_arm2)/Constant.ARM_STEP, (fAng.arm2_gripper-iAng.arm2_gripper)/Constant.ARM_STEP)

def increaseArmStep(steps):
    global armAngles

    armAngles.armBase_armBaseTop += steps[0]
    armAngles.armBaseTop_arm1 += steps[1]
    armAngles.arm1_arm2 += steps[2]
    armAngles.arm2_gripper = (90-armAngles.armBaseTop_arm1) - armAngles.arm1_arm2

def moveArm(request):
    print("New called")
    import time
    global pub, armAngles
    print(request)
    desiredPose = Vector3(request.x,request.y,request.z)
    
    finalArmAngles = armControl.changeArmPosition(desiredPose)

    if finalArmAngles is not None:
        steps = getSteps(finalArmAngles)

        for i in range(Constant.ARM_STEP):
            increaseArmStep(steps)
            
            pub.publish(armAngles)
            time.sleep(0.05)

            if i + 1 >= Constant.ARM_STEP:
                return True

    return False

class AlignGripper:
    def __init__(self):
        self.detectedObjLocSub = rospy.Subscriber("/" + Constant.ROBOT_NAME + '/detected_object_location', Point, self.objDetectedCallback)
        

    def objDetectedCallback(self, loc):
        diff = self.getDifference(loc)

        if diff[0] > 0:     # move arm to the right
            pass
        
        elif diff[0] < 0:   # move arm to the left
            pass

        if diff[1] > 0:     # move arm downwards
            pass

        elif diff[1] < 0:   # move arm upwards
            pass

    def getDifference(self, loc):
        centerX = Constant.CAMERA_WIDTH/2
        centerY = Constant.CAMERA_HEIGHT/2

        xDiff = loc.x - centerX
        yDiff = loc.y - centerY

        return xDiff, yDiff

def getArmAngles(angles):
    return armAngles

if __name__=='__main__':
    main()