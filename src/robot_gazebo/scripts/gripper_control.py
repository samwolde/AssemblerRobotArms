#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from robot_lib.msg import GripperAngles
import math
import re


class GripperController:
    def __init__(self):
        self.cur_status = "Closed"
        self.fingers_to_move = []
        self.desired_angles = GripperAngles()

        self.col_sub = rospy.Subscriber("/robot/gripper/collision", String, self.collision)
        self.pub = rospy.Publisher("/robot/gripper/move_fingers", GripperAngles, queue_size=10)
        self.command_sub = rospy.Subscriber("/robot/gripper/command", String, self.command_listener)

    def open(self):
        if self.cur_status == "Open":
            return
        elif self.cur_status == "Closed":
            self.fingers_to_move.append("finger_one")
            self.fingers_to_move.append("finger_two")
            self.fingers_to_move.append("finger_three")
            self.fingers_to_move.append("finger_four")
            self.move_palm(90)
            self.move_finger()

        elif self.cur_status == "Holding":
            self.fingers_to_move.append("finger_one_tip")
            self.fingers_to_move.append("finger_two_tip")
            self.fingers_to_move.append("finger_three_tip")
            self.fingers_to_move.append("finger_four_tip")
            self.move_finger_tip(90)  # ???

        self.cur_status = "Open"

    def close(self):
        if self.cur_status == "Closed":
            return
        else:  # Open
            self.fingers_to_move.append("finger_one")
            self.fingers_to_move.append("finger_two")
            self.fingers_to_move.append("finger_three")
            self.fingers_to_move.append("finger_four")
            self.move_finger(-45)
            self.move_palm(-90)
            self.cur_status = "Closed"

    def catch(self):
        self.fingers_to_move.append("finger_one_tip")
        self.fingers_to_move.append("finger_two_tip")
        self.fingers_to_move.append("finger_three_tip")
        self.fingers_to_move.append("finger_four_tip")
        self.move_finger_tip(-90)
        self.cur_status = "Holding"

    def collision(self, msg):

        match = re.search('data: "robot(.*)robot::', str(msg))
        if match:

            match1 = re.search('::(.*)::', match.group(1))
            if match1:

                if match1.group(1) in self.fingers_to_move:
                    self.fingers_to_move.remove(match1.group(1))
                    

    def move_palm(self, deg):

        j = 0
        if deg > 0:
            while j < deg:
                
                self.desired_angles.palm -= 1
                self.publish
                j += 1
        else:
             while j > deg:
                
                self.desired_angles.palm += 1
                self.publish
                j -= 1

    def move_finger(self, deg=45):

        j = 0
        if deg > 0:

            while len(self.fingers_to_move) != 0 and j < deg:

                for i in self.fingers_to_move:
                    self.finger_helper(i, 1)

                self.publish
                j += 1

        else:

            while len(self.fingers_to_move) != 0 and j > deg:

                for i in self.fingers_to_move:
                    self.finger_helper(i, -1)

                self.publish
                j -= 1

        self.clear_list

    def finger_helper(self, i, x):
        if i == "finger_one":
            self.desired_angles.palm_finger1 -= x
        elif i == "finger_two":
            self.desired_angles.palm_finger2 += x
        elif i == "finger_three":
            self.desired_angles.palm_finger3 += x
        elif i == "finger_four":
            self.desired_angles.palm_finger4 -= x

    def move_finger_tip(self, deg=90):

        j = 0

        if deg > 0:

            while len(self.fingers_to_move) != 0 and j < deg:

                for i in self.fingers_to_move:
                    self.finger_tip_helper(i, 1)

                self.publish
                j += 1

        else:

            while len(self.fingers_to_move) != 0 and j > deg:

                for i in self.fingers_to_move:
                    self.finger_tip_helper(i, -1)

                self.publish
                j -= 1

        self.clear_list

    def finger_tip_helper(self, i, x):
        if i == "finger_one_tip":
            self.desired_angles.finger1_tip -= x
            if self.desired_angles.finger1_tip == 0:
                self.fingers_to_move.remove("finger_one_tip")
        elif i == "finger_two_tip":
            self.desired_angles.finger2_tip += x
            if self.desired_angles.finger2_tip == 0:
                self.fingers_to_move.remove("finger_two_tip")
        elif i == "finger_three_tip":
            self.desired_angles.finger3_tip += x
            if self.desired_angles.finger3_tip == 0:
                self.fingers_to_move.remove("finger_three_tip")
        elif i == "finger_four_tip":
            self.desired_angles.finger4_tip -= x
            if self.desired_angles.finger4_tip == 0:
                self.fingers_to_move.remove("finger_four_tip")

    def speed(deg):
        pass

    @property
    def publish(self):
        self.pub.publish(self.desired_angles)
        rospy.sleep(0.1)

    @property
    def clear_list(self):
        while len(self.fingers_to_move) > 0:
            self.fingers_to_move.pop()

    def command_listener(self, msg):
        
        if msg.data == "open":
            self.open()
        elif msg.data == "catch":
            self.catch()
        elif msg.data == "close":
            self.close()


def main():
    rospy.init_node("gripper_controller")
    gripper_cnt = GripperController()

    # demo
    gripper_cnt.open()
    gripper_cnt.catch()
    gripper_cnt.open()
    gripper_cnt.close()

    rospy.spin()


if __name__ == "__main__":
    main()
