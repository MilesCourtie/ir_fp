#!/usr/bin/env python3

from robot_controller import RobotController
import rospy
import random


class Zigzag:
    def __init__(self, robot_width):
        # use this object to control the robot
        self.robot_controller = RobotController()
        self.robot_width = robot_width

        # Write any other attributes here

    def traverse(self):
        # Dont know what the width should be
        while True:
            done = True
            # No clue what scale we are looking at
            self.robot_controller.drive_until_blocked()
            self.robot_controller.turn_right(90)

            if not self.robot_controller.is_blocked_in_front():
                done = False
                self.robot_controller.drive_forward(self.robot_width)

            self.robot_controller.turn_right(90)
            self.robot_controller.drive_until_blocked()
            self.robot_controller.turn_left(90)

            if not self.robot_controller.is_blocked_in_front():
                done = False
                self.robot_controller.drive_forward(self.robot_width)
                self.robot_controller.turn_left(90)

            if done:
                self.robot_controller.stop_robot()
                break


if __name__ == "__main__":
    try:
        zigzag = Zigzag(0.35)

        while not rospy.is_shutdown():
            zigzag.traverse()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
