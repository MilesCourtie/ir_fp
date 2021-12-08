#!/usr/bin/env python3

from robot_controller import RobotController
import rospy
import random

robot = RobotController("random_walk")

def traverse():
    while not robot.is_shutdown():
        print("Driving")
        robot.drive_until_blocked()
        print("Turning")
        random_angle = random.randint(10, 180)
        robot.turn_left(random_angle)


if __name__ == "__main__":
    try:
        traverse()
        robot.wait()
    except rospy.ROSInterruptException:
        pass
