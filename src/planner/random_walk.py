#!/usr/bin/env python3

from robot_controller import RobotController
import rospy
import random


class RandomWalk:
    def __init__(self):
        # rospy.init_node("random_walk", anonymous=True)

        # use this object to control the robot
        self.robot_controller = RobotController()

        # Write any other attributes here

    def traverse(self):
        while True:
            print("Driving")
            self.robot_controller.drive_until_blocked()
            print("Turing")
            random_angle = random.randint(10, 180)
            self.robot_controller.turn_left(random_angle)


if __name__ == "__main__":
    try:
        random_walk = RandomWalk()

        while not rospy.is_shutdown():
            random_walk.traverse()
            break

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
