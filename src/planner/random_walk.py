#!/usr/bin/env python3.8
from planner.robot_controller import RobotController
import rospy


class RandomWalk:
    def __init__(self):
        rospy.init_node("random_walk", anonymous=True)

        # use this object to control the robot
        self.robot_controller = RobotController()

        # Write any other attributes here

    def some_method(self):
        # implement the method using the robot controller
        pass
