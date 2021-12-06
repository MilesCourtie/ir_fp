#!/usr/bin/env python3.8
from planner.robot_controller import RobotController
import rospy
import random


class RandomWalk:
    def __init__(self):
        rospy.init_node("random_walk", anonymous=True)

        # use this object to control the robot
        self.robot_controller = RobotController()
	
        # Write any other attributes here

    def traverse(self):
        while(true):
            randomAngle = random.randint(0,360)
            self.robot_controller.drive_until_blocked(200)
            self.robot_controller.turn_left(randomAngle)
	    
	    
if __name__ == "__main__":
    try:
        randomWalk = RandomWalk()
        
        while not rospy.is_shutdown():
            randomWalk.traverse()
            break

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
