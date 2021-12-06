#!/usr/bin/env python3.8
from planner.robot_controller import RobotController
import rospy
import random


class Zigzag:
    def __init__(self):
        rospy.init_node("zigzag", anonymous=True)

        # use this object to control the robot
        self.robot_controller = RobotController()
	
        # Write any other attributes here

    def traverse(self):
        #Dont know what the width should be
        robotWidth = 10
        while(true):
            done = True
            #No clue what scale we are looking at
            robot_controller.drive_until_blocked(500)
            robot_controller.turn_right(90)
            if robot_controller.is_blocked_in_front(10) == False:
                done = False
                robot_controller.drive_forward(robotWidth)
            robot_controller.turn_right(90)
            robot_controller.drive_until_blocked(500)
            robot_controller.turn_left(90)
            if robot_controller.is_blocked_in_front(10) == False:
                done = False
                robot_controller.drive_forward(robotWidth)
            robot_controller.turn_left(90)
            if done == True:
            	robot_controller.stop_robot()
            	break
	    
	    
if __name__ == "__main__":
    try:
        zigzag = Zigzag()
        
        while not rospy.is_shutdown():
            zigzag.traverse()
            break

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
