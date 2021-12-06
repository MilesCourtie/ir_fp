#!/usr/bin/env python3.8
from planner.robot_controller import RobotController
import rospy
import random


class Spiral:
    def __init__(self):
        rospy.init_node("spiral", anonymous=True)

        # use this object to control the robot
        self.robot_controller = RobotController()
	
        # Write any other attributes here

    #How to do cells and check if theyve been explored? 
    def traverse(self):
        while(true):
            if robot_controller.is_blocked_in_right(10) and right cell unexplored:
                robot_controller.turn_right(90)
            elif robot_controller.is_blocked_in_front(10) and front cell unexplored:
                robot_controller.drive_forward(10)
            elif robot_controller.is_blocked_in_left(10) and left cell unexplored:
                robot_controller.turn_left(90)
            else
                robot_controller.stop_robot()
            #Move to new location and go again
            break
            
	   
if __name__ == "__main__":
    try:
        spiral = Spiral()
        
        while not rospy.is_shutdown():
            spiral.traverse()
            break

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
