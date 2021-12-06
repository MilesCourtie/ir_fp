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
        #Turn the robots cooridantes into grid X and Y values
        gridX = robot_controller.getX()/20 + 1
        gridY = robot_controller.getY()/20 + 1
        #Initialise the direction the robot is facing
        direction = "E"
        #Makes a 2D 20x20 array out of the map
    	grid = [[0] * (mapWidth/20 + 1)] * (mapHeight/20 + 1)
    	#Turns current grid position to explored
    	grid[gridX][gridY] = 1
        while(true):
            if robot_controller.is_blocked_in_right(20) and checkRightCell(direction, grid, gridX, gridY) == False:
                robot_controller.turn_right(90)
                direction = rotateRight(direction)
            elif robot_controller.is_blocked_in_front(20) and checkForwardCell(direction, grid, gridX, gridY) == False:
                robot_controller.drive_forward(20)
                if direction == "E":
                    gridX += 1
                elif direction == "S":
                    gridY += 1
                elif direction == "W":
                    gridX -= 1
                elif direction == "N":
                    gridY -= 1
                
            elif robot_controller.is_blocked_in_left(20) and checkLeftCell(direction, grid, gridX, gridY) == False:
                robot_controller.turn_left(90)
                direction = rotateLeft(direction)
            else
                robot_controller.stop_robot()
            #Move to new location and go again
            break
     
    #Change direction when rotating right      
    def rotateRight(direction):
        if direction == "E":
            direction = "S"
        elif direction == "S":
            direction = "W"
        elif direction == "W":
            direction = "N"
        elif direction == "N":
            direction = "E"
        return direction
    
    #Change direction when rotating left   
    def rotateLeft(direction):
        if direction == "E":
            direction = "N"
        elif direction == "S":
            direction = "E"
        elif direction == "W":
            direction = "S"
        elif direction == "N":
            direction = "W"
        return direction
    
    #Check if right cell is unexplored
    def checkRightCell(direction, grid, x, y):
        if direction == "E":
            if grid[x][y + 1] == 1:
                return True
        elif direction == "S":
            if grid[x - 1][y] == 1:
                return True
        elif direction == "W":
            if grid[x][y - 1] == 1:
                return True
        elif direction == "N":
            if grid[x + 1][y] == 1:
                return True
        return False
        
    #Check if left cell is unexplored
    def checkLeftCell(direction, grid, x, y):
        if direction == "E":
            if grid[x][y - 1] == 1:
                return True
        elif direction == "S":
            if grid[x + 1][y] == 1:
                return True
        elif direction == "W":
            if grid[x][y + 1] == 1:
                return True
        elif direction == "N":
            if grid[x - 1][y] == 1:
                return True
        return False
        
    #Check if front cell is unexplored
    def checkForwardCell(direction, grid, x, y):
        if direction == "E":
            if grid[x + 1][y] == 1:
                return True
        elif direction == "S":
            if grid[x][y + 1] == 1:
                return True
        elif direction == "W":
            if grid[x - 1][y] == 1:
                return True
        elif direction == "N":
            if grid[x][y - 1] == 1:
                return True
        return False     
	   
if __name__ == "__main__":
    try:
        spiral = Spiral()
        
        while not rospy.is_shutdown():
            spiral.traverse()
            break

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
