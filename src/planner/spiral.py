#!/usr/bin/env python3

from math import fmod
from robot_controller import RobotController, RobotInterruptException
import rospy
import random
import enum
import numpy


#Initialise directions
class Direction(enum.Enum):
    east = -1
    south = 0
    west = 1
    north = 2

    def left(self):
        value = self.value
        if value == -1:
            return Direction(2)
        else:
            return Direction(value - 1)

    def right(self):
        value = self.value
        if value == 2:
            return Direction(-1)
        else:
            return Direction(value + 1)


robot_controller = RobotController("spiral")

#Initialise attributes
map_width = 21 
map_height = 13
grid_sizex = map_width *  2 
grid_sizey = map_height * 2

pos_x = 0
pos_y = 23
direction = Direction(-1)
grid = [[0] *grid_sizey for i in range(grid_sizex)]

# Mark starting point as explored
grid[pos_x][pos_y] = 1

def check_right_cell():
    print("Checking right")
    print("X:{}, Y:{}".format(pos_x + fmod(direction.left().value, 2), pos_y + fmod(direction.left().left().value,2)))
    if grid[pos_x + int(fmod(direction.left().value, 2))][pos_y + int(fmod(direction.left().left().value,2))]:
        print("visited")
    return grid[pos_x + int(fmod(direction.left().value, 2))][pos_y + int(fmod(direction.left().left().value,2))]

def check_left_cell():
    print("Checking left")
    print("X:{}, Y:{}, val:{}".format(pos_x + fmod(direction.right().value, 2), pos_y + fmod(direction.value,2),grid[pos_x + int(fmod(direction.right().value, 2))][pos_y + int(fmod(direction.value,2))]))
    if grid[pos_x + int(fmod(direction.right().value, 2))][pos_y + int(fmod(direction.value,2))]:
        print("visited")
    return grid[pos_x + int(fmod(direction.right().value, 2))][pos_y + int(fmod(direction.value,2))]


def check_forward_cell():
    print("checking forward")
    
    print("X:{}, Y:{}, val:{}".format(pos_x + fmod(direction.right().right().value, 2), pos_y + fmod(direction.right().value,2),grid[pos_x + int(fmod(direction.right().right().value, 2))][pos_y + int(fmod(direction.right().value,2))]))
    if grid[pos_x + int(fmod(direction.right().right().value, 2))][pos_y + int(fmod(direction.right().value,2))]:
        print("visited")
    return grid[pos_x + int(fmod(direction.right().right().value, 2))][pos_y + int(fmod(direction.right().value,2))]

def spiral():
    global direction
    global grid
    global pos_y, pos_x
    while not robot_controller.is_shutdown():
        grid[pos_x][pos_y] = 1
        print("--------------------")
        print("set: currx:{},curry:{}".format(pos_x,pos_y))
        print(direction)
        print("blocked right?:{}".format(robot_controller.is_blocked_right()))
        if (not robot_controller.is_blocked_right()) and  (not check_right_cell()):
            print('Turning right')
            robot_controller.turn_right(90)
            direction = direction.right()
            
        elif (not robot_controller.is_blocked_front()) and (not check_forward_cell()):
            robot_controller.drive_forward(0.5)
            #Change the direction variable to match the direction te robot is facing
            if direction == Direction.east:
                pos_x += 1
            elif direction == Direction.south:
                pos_y += 1
            elif direction == Direction.west:
                pos_x -= 1
            elif direction == Direction.north:
                pos_y -= 1
                
        elif (not robot_controller.is_blocked_left()) and (not check_left_cell()):
            robot_controller.turn_left(90)
            direction = direction.left()
            
        else:
            print("Finished spiral")

if __name__ == "__main__":
    try:
        print(len(grid[0]))
        print(len(grid))

        spiral()
        robot_controller.wait()
    except RobotInterruptException:
        pass
