#!/usr/bin/env python3

from math import fmod
from robot_controller import RobotController
import rospy
import random
import enum
import numpy


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


# Write any other attributes here
robot_controller = RobotController()

map_width = 20 
map_height = 12
grid_sizex = map_width *  2 
grid_sizey = map_height * 2

pos_x = 10
pos_y = 6
direction = Direction(2)
grid = [[0] *grid_sizex for i in range(grid_sizey)]

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
    print("X:{}, Y:{}, val:{}".format(pos_x + fmod(direction.left().left().value, 2), pos_y + fmod(direction.left().value,2),grid[pos_x + int(fmod(direction.left().left().value, 2))][pos_y + int(fmod(direction.left().value,2))]))
    if grid[pos_x + int(fmod(direction.left().left().value, 2))][pos_y + int(fmod(direction.left().value,2))]:
        print("visited")
    return grid[pos_x + int(fmod(direction.left().left().value, 2))][pos_y + int(fmod(direction.left().value,2))]

def bsa():
    while True:
        print("blocked right?:{}".format(robot_controller.is_blocked_in_right()))
        if not robot_controller.is_blocked_in_right() and  not check_right_cell():
            robot_controller.turn_right(90)
            global direction
            direction = direction.right()
        elif not robot_controller.is_blocked_in_front() and not check_forward_cell():
            robot_controller.drive_forward(2)
            
            global pos_y, pos_x
            if direction == Direction.east:
                pos_x += 1
            elif direction == direction.south:
                pos_y += 1
            elif direction == direction.west:
                pos_x -= 1
            elif direction == direction.north:
                pos_y -+ 1
        elif not robot_controller.is_blocked_in_left and not check_left_cell():
            robot_controller.turn_left(90)
            direction = direction.left()
        else:
            robot_controller.stop_robot()
            print("Stopped")

if __name__ == "__main__":
    try:
        
        robot_controller.drive_until_blocked()
        while not rospy.is_shutdown():
            print('x:{},y:{}'.format(robot_controller.get_x(),robot_controller.get_y()))
            bsa()
            break

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
